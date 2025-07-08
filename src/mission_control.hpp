// mission_control.hpp
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "lora_comm.hpp"
#include "LogInterface.hpp"
#include "PacketClasses.hpp" // calcCRC16, PacketBase, PacketCommand, etc.
#include "LoRaCore.hpp"
#include "wifi_manager.hpp"

#include "PacketAsaExchange.hpp"

// -----------------------------------------------------------------------------
// MissionControl
//
// Handles sending commands and telemetry requests over LoRa,
// processes incoming fragments, ASA speed‐adaptation handshake, and ACKs.
// -----------------------------------------------------------------------------
class MissionControl : public LogInterface
{
public:
    unsigned long lastPongHeartbeat = 0;    // время последней посылки
    unsigned long nextPongInterval = 15000; // первый интервал (мс)
    MissionControl()
    {
        loraComm = new LoRaComm(MY_DEVICE_ID, this);
    }

    ~MissionControl()
    {
        delete loraComm;
    }

    // Initialize LoRaComm
    void begin()
    {
        randomSeed(micros());
        WiFiManager::begin("Radiation", "polkalol", 3 * 3600, 30 * 60 * 1000);
        Serial.println(F("[MC] Starting Mission Control..."));
        if (!loraComm->begin())
        {
            addLog(F("[MC] ERROR: Failed to initialize LoRaComm"));
        }
        else
        {
            LoRaCore::init(loraComm); // запускает задачи на втором ядре
            addLog(F("[MC] LoRaComm initialized"));
            // sendInfoRequest(CMD_TELEMETRY_FRAGMENT); // запросить телеметрию:
            // sendInfoRequest(CMD_INFO_ENGINE);        // запросить “InfoEngine”:
            // sendInfoRequest(CMD_STATUS);             // запросить общий статус:
        }
    }

    // Main loop: check for and handle incoming LoRa frames
    void loop()
    {

        uint8_t senderId;
        PacketBase hdr;
        uint8_t payloadBuf[MAX_LORA_PAYLOAD];

        LoRaPacket pkt;

        if (LoRaCore::receive(pkt))
        {
            PacketBase hdr;
            hdr.packetType = pkt.packetType;
            hdr.packetId = pkt.packetId;
            hdr.payloadLen = pkt.payloadLen;
            if (pkt.receiverId == MISSION_CONTROL_ID)
            {
                // addLog("[" + String(currentProfileIndex) + "]  Received LoRaPacket: sender=" + String(pkt.senderId) +
                //        ", type=" + String((char)pkt.packetType) +
                //        ", id=" + String(pkt.packetId) +
                //        ", len=" + String(pkt.payloadLen));
                handlePacket(pkt.senderId, hdr, pkt.payload);
            }
        }

        // 🕒 Проверка потери пинга
        if (millis() - lastPingTime > checkInterval && currentProfileIndex > 0)
        {
            currentProfileIndex = 0;
            const auto &profile = loraProfiles[currentProfileIndex];

            addLog("⚠️ No ping in " + String(checkInterval / 1000) + " sec. Degrading LoRa profile to index " + String(currentProfileIndex) +
                   " (SF=" + profile.spreadingFactor + ", BW=" + profile.bandwidth + ", CR=" + profile.codingRate + ")");
            PacketAsaApprove resp;
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);
            resp.profileIndex = currentProfileIndex;
            LoRaCore::sendPacketBase(BOAT_DEVICE_ID, resp, &resp.profileIndex, false);
            vTaskDelay(pdMS_TO_TICKS(500));
            applyProfile(currentProfileIndex);

            lastPingTime = millis(); // Сбросим таймер
        }
        else
        {
            if (millis() - lastPongHeartbeat >= nextPongInterval)
            {
                sendHeartbeatPong();
            }
        }
    }

    // Send an arbitrary command string (e.g. “M:120”)
    void sendCommandString(const String &cmdStr)
    {
        if (cmdStr.length() > MAX_LORA_PAYLOAD)
        {
            addLog("[MC] ⚠️ Command too long (" + String(cmdStr.length()) +
                   " bytes), truncating to " + String(MAX_LORA_PAYLOAD));
        }

        size_t len = std::min<size_t>(cmdStr.length(), MAX_LORA_PAYLOAD);

        // 👇 Локальный буфер, живёт до вызова sendPacketBase → безопасно
        uint8_t tempBuf[MAX_LORA_PAYLOAD];
        memcpy(tempBuf, cmdStr.c_str(), len);

        PacketCommand cmd{};
        cmd.packetType = CMD_COMMAND_STRING;
        cmd.packetId = nextPacketId++;
        cmd.payloadLen = len;

        // 👇 Передаём стабильный буфер
        LoRaCore::sendPacketBase(BOAT_DEVICE_ID, cmd, tempBuf);
    }
    void sendHeartbeatPong()
    {
        PacketCommand pong{};
        pong.packetType = CMD_PONG;
        pong.packetId = nextPacketId++;
        pong.payloadLen = 0;

        LoRaCore::sendPacketBase(BOAT_DEVICE_ID, pong, nullptr, false);
        addLog(F("[MC] 🔄 Heartbeat PONG sent"));

        // cформируем новый случайный интервал 10–20 с
        nextPongInterval = random(10000UL, 20001UL);
        lastPongHeartbeat = millis();
    }

    void addLog(const String &msg) override
    {
        Serial.print('[');
        Serial.print(timeStr());
        Serial.print("] ");
        Serial.println(msg);
    }

    void applyProfile(uint8_t idx)
    {
        currentProfileIndex = idx;
        const auto &p = loraProfiles[idx];
        addLog("⚙️ Applying LoRa profile index " + String(idx) +
               " (SF=" + String(p.spreadingFactor) +
               ", BW=" + String(p.bandwidth) +
               ", CR=" + String(p.codingRate) + ")");
        loraComm->applySettings(p.spreadingFactor, p.codingRate, p.bandwidth);
    }

    void scanSpectrumCSV(float startMHz = 863.0f,
                         float stopMHz = 928.0f,
                         float stepMHz = 1.0f,
                         uint8_t samples = 10,
                         uint16_t dwellMs = 25)
    {
        // 1. Save current settings
        auto &radio = loraComm->getRadio(); // RadioLib instance
        float origFreq = LORA_FREQUENCY;

        addLog(F("=== LoRa spectrum scan CSV (freq_MHz,rssi_dBm) ==="));

        // 2. Sweep the band
        for (float f = startMHz; f <= stopMHz + 1e-6; f += stepMHz)
        {
            radio.setFrequency(f);
            // Continuous RX lets RSSI settle
            radio.startReceive(); // switch to RX mode

            float sum = 0;
            for (uint8_t i = 0; i < samples; i++)
            {
                delay(dwellMs);
                sum += radio.getRSSI(); // instant RSSI in dBm
            }
            float avg = sum / samples;
            Serial.printf("%.1f,%.1f\n", f, avg);
        }

        addLog(F("=== scan done ==="));

        // 3. Restore original settings
        radio.setFrequency(origFreq);
        radio.standby(); // back to standby; LoRaComm keeps config
        // если нужно перезапустить приём → LoRaCore::resumeRX();
    }

private:
    LoRaComm *loraComm;
    uint8_t nextPacketId = 0;
    unsigned long lastPingTime = 0;
    int currentProfileIndex = 0;
    unsigned long checkInterval = 35 * 1000;

    String timeStr() const
    {
        time_t now = time(nullptr);
        if (now > 1600000000)
        {
            struct tm *t = localtime(&now);
            char buf[9];
            strftime(buf, sizeof buf, "%H:%M:%S", t);
            return String(buf);
        }
        return F("00:00:00");
    }
    // Assemble telemetry JSON fragments
    void processTelemetryFragment(const String &frag)
    {
        // Format: “[i/N]…chunk…”
        int slashPos = frag.indexOf('/');
        int closePos = frag.indexOf(']');
        int idx = frag.substring(1, slashPos).toInt();
        int total = frag.substring(slashPos + 1, closePos).toInt();
        String chunk = frag.substring(closePos + 1);

        static String buffer;
        static int expected = 0;

        if (idx == 0)
        {
            buffer.clear();
            expected = total;
        }
        buffer += chunk;

        if (idx == expected - 1)
        {
            addLog(F("[MC] 🚀 Telemetry fully received:"));
            addLog(buffer);
            // TODO: deserializeJson(buffer) and handle data…
        }
    }
    // Handle incoming PacketBase + payloadBuf
    void handlePacket(uint8_t sender,
                      const PacketBase &hdr,
                      const uint8_t *buf)
    {
        lastPingTime = millis(); // Обновляем время последнего пинга
        float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();
        addLog("profile:" + String(currentProfileIndex) + " .RSSI:" + String(rssi) + "dBm, SNR=" + String(snr, 1) + "dB");
        switch (hdr.packetType)
        {
            // case CMD_ACK:
            // {
            //     if (hdr.payloadLen != sizeof(uint16_t))
            //     { // защита от пустых ACK-ов
            //         addLog("[MC] ⚠️ ACK with invalid size" + String(hdr.payloadLen) +
            //                " from " + String(sender) + ", expected " + String(sizeof(uint16_t)));
            //         break;
            //     }
            //     uint16_t ackedId;
            //     memcpy(&ackedId, buf, sizeof(ackedId));
            //     addLog("[MC] ACK for packet ID: " + String(ackedId));
            //     break;
            // }

        case CMD_TELEMETRY_FRAGMENT:
        {
            String fragment(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            processTelemetryFragment(fragment);
            break;
        }
        case CMD_REQUEST_ASA:
        case CMD_REPOSNCE_ASA:
        {
            uint8_t profileIndex = 0;
            if (!parseAsaPacket(hdr, buf, profileIndex))
            {
                addLog("1 [MC] Invalid ASA packet received");
                break;
            }
            // Ответим только на CMD_REQUEST_ASA
            if (hdr.packetType == CMD_REQUEST_ASA)
            {
                sendAsaResponse(nextPacketId++, profileIndex, sender);
                vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(300));
                addLog("3 [MC] ASA Response sent");
            }
            else
            {
                addLog("4 [MC] ASA Response received, applying profile index " + String(profileIndex));
            }
            vTaskDelay(pdMS_TO_TICKS(100)); // Ждем, чтобы пакет ушел
            applyProfile(profileIndex);
            break;
        }

        case CMD_PING:
        {
            PacketCommand pong;
            pong.packetType = CMD_PONG;
            pong.packetId = nextPacketId++;
            pong.payloadLen = 0;
            // loraComm->sendPacket(pong, sender);
            LoRaCore::sendPacketBase(sender, pong, nullptr, false);
            addLog("Pong->BOAT");
            break;
        }
        case CMD_RSSI_REPORT:
        {
            addLog("Got CMD_RSSI_REPORT");
            PacketRssiReport rpt{};
            rpt.packetType = hdr.packetType;
            rpt.packetId = hdr.packetId;
            rpt.payloadLen = hdr.payloadLen;

            memcpy(reinterpret_cast<uint8_t *>(&rpt) + sizeof(PacketBase), buf, hdr.payloadLen);

            addLog("📥 RSSI Report: raw=" + String(rpt.rawRssi) + ", smoothed=" + String(rpt.smoothedRssi));
            break;
        }
            // case CMD_REQUEST_ASA:
            // {

            //     // 1. Проверка длины (int + int + float)
            //     if (hdr.payloadLen != sizeof(uint8_t))
            //     {
            //         addLog("[MC] Invalid ASA packet size");
            //         break;
            //     }

            //     if (abs(rssi / currentProfileIndex) < 5)
            //     {
            //         addLog("[MC] Received ASA response with SNR=" + String(snr, 1) +
            //                ", RSSI=" + String(rssi) + "dBm, but no change in profile index.abs(rssi/currentProfileIndex)=" + String(abs(rssi / currentProfileIndex)) + "< 10");
            //         break;
            //     }

            //     // 2. Прочитать поля из буфера
            //     PacketAsaRequest req;
            //     req.packetType = CMD_REQUEST_ASA;
            //     req.packetId = hdr.packetId;
            //     req.payloadLen = hdr.payloadLen;

            //     uint8_t profileIndex = buf[0];
            //     const auto &profile = loraProfiles[profileIndex];
            //     addLog("[MC] Received ASA request from sender " + String(sender) +
            //            ", packetId=" + String(hdr.packetId) +
            //            ", payloadLen=" + String(hdr.payloadLen) + " (profileIndex=" + String(profileIndex) + " profile: SF=" + String(profile.spreadingFactor) + ", BW=" + String(profile.bandwidth) + ", CR=" + String(profile.codingRate) + ")");
            //     // Формируем ответ
            //     PacketAsaApprove resp{};
            //     resp.packetType = CMD_REPOSNCE_ASA;
            //     resp.packetId = nextPacketId++;
            //     resp.payloadLen = sizeof(uint8_t);
            //     resp.profileIndex = profileIndex;

            //     loraComm->sendPacket(resp, sender);
            //     addLog("⚙️ MC: Approving and applying LoRa profile index " + String(profileIndex));

            //     delay(50);
            //     applyProfile(profileIndex);
            //     break;
            // }

        default:
            // Unhandled packet types …
            addLog("[MC] Unhandled packet type: [" + String((char)hdr.packetType) +
                   "][" + String((int)hdr.packetType) + "], sender=" + String(sender) +
                   ", payloadLen=" + String(hdr.payloadLen));
            break;
        }

        if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG && hdr.packetType != CMD_REQUEST_ASA)
        {
            PacketAck ackOut{};
            ackOut.packetType = CMD_ACK;
            ackOut.packetId = nextPacketId++;
            ackOut.ackedId = hdr.packetId;
            ackOut.payloadLen = sizeof(ackOut.ackedId);

            uint8_t ackBuf[sizeof(ackOut.ackedId)];
            memcpy(ackBuf, &ackOut.ackedId, sizeof(uint16_t));

            LoRaCore::sendPacketBase(sender, ackOut, ackBuf, false);
            addLog("[MC] 📨 Sent ACK for packet ID " + String(hdr.packetId));
        }
    }
    // MissionControl.hpp
    void sendInfoRequest(CommandType what)
    {
        // что именно спрашиваем: T/I/S/F/G/D/K…
        PacketCommand cmd{};
        cmd.packetType = CMD_REQUEST_INFO;
        cmd.packetId = nextPacketId++;
        cmd.payloadLen = 1;
        // первый байт payload — нужный код
        uint8_t code = what;
        LoRaCore::sendPacketBase(BOAT_DEVICE_ID, cmd, &code);
        addLog("[MC] Sent P(request info=" + String((char)what) + ") to boat");
    }
};
