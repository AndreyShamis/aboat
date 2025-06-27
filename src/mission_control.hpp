// mission_control.hpp
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "lora_comm.hpp"
#include "LogInterface.hpp"
#include "PacketClasses.hpp" // calcCRC16, PacketBase, PacketCommand, etc.

// -----------------------------------------------------------------------------
// MissionControl
//
// Handles sending commands and telemetry requests over LoRa,
// processes incoming fragments, ASA speed‐adaptation handshake, and ACKs.
// -----------------------------------------------------------------------------
class MissionControl : public LogInterface
{
public:
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
        Serial.println(F("[MC] Starting Mission Control..."));
        if (!loraComm->begin())
        {
            addLog(F("[MC] ERROR: Failed to initialize LoRaComm"));
        }
        else
        {
            addLog(F("[MC] LoRaComm initialized"));
            sendInfoRequest(CMD_TELEMETRY_FRAGMENT); // запросить телеметрию:
            sendInfoRequest(CMD_INFO_ENGINE);        // запросить “InfoEngine”:
            sendInfoRequest(CMD_STATUS);             // запросить общий статус:
        }
    }

    // Main loop: check for and handle incoming LoRa frames
    void loop()
    {
        uint8_t senderId;
        PacketBase hdr;
        uint8_t payloadBuf[MAX_LORA_PAYLOAD];

        if (loraComm->parseReceivedPacket(senderId, hdr, payloadBuf))
        {
            // Собираем сообщение в один String
            String msg = "[MC][" + String(currentProfileIndex) + "] Received packet:" + hdr.toString() + ", from=" + String(senderId) + " payloadBuf:" + String(reinterpret_cast<const char *>(payloadBuf), hdr.payloadLen);
            addLog(msg);
            handlePacket(senderId, hdr, payloadBuf);
        }

        // 🕒 Проверка потери пинга
        if (millis() - lastPingTime > checkInterval && currentProfileIndex > 0)
        {
            currentProfileIndex--;
            const auto &profile = loraProfiles[currentProfileIndex];

            addLog("⚠️ No ping in 1 minute. Degrading LoRa profile to index " + String(currentProfileIndex) +
                   " (SF=" + profile.spreadingFactor + ", BW=" + profile.bandwidth + ", CR=" + profile.codingRate + ")");

            // Формируем ASA-ответ без запроса
            PacketAsaApprove resp;
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);

            resp.profileIndex = currentProfileIndex;
            loraComm->sendPacket(resp, BOAT_DEVICE_ID);
            delay(50); // Ждем, чтобы пакет ушел
            applyProfile(currentProfileIndex);

            lastPingTime = millis(); // Сбросим таймер
        }
    }

    // Send an arbitrary command string (e.g. “M:120”)
    void sendCommandString(const String &cmdStr)
    {
        PacketCommand cmd{};
        cmd.packetType = CMD_COMMAND_STRING;
        cmd.packetId = nextPacketId++;
        cmd.payloadLen = cmdStr.length();
        memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase),
               cmdStr.c_str(),
               cmd.payloadLen);

        loraComm->sendPacket(cmd, BOAT_DEVICE_ID);
        addLog(String("[MC] Sent command string: ") + cmdStr);
    }

    // Log sink (override)
    void addLog(const String &msg) override
    {
        Serial.println(msg);
    }
    void applyProfile(uint8_t idx)
    {
        currentProfileIndex = idx;
        const auto &p = loraProfiles[idx];
        loraComm->applySettings(p.spreadingFactor, p.codingRate, p.bandwidth);
    }

private:
    LoRaComm *loraComm;
    uint8_t nextPacketId = 0;
    unsigned long lastPingTime = 0;
    int currentProfileIndex = 3;
    unsigned long checkInterval = 12 * 1000;

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
        case CMD_ACK:
        {
            // PacketAck ack;
            uint8_t ackedId = buf[0];
            // ack = *reinterpret_cast<const PacketAck *>(&hdr);
            //  uint16_t ackFor = atoi(reinterpret_cast<const char *>(buf));
            addLog(String("[MC] ACK for packet ID: ") + ackedId);
            break;
        }

        case CMD_TELEMETRY_FRAGMENT:
        {
            String fragment(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            processTelemetryFragment(fragment);
            break;
        }
        case CMD_REPOSNCE_ASA:
        {
            // 1. Проверка длины (int + int + float)
            if (hdr.payloadLen != sizeof(uint8_t))
            {
                addLog("[MC] Invalid ASA packet size " + String(hdr.payloadLen) + 
                       ", expected " + String(sizeof(uint8_t)));
                break;
            }

            if (abs(rssi / currentProfileIndex) < 5)
            {
                addLog("[MC] Received ASA response with SNR=" + String(snr, 1) +
                       ", RSSI=" + String(rssi) + "dBm, but no change in profile index.abs(rssi/currentProfileIndex)=" + String(abs(rssi / currentProfileIndex)) + "< 10");
                break;
            }

            // 2. Прочитать поля из буфера
            PacketAsaRequest req;
            req.packetType = CMD_REQUEST_ASA;
            req.packetId = hdr.packetId;
            req.payloadLen = hdr.payloadLen;

            uint8_t profileIndex = buf[0];
            const auto &profile = loraProfiles[profileIndex];

            // Формируем ответ
            PacketAsaApprove resp{};
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);
            resp.profileIndex = profileIndex;

            loraComm->sendPacket(resp, sender);
            addLog("⚙️ MC: Approving and applying LoRa profile index " + String(profileIndex));

            delay(50);
            applyProfile(profileIndex);
            break;
        }
        case CMD_PING:
        {
            PacketCommand pong;
            pong.packetType = CMD_PONG;
            pong.packetId = nextPacketId++;
            pong.payloadLen = 0;
            loraComm->sendPacket(pong, sender);
            addLog("[handlePacket] Pong sent to boat.");
            break;
        }

        case CMD_REQUEST_ASA:
        {

            // 1. Проверка длины (int + int + float)
            if (hdr.payloadLen != sizeof(uint8_t))
            {
                addLog("[MC] Invalid ASA packet size");
                break;
            }

            if (abs(rssi / currentProfileIndex) < 5)
            {
                addLog("[MC] Received ASA response with SNR=" + String(snr, 1) +
                       ", RSSI=" + String(rssi) + "dBm, but no change in profile index.abs(rssi/currentProfileIndex)=" + String(abs(rssi / currentProfileIndex)) + "< 10");
                break;
            }

            // 2. Прочитать поля из буфера
            PacketAsaRequest req;
            req.packetType = CMD_REQUEST_ASA;
            req.packetId = hdr.packetId;
            req.payloadLen = hdr.payloadLen;

            uint8_t profileIndex = buf[0];
            const auto &profile = loraProfiles[profileIndex];
            addLog("[MC] Received ASA request from sender " + String(sender) +
                   ", packetId=" + String(hdr.packetId) +
                   ", payloadLen=" + String(hdr.payloadLen) + " (profileIndex=" + String(profileIndex) + " profile: SF=" + String(profile.spreadingFactor) + ", BW=" + String(profile.bandwidth) + ", CR=" + String(profile.codingRate) + ")");
            // Формируем ответ
            PacketAsaApprove resp{};
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);
            resp.profileIndex = profileIndex;

            loraComm->sendPacket(resp, sender);
            addLog("⚙️ MC: Approving and applying LoRa profile index " + String(profileIndex));

            delay(50);
            applyProfile(profileIndex);
            break;
        }

        default:
            // Unhandled packet types …
            addLog("[MC] Unhandled packet type: " + String((char)hdr.packetType) +
                   ", sender=" + String(sender) +
                   ", payloadLen=" + String(hdr.payloadLen));
            break;
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
        memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase), &code, 1);
        loraComm->sendPacket(cmd, BOAT_DEVICE_ID);
        addLog("[MC] Sent P(request info=" + String((char)what) + ") to boat");
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
};
