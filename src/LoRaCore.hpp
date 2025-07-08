#pragma once
// LoRaCore.hpp
#include <Arduino.h>
#include "PacketClasses.hpp"
#include "lora_comm.hpp"
#include "LogInterface.hpp" 
static LogInterface *logger = nullptr;

inline void LLog(const String &s)
{
    if (logger)
        logger->addLog(s);
    else
        Serial.println(s);
}

struct PendingSend
{
    LoRaPacket pkt;
    uint32_t timestamp;
    uint8_t retries;
};

std::vector<PendingSend> pending;
const uint8_t MAX_RETRIES = 3;
const uint32_t RETRY_TIMEOUT_MS = 4500;

namespace LoRaCore
{

    QueueHandle_t incomingQueue = nullptr;
    QueueHandle_t outgoingQueue = nullptr;
    static LoRaComm *loRa = nullptr;

    void handleAck(const LoRaPacket &pkt)
    {
        if (pkt.payloadLen != sizeof(uint16_t))
            return;
        uint16_t ackedId;
        memcpy(&ackedId, pkt.payload, sizeof(ackedId));

        auto it = std::find_if(pending.begin(), pending.end(), [ackedId](const PendingSend &p)
                               { return p.pkt.packetId == ackedId; });
        if (it != pending.end())
        {
            LLog("✅ACK:" + String(ackedId));
            pending.erase(it);
        }
    }

    // =================== TASKS =====================
    void receiveTask(void *param)
    {
        SX1262 &radio = loRa->getRadio();
        while (true)
        {
            if (LoRaComm::packetReceivedFlag)
            {
                LoRaComm::packetReceivedFlag = false;
                unsigned long t0 = millis();
                LoRaPacket pkt;
                int len = radio.getPacketLength();
                if (len > 0 && len <= sizeof(LoRaPacket))
                {
                    radio.readData((uint8_t *)&pkt, len);
                    unsigned long t1 = millis();
                    radio.startReceive();
                    if (pkt.senderId == MY_DEVICE_ID)
                    {
                        continue;
                    }
                    LLog("[LEN "+String(len)+"]RX " + String(t1 - t0) + "ms→Packet:" + LoRaPacketToStr(pkt));
                    // Сначала проверяем, ACK ли это
                    if (pkt.packetType == CMD_ACK)
                    {
                        handleAck(pkt);
                    }
                    else
                    {
                        xQueueSendToBack(incomingQueue, &pkt, 0);
                    }
                }
                else
                {
                    radio.startReceive();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    void sendTask(void *param)
    {
        SX1262 &radio = loRa->getRadio();
        LoRaPacket pkt;

        while (true)
        {
            if (xQueueReceive(outgoingQueue, &pkt, portMAX_DELAY) == pdTRUE)
            {
                radio.standby();
                ssize_t len = offsetof(LoRaPacket, payload) + pkt.payloadLen;
                //len += sizeof(uint16_t); // CRC16 size
                unsigned long t0 = millis();
                radio.transmit((uint8_t *)&pkt, len);
                unsigned long txDuration = millis() - t0;
                radio.startReceive();
                String msg = "FREQ/BW: " + String(loRa->currentFreq) + "/" +
                             String(loRa->currentBW) + "/SF" + String(loRa->currentSF) + "/CR" +
                             String(loRa->currentCR) + "→TX=" + String(txDuration) + " ms " +
                             LoRaPacketToStr(pkt);
                LLog(msg);
            }
        }
    }

    void resendTask(void *param)
    {
        while (true)
        {
            uint32_t now = millis();
            for (auto it = pending.begin(); it != pending.end();)
            {
                if (now - it->timestamp > RETRY_TIMEOUT_MS)
                {
                    if (it->retries < MAX_RETRIES)
                    {
                        xQueueSendToBack(outgoingQueue, &it->pkt, 0);
                        it->timestamp = now;
                        it->retries++;
                        LLog("RetryPID " + String(it->pkt.packetId));
                        ++it;
                    }
                    else
                    {
                        LLog("❌ Drop packet id " + String(it->pkt.packetId));
                        it = pending.erase(it);
                    }
                }
                else
                {
                    ++it;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // =================== INIT =====================
    void init(LoRaComm *comm, LogInterface *logIf = nullptr)
    {
        loRa = comm;
        logger = logIf;

        incomingQueue = xQueueCreate(20, sizeof(LoRaPacket));
        outgoingQueue = xQueueCreate(30, sizeof(LoRaPacket));

        assert(incomingQueue != nullptr);
        assert(outgoingQueue != nullptr);

        BaseType_t res1 = xTaskCreatePinnedToCore(receiveTask, "LoRaRecv", 4096, nullptr, 3, nullptr, 1);
        BaseType_t res2 = xTaskCreatePinnedToCore(sendTask, "LoRaSend", 4096, nullptr, 2, nullptr, 1);
        BaseType_t res3 = xTaskCreatePinnedToCore(resendTask, "LoRaRetry", 4096, nullptr, 1, nullptr, 1);

        assert(res1 == pdPASS);
        assert(res2 == pdPASS);
        assert(res3 == pdPASS);
    }

    bool sendPacketBase(uint8_t receiverId, const PacketBase &base, const uint8_t *payload, bool waitForAck = true)
    {
        if (!outgoingQueue)
            return false;
        LoRaPacket frame;
        packBaseIntoLoRa(frame, MY_DEVICE_ID, receiverId, base, payload);
        bool ok = xQueueSendToBack(outgoingQueue, &frame, 0) == pdTRUE;
        if (ok && waitForAck)
        {
            pending.push_back({frame, millis(), 0});
        }
        return ok;
    }

    bool send(const LoRaPacket &pkt)
    {
        if (!outgoingQueue)
            return false;
        return xQueueSendToBack(outgoingQueue, &pkt, 0) == pdTRUE;
    }

    bool receive(LoRaPacket &pkt)
    {
        if (!incomingQueue)
            return false;
        return xQueueReceive(incomingQueue, &pkt, 0) == pdTRUE;
    }
} // namespace LoRaCore
