#pragma once
// LoRaCore.hpp
#include <Arduino.h>
#include <vector>
#include "PacketClasses.hpp"
#include "lora_comm.hpp"
#include "LogInterface.hpp"

static LogInterface *logger = nullptr;

inline void LLog(const String &s)
{
    // Создаем копию строки для безопасности в многопоточной среде
    String safeCopy = s;
    if (logger)
        logger->addLog(safeCopy);
    else
        Serial.println(safeCopy);
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
            // Подробное логирование ACK
            char logBuffer[100];
            snprintf(logBuffer, sizeof(logBuffer), "✅ACK received: id=%u, from=%u, type=%02X", 
                ackedId, pkt.senderId, pkt.packetType);
            LLog(String(logBuffer));
            pending.erase(it);
        }
    }

    // =================== TASKS =====================
    void receiveTask(void *param)
    {
        SX1262 &radio = loRa->getRadio();
        
        // Stack optimization: allocate once, reuse
        static LoRaPacket pkt;
        static char logBuffer[200]; // Увеличен для детального логирования
        
        while (true)
        {
            if (LoRaComm::packetReceivedFlag)
            {
                LoRaComm::packetReceivedFlag = false;
                unsigned long t0 = millis();
                
                int len = radio.getPacketLength();
                if (len > 0 && len <= sizeof(LoRaPacket))
                {
                    // Clear packet structure
                    memset(&pkt, 0, sizeof(pkt));
                    
                    radio.readData((uint8_t *)&pkt, len);
                    unsigned long t1 = millis();
                    
                    radio.startReceive();
                    
                    if (pkt.senderId == MY_DEVICE_ID)
                    {
                        continue;
                    }
                    
                    // Подробное RX логирование с payload
                    String payloadHex = "";
                    for (int i = 0; i < pkt.payloadLen && i < 16; i++) { // Ограничиваем 16 байтами для стека
                        char hexByte[4];
                        snprintf(hexByte, sizeof(hexByte), "%02X ", pkt.payload[i]);
                        payloadHex += hexByte;
                    }
                    if (pkt.payloadLen > 16) payloadHex += "...";
                    
                    snprintf(logBuffer, sizeof(logBuffer), 
                        "[LEN %d]RX %lums→[%u->%u], T=[%02X/%u], id=%u, plLen=%u",
                        len, t1 - t0, pkt.senderId, pkt.receiverId, 
                        pkt.packetType, pkt.packetType, pkt.packetId, pkt.payloadLen);
                    
                    String fullLog = String(logBuffer) + ", pl=" + payloadHex;
                    LLog(fullLog);
                    
                    // Сначала проверяем, ACK ли это
                    if (pkt.packetType == CMD_ACK)
                    {
                        handleAck(pkt);
                    }
                    else
                    {
                        // Don't block if queue is full - drop packet to prevent stack overflow
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
        
        // Stack optimization: allocate once, reuse
        static LoRaPacket pkt;
        static char logBuffer[200]; // Увеличен для детального логирования
        
        while (true)
        {
            if (xQueueReceive(outgoingQueue, &pkt, portMAX_DELAY) == pdTRUE)
            {
                radio.standby();
                ssize_t len = offsetof(LoRaPacket, payload) + pkt.payloadLen;
                
                unsigned long t0 = millis();
                radio.transmit((uint8_t *)&pkt, len);
                unsigned long txDuration = millis() - t0;
                radio.startReceive();
                
                // Подробное TX логирование с payload
                String payloadHex = "";
                for (int i = 0; i < pkt.payloadLen && i < 16; i++) { // Ограничиваем 16 байтами для стека
                    char hexByte[4];
                    snprintf(hexByte, sizeof(hexByte), "%02X ", pkt.payload[i]);
                    payloadHex += hexByte;
                }
                if (pkt.payloadLen > 16) payloadHex += "...";
                
                snprintf(logBuffer, sizeof(logBuffer), 
                    "[LEN %d]TX %lums→[%u->%u], T=[%02X/%u], id=%u, plLen=%u",
                    (int)len, txDuration, pkt.senderId, pkt.receiverId,
                    pkt.packetType, pkt.packetType, pkt.packetId, pkt.payloadLen);
                
                String fullLog = String(logBuffer) + ", pl=" + payloadHex;
                LLog(fullLog);
            }
        }
    }

    void resendTask(void *param)
    {
        // Stack optimization: use minimal local variables
        static char logBuffer[120]; // Увеличен для детального логирования
        
        while (true)
        {
            uint32_t now = millis();
            for (auto it = pending.begin(); it != pending.end();)
            {
                if (now - it->timestamp > RETRY_TIMEOUT_MS)
                {
                    if (it->retries < MAX_RETRIES)
                    {
                        // Don't block if queue is full to prevent deadlock
                        if (xQueueSendToBack(outgoingQueue, &it->pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
                            it->timestamp = now;
                            it->retries++;
                            
                            // Подробное логирование повтора
                            snprintf(logBuffer, sizeof(logBuffer), "🔄Retry: id=%u #%u, T=%02X, to=%u", 
                                it->pkt.packetId, it->retries, it->pkt.packetType, it->pkt.receiverId);
                            LLog(String(logBuffer));
                        }
                        ++it;
                    }
                    else
                    {
                        // Подробное логирование отброса пакета
                        snprintf(logBuffer, sizeof(logBuffer), "❌Drop: id=%u, T=%02X, to=%u (max retries)", 
                            it->pkt.packetId, it->pkt.packetType, it->pkt.receiverId);
                        LLog(String(logBuffer));
                        it = pending.erase(it);
                    }
                }
                else
                {
                    ++it;
                }
            }
            // Longer delay to reduce CPU usage
            vTaskDelay(pdMS_TO_TICKS(50));
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

        // Увеличиваем размер стека для предотвращения stack canary errors
        // Оригинал: 4096 → Новый: 6144 (на 50% больше)
        BaseType_t res1 = xTaskCreatePinnedToCore(receiveTask, "LoRaRecv", 6144, nullptr, 3, nullptr, 1);
        BaseType_t res2 = xTaskCreatePinnedToCore(sendTask, "LoRaSend", 6144, nullptr, 2, nullptr, 1);
        BaseType_t res3 = xTaskCreatePinnedToCore(resendTask, "LoRaRetry", 4096, nullptr, 1, nullptr, 1); // Retry task - меньше нагрузки

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
