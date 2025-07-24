#pragma once
// LoRaCore.hpp - Unified LoRa Communication System
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <vector>
#include <functional>
#include "settings.h"
#include "PacketClasses.hpp"
#include "LogInterface.hpp"
#include "boat_utils.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// ENUMS AND STRUCTURES
// ─────────────────────────────────────────────────────────────────────────────
enum class RadioMode : uint8_t
{
    LORA,
    FSK
};

struct LoRaProfile
{
    uint8_t sf{LORA_SF};          // 7‑12
    uint8_t cr{LORA_CODING_RATE}; // 5‑8  (CR4/5..CR4/8)
    float bw{LORA_BANDWIDTH};     // kHz (125/250/500)
};

struct FSKProfile
{
    uint32_t bitrate{38400};   // bit/s
    uint32_t deviation{25000}; // Hz
    uint32_t rxBw{50000};      // Hz (Rx filter BW)
};

struct PendingSend
{
    LoRaPacket pkt = {}; // Initialize to zero
    uint32_t timestamp;
    uint8_t retries;
};

// ─────────────────────────────────────────────────────────────────────────────
// HELPER FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────
inline void packBaseIntoLoRa(LoRaPacket &out, uint8_t senderId, uint8_t receiverId,
                             const PacketBase &base, const uint8_t *payload)
{
    memset(&out, 0, sizeof(out));
    out.setSenderId(senderId);
    out.setReceiverId(receiverId);
    out.packetType = base.packetType;
    out.packetId = base.packetId;
    
    // Debug: Log the input values to track corruption
    char debugBuf[100];

    if (base.payloadLen > MAX_LORA_PAYLOAD) { // Safety check: prevent memory corruption from invalid payload length
        out.payloadLen = 0; // Set to 0 to prevent further corruption
        snprintf(debugBuf, sizeof(debugBuf), "❌ packBase: Invalid payloadLen=%u > MAX=%u", base.payloadLen, MAX_LORA_PAYLOAD);
        Serial.println(debugBuf);
    } else {
        out.payloadLen = base.payloadLen;
        if (base.payloadLen > 0 && payload != nullptr)
        {
            memcpy(out.payload, payload, base.payloadLen);
        }
    }
}


class LoRaCore
{
private:
    uint8_t myDeviceId;
    Module *_module;
    SX1262 radio;
    LogInterface *_log;
    QueueHandle_t incomingQueue = nullptr;
    QueueHandle_t outgoingQueue = nullptr;
    SemaphoreHandle_t radioSemaphore = nullptr;
    static TaskHandle_t receiverTaskHandle;
    std::vector<PendingSend> pending;
    SemaphoreHandle_t pendingMutex = nullptr; // Мьютекс для pending vector
    std::function<void(PacketId_t, uint8_t, uint8_t)> ackCallback = nullptr; // callback(packetId, senderId, packetType)
    std::vector<String> logBuffer;
    SemaphoreHandle_t logMutex = nullptr;
    static const size_t MAX_LOG_BUFFER_SIZE = 30;
    
    // Флаг активного приема - блокирует передачу
    volatile bool receivingInProgress = false;
    uint8_t currentMaxRetries = 4;
    uint32_t currentRetryTimeoutMs = 3200;
    static const uint32_t FAST_TX_THRESHOLD_MS = 100;  // Быстрая передача < 100мс
    static const uint32_t SLOW_TX_THRESHOLD_MS = 1000; // Медленная передача > 1сек
    static const uint32_t QUEUE_FULL_RETRY_MS = 200;   // Повтор при заполненной очереди
    RadioMode _mode{RadioMode::LORA};
    bool _manual{false};
    LoRaProfile _loraLong;
    FSKProfile _fskFast;
    int currentSF{LORA_SF};
    int currentCR{LORA_CODING_RATE};
    float currentBW{LORA_BANDWIDTH};
    float currentFreq{LORA_FREQUENCY};
    int8_t currentTX{LORA_TX_POWER};
    uint32_t currentBitrate{0};
    uint32_t currentDeviation{0};
    uint8_t currentProfileIndex{0};
    static constexpr float RSSI_ENTER_FSK = -85.0f;
    static constexpr float RSSI_LEAVE_FSK = -92.0f;
    static constexpr float SNR_ENTER_FSK = 8.0f;
    PacketId_t nextPacketId{0}; // Централизованный счетчик ID пакетов

    // Система агрегированных ACK
    PacketBulkAck pendingBulkAck;
    unsigned long lastBulkAckTime = 0;
    static constexpr unsigned long BULK_ACK_INTERVAL_MS = 600;
    static constexpr unsigned long BULK_ACK_MAX_WAIT_MS = 250;

    void putToLogBuffer(const String& msg)
    {
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            logBuffer.push_back(msg);
            if (logBuffer.size() > MAX_LOG_BUFFER_SIZE)
            {
                logBuffer.erase(logBuffer.begin());
            }
            xSemaphoreGive(logMutex);
        }
    }

    void updateRetryParameters()
    {
        if (_mode == RadioMode::LORA)
        {
            // Для LoRa профилей рассчитываем время передачи на основе SF, BW, CR
            // Приблизительная формула времени передачи пакета в LoRa (в мс)
            float symbolTime = (1 << currentSF) / (currentBW * 1000); // время символа в секундах
            float preambleTime = (8 + 4.25f) * symbolTime;            // преамбула + sync

            // Примерная длина пакета 50 байт (включая заголовки)
            float payloadSymbols = 8.0f + std::max(ceil((8.0f * 50 - 4.0f * currentSF + 28 + 16) /
                                                        (4.0f * (currentSF - 2))),
                                                   0.0f) *
                                              currentCR;
            float packetTime = (preambleTime + payloadSymbols * symbolTime) * 1000; // в мс

            // Адаптивный таймаут: 3-4 времени передачи пакета + буфер
            currentRetryTimeoutMs = std::max(8500U, (uint32_t)(packetTime * 3.5f + 1000));
            if (currentSF <= 7) // Количество повторов зависит от SF (чем выше SF, тем больше повторов)
            {
                currentMaxRetries = 2; // Быстрые профили
            }
            else if (currentSF <= 9)
            {
                currentMaxRetries = 3; // Средние профили
            }
            else
            {
                currentMaxRetries = 4; // Медленные но надежные профили
            }

            char s[120];
            snprintf(s, sizeof(s), "📊 Adaptive LoRa retry: SF%d → timeout=%ums, retries=%u (pkt≈%.1fms)",
                     currentSF, currentRetryTimeoutMs, currentMaxRetries, packetTime);
            putToLogBuffer(String(s));
        }
        else if (_mode == RadioMode::FSK)
        {
            // Для FSK/GFSK рассчитываем на основе битрейта/ Время передачи 50-байтового пакета при текущем битрейте
            float packetTime = (50.0f * 8.0f * 1000.0f) / currentBitrate; // в мс
            currentRetryTimeoutMs = std::max(1500U, (uint32_t)(packetTime * 2.5f + 600)); // Адаптивный таймаут: 2-3 времени передачи + буфер
            if (currentBitrate >= 19200) // FSK быстрее, меньше повторов
            {
                currentMaxRetries = 2; // Высокоскоростные FSK
            }
            else
            {
                currentMaxRetries = 3; // Низкоскоростные FSK
            }

            char s[120];
            snprintf(s, sizeof(s), "📊 Adaptive FSK retry: %ukbps → timeout=%ums, retries=%u (pkt≈%.1fms)",
                     currentBitrate / 1000, currentRetryTimeoutMs, currentMaxRetries, packetTime);
            putToLogBuffer(String(s));
        }
    }

    void LLog(const char *s)
    {
        if (_log)
            _log->addLog(String(s));
        else
            Serial.println(s);
    }

    void LLog(const String &s)
    {
        LLog(s.c_str());
    }

    bool applyLoRa(const LoRaProfile &p)
    {
        bool success = radio.setModem(RADIOLIB_MODEM_LORA) == RADIOLIB_ERR_NONE &&
                       radio.setFrequency(currentFreq) == RADIOLIB_ERR_NONE &&
                       radio.setSpreadingFactor(p.sf) == RADIOLIB_ERR_NONE &&
                       radio.setCodingRate(p.cr) == RADIOLIB_ERR_NONE &&
                       radio.setBandwidth(p.bw) == RADIOLIB_ERR_NONE &&
                       radio.setPreambleLength(8) == RADIOLIB_ERR_NONE &&
                       radio.setCRC(true) == RADIOLIB_ERR_NONE &&
                       radio.setOutputPower(currentTX) == RADIOLIB_ERR_NONE;

        if (success)
        {
            currentSF = p.sf;
            currentCR = p.cr;
            currentBW = p.bw;
            updateRetryParameters(); // Обновляем адаптивные параметры
        }
        return success;
    }

    bool applyFSK(const FSKProfile &p)
    {
        // Для SX1262 используем beginFSK() - это GFSK модуляция / Параметры: bitrate (kbps), freqDev (kHz), rxBw (kHz), preambleLength
        int result = radio.beginFSK(p.bitrate / 1000.0f, p.deviation / 1000.0f, p.rxBw / 1000.0f, 4);
        if (result != RADIOLIB_ERR_NONE)
        {
            return false;
        }
        bool success = radio.setCRC(true) == RADIOLIB_ERR_NONE; // Включаем CRC и обновляем текущие параметры
        if (success)
        {
            currentBitrate = p.bitrate;
            currentDeviation = p.deviation;
            currentBW = p.rxBw;
            updateRetryParameters(); // Обновляем адаптивные параметры
        }
        return success;
    }

    bool switchTo(RadioMode m)
    {
        if (m == _mode)
            return true;
        bool ok = (m == RadioMode::LORA) ? applyLoRa(_loraLong) : applyFSK(_fskFast);
        if (ok)
        {
            _mode = m;
            if (_log)
                LLog(String("LoRaCore: Switched to ") + (m == RadioMode::LORA ? "LoRa" : "GFSK"));
            radio.startReceive();
        }
        return ok;
    }

    void handleAck(const LoRaPacket &pkt)
    {
        if (pkt.payloadLen != sizeof(PacketId_t))
        {
            char s[80];
            snprintf(s, sizeof(s), "❌ Invalid ACK payload len: %u (expected %u)", pkt.payloadLen, sizeof(PacketId_t));
            putToLogBuffer(String(s));
            return;
        }
        PacketId_t ackedId;
        memcpy(&ackedId, pkt.payload, sizeof(ackedId));
        char s[80];
        snprintf(s, sizeof(s), "📩 Single ACK received: id=%u from device %u", ackedId, pkt.getSenderId());
        putToLogBuffer(String(s));

        handleSingleAck(ackedId, pkt.getSenderId(), pkt.packetType);
    }

    void handleBulkAck(const LoRaPacket &pkt)
    {
        if (pkt.payloadLen < sizeof(uint8_t))
        {
            putToLogBuffer(String("❌ BULK ACK: Invalid payload length"));
            return;
        }

        uint8_t count = pkt.payload[0];

        if (count > 10 || pkt.payloadLen != sizeof(uint8_t) + (count * sizeof(PacketId_t)))
        {
            char errorLog[120];
            snprintf(errorLog, sizeof(errorLog), "❌ BULK ACK: Invalid count=%u or length mismatch", count);
            putToLogBuffer(String(errorLog));
            return;
        }

        PacketId_t ackedIds[10];
        memcpy(ackedIds, &pkt.payload[1], count * sizeof(PacketId_t));
        PacketId_t uniqueIds[10];
        uint8_t uniqueCount = 0;

        for (uint8_t i = 0; i < count; i++)
        {
            bool isDuplicate = false;
            for (uint8_t j = 0; j < uniqueCount; j++)            // Проверяем, есть ли уже такой ID в списке уникальных
            {
                if (uniqueIds[j] == ackedIds[i])
                {
                    isDuplicate = true;
                    break;
                }
            }
            if (!isDuplicate && uniqueCount < 10)// Если ID уникальный, добавляем его в список
            {
                uniqueIds[uniqueCount] = ackedIds[i];
                uniqueCount++;
            }
        }
        String idList = ""; // Логируем результат с информацией о дубликатах
        String uniqueIdList = "";
        for (uint8_t i = 0; i < count; i++)// Список всех полученных ID
        {
            if (i > 0)
                idList += ",";
            idList += String(ackedIds[i]);
        }
        for (uint8_t i = 0; i < uniqueCount; i++) // Список уникальных ID
        {
            if (i > 0)
                uniqueIdList += ",";
            uniqueIdList += String(uniqueIds[i]);
        }

        char successLog[200];
        if (uniqueCount != count)
        {
            snprintf(successLog, sizeof(successLog), "📩 BULK ACK: %u IDs [%s] → %u unique [%s] from device %u (filtered %u duplicates)", count, idList.c_str(), uniqueCount, uniqueIdList.c_str(), pkt.getSenderId(), count - uniqueCount);
        }
        else
        {
            snprintf(successLog, sizeof(successLog), "📩 BULK ACK received: %u IDs [%s] from device %u",  count, idList.c_str(), pkt.getSenderId());
        }
        putToLogBuffer(String(successLog));

        for (uint8_t i = 0; i < uniqueCount; i++) // Обрабатываем только уникальные ID
        {
            handleSingleAck(uniqueIds[i], pkt.getSenderId(), pkt.packetType);
        }
    }

    // Добавить ACK в bulk пакет
    void addToBulkAck(PacketId_t packetId, uint8_t targetDeviceId)
    {
        if (pendingBulkAck.addAck(packetId))
        {
            char s[100];
            snprintf(s, sizeof(s), "📦 Added ACK for packet %u to bulk (%u/10)", packetId, pendingBulkAck.count);
            putToLogBuffer(String(s));

            // Если пакет заполнен или прошло достаточно времени - отправляем
            if (pendingBulkAck.isFull() ||
                (millis() - lastBulkAckTime > BULK_ACK_MAX_WAIT_MS && !pendingBulkAck.isEmpty()))
            {
                sendBulkAck(targetDeviceId);
            }
        }
        else
        {
            // Пакет переполнен - отправляем текущий и начинаем новый
            sendBulkAck(targetDeviceId);
            if (pendingBulkAck.addAck(packetId)) {
                char s[100];
                snprintf(s, sizeof(s), "📦 Started new bulk ACK with packet %u", packetId);
                putToLogBuffer(String(s));
            }
        }
    }

    // Отправить накопленные ACK
    void sendBulkAck(uint8_t targetDeviceId)
    {
        if (pendingBulkAck.isEmpty())
            return;

        // Диагностика перед отправкой
        if (pendingBulkAck.hasDuplicates()) {
            putToLogBuffer(String("⚠️ WARNING: BULK ACK contains duplicates: ") + pendingBulkAck.getDebugInfo());
        }

        // Формируем payload: count + массив ID
        uint8_t payload[1 + 10 * sizeof(PacketId_t)];
        payload[0] = pendingBulkAck.count;
        memcpy(&payload[1], pendingBulkAck.ackedIds, pendingBulkAck.count * sizeof(PacketId_t));

        size_t payloadSize = sizeof(uint8_t) + (pendingBulkAck.count * sizeof(PacketId_t));

        // Отправляем bulk ACK
        sendPacketBase(targetDeviceId, pendingBulkAck, payload, false);
        
        putToLogBuffer(String("✅ Sent BULK ACK for ") + String(pendingBulkAck.count) + " packets: " + pendingBulkAck.getDebugInfo());

        pendingBulkAck.clear();
        lastBulkAckTime = millis();
    }

    // Проверить, нужно ли отправить bulk ACK по таймауту
    void checkBulkAckTimeout(uint8_t targetDeviceId)
    {
        if (!pendingBulkAck.isEmpty() &&
            (millis() - lastBulkAckTime > BULK_ACK_INTERVAL_MS))
        {
            sendBulkAck(targetDeviceId);
        }
    }

private:
    void handleSingleAck(PacketId_t ackedId, uint8_t senderId, uint8_t packetType)
    {
        if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) // Потокобезопасный доступ к pending
        {
            auto it = std::find_if(pending.begin(), pending.end(), [ackedId](const PendingSend &p)
                                   { return p.pkt.packetId == ackedId; });
            if (it != pending.end())
            {
                uint8_t originalPacketType = it->pkt.packetType;
                char s[100]; // Логируем подтверждение ACK
                snprintf(s, sizeof(s), "✅ACK confirmed: id=%u, from=%u, type=%c, origType=%c",ackedId, senderId, packetType, originalPacketType);
                putToLogBuffer(String(s));
                pending.erase(it);
                if (ackCallback) // Вызываем callback если он установлен
                {
                    char callbackLog[120];  // Логируем вызов callback для ASA отладки
                    snprintf(callbackLog, sizeof(callbackLog), "🔔 Calling ACK callback: id=%u, sender=%u, origType=%c", ackedId, senderId, originalPacketType);
                    putToLogBuffer(String(callbackLog));
                    ackCallback(ackedId, senderId, originalPacketType);
                }
            }
            else
            {
                char s[80];// Пакет с таким ID не найден в pending списке - возможно дублированный ACK
                snprintf(s, sizeof(s), "⚠️ Duplicate ACK ignored: id=%u from device %u (not in pending)", ackedId, senderId);
                putToLogBuffer(String(s));
            }
            xSemaphoreGive(pendingMutex);
        }
    }

    static void logTaskWrapper(void *param)
    {
        static_cast<LoRaCore *>(param)->logTask();
    }

    static void receiveTaskWrapper(void *param)
    {
        static_cast<LoRaCore *>(param)->receiveTask();
    }

    static void sendTaskWrapper(void *param)
    {
        static_cast<LoRaCore *>(param)->sendTask();
    }

    static void resendTaskWrapper(void *param)
    {
        static_cast<LoRaCore *>(param)->resendTask();
    }

    void logTask()
    {
        while (true)
        {
            if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(500)) == pdTRUE)
            {
                if (!logBuffer.empty())
                {
                    std::vector<String> logsToProcess = logBuffer; // Копируем все логи для обработки
                    logBuffer.clear();
                    xSemaphoreGive(logMutex);
                    for (const auto &logEntry : logsToProcess) // Обрабатываем логи вне критической секции
                    {
                        if (_log)
                        {
                            _log->addLog(logEntry);
                        }
                        else
                        {
                            Serial.println(logEntry);
                        }
                    }
                }
                else
                {
                    xSemaphoreGive(logMutex);
                }
            }
            uint32_t randomDelay = 21 + (esp_random() % 29);
            vTaskDelay(pdMS_TO_TICKS(randomDelay));
        }
    }

    void receiveTask()
    {
        static char s[200]; // Keep static buffer for performance
        static String fullLog = "";

        while (true)
        {
            LoRaPacket pkt = {}; // Create fresh packet each time - no static!
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // спим до события

            fullLog = "";
            
            // Устанавливаем флаг активного приема - это блокирует передачу
            receivingInProgress = true;
            
            // Пытаемся захватить семафор неблокирующе
            if (radioSemaphore && xSemaphoreTake(radioSemaphore, 0) == pdTRUE) {
                // Захватили семафор - можем читать
            unsigned long t0 = millis();
            int len = radio.getPacketLength();

            if (len > 0 && len <= sizeof(LoRaPacket))
            {
                if (len < offsetof(LoRaPacket, payload))
                {
                    char shortLog[80];
                    snprintf(shortLog, sizeof(shortLog), "🚫 PACKET TOO SHORT: len=%d < header_size=%d", len, (int)offsetof(LoRaPacket, payload));
                    putToLogBuffer(String(shortLog));
                    radio.startReceive();
                    if (radioSemaphore)
                        xSemaphoreGive(radioSemaphore);
                    receivingInProgress = false; // Сбрасываем флаг при ошибке
                    continue;
                }

                memset(&pkt, 0, sizeof(pkt));
                radio.readData((uint8_t *)&pkt, len);
                unsigned long t1 = millis();
                radio.startReceive();

                // Валидация целостности пакета
                if (pkt.getSenderId() == myDeviceId)
                {
                    if (radioSemaphore)
                        xSemaphoreGive(radioSemaphore);
                    receivingInProgress = false; // Сбрасываем флаг при отфильтровке
                    continue;
                }
            

                String payloadHex = "";
                // Safety check to prevent corruption from invalid payload length
                int safePayloadLen = (pkt.payloadLen > MAX_LORA_PAYLOAD) ? 0 : pkt.payloadLen;
                int maxPayloadToShow = std::min(safePayloadLen, 32);
                for (int i = 0; i < maxPayloadToShow; i++)
                {
                    char hexByte[4];
                    snprintf(hexByte, sizeof(hexByte), "%02X ", pkt.payload[i]);
                    payloadHex += hexByte;
                }
                if (pkt.payloadLen > MAX_LORA_PAYLOAD) {
                    payloadHex = "❌CORRUPTED_LEN=" + String(pkt.payloadLen);
                } else if (pkt.payloadLen > 16) {
                    payloadHex += "...";
                }

                snprintf(s, sizeof(s), "[RX:%d][L:%d]%lums→[%u->%u], T=[%c/%d], id=%u, plLen=%u", getCurrentProfileIndex(), len, t1 - t0, pkt.getSenderId(), pkt.getReceiverId(), pkt.packetType, pkt.packetType, pkt.packetId, pkt.payloadLen);
                fullLog = String(s) + ", pl=" + payloadHex;

                if (pkt.packetType == CMD_ACK)
                {
                    handleAck(pkt);
                }
                else if (pkt.packetType == CMD_BULK_ACK)
                {
                    handleBulkAck(pkt);
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

            if (radioSemaphore)
                xSemaphoreGive(radioSemaphore);
            
            // Сбрасываем флаг приема - разрешаем передачу
            receivingInProgress = false;

            if (fullLog.length() > 0)
            {
                putToLogBuffer(fullLog);
            }
            } else {
                // Не смогли захватить семафор - передача идет, подождем немного
                receivingInProgress = false;
                vTaskDelay(pdMS_TO_TICKS(1)); // Короткая пауза
            }
        }
    }

    void sendTask()
    {
        static char s[200];
        while (true)
        {
            LoRaPacket pkt = {};
            if (xQueueReceive(outgoingQueue, &pkt, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // КРИТИЧНО: Проверяем флаг активного приема - Если идет прием - НЕ ПЕРЕДАЕМ, возвращаем пакет в очередь
                if (receivingInProgress) {
                    // Возвращаем пакет в начало очереди для повторной попытки
                    xQueueSendToFront(outgoingQueue, &pkt, 0);
                    vTaskDelay(pdMS_TO_TICKS(1)); // Короткая пауза
                    continue;
                }
                
                if (radioSemaphore)
                    xSemaphoreTake(radioSemaphore, portMAX_DELAY);

                // Двойная проверка: возможно прием начался пока мы ждали семафор
                if (receivingInProgress) {
                    // Освобождаем семафор и возвращаем пакет в очередь
                    if (radioSemaphore)
                        xSemaphoreGive(radioSemaphore);
                    xQueueSendToFront(outgoingQueue, &pkt, 0);
                    vTaskDelay(pdMS_TO_TICKS(1));
                    continue;
                }

                radio.standby();
                ssize_t len = offsetof(LoRaPacket, payload) + pkt.payloadLen;
                
                
                LoRaPacket txPkt = pkt;  // Create a copy for transmission to prevent radio library from corrupting original
                
                unsigned long t0 = millis();
                int result = radio.transmit((uint8_t *)&txPkt, len); // Use copy for transmission
                radio.startReceive();
                if (radioSemaphore)
                    xSemaphoreGive(radioSemaphore);
                unsigned long txDuration = millis() - t0;

                String payloadHex = "";
                int safePayloadLen = (pkt.payloadLen > MAX_LORA_PAYLOAD) ? 0 : pkt.payloadLen;
                for (int i = 0; i < safePayloadLen; i++)
                {
                    char hexByte[4];
                    snprintf(hexByte, sizeof(hexByte), "%02X ", pkt.payload[i]);
                    payloadHex += hexByte;
                }
                if (pkt.payloadLen > MAX_LORA_PAYLOAD) {
                    payloadHex = "❌CORRUPTED_LEN=" + String(pkt.payloadLen);
                }

                
                snprintf(s, sizeof(s),"[TX:%d][L:%d]%lums→[%u->%u], T=[%c/%d], id=%u, plLen=%u ", getCurrentProfileIndex(), (int)len,txDuration, pkt.getSenderId(), pkt.getReceiverId(), pkt.packetType, pkt.packetType, pkt.packetId, pkt.payloadLen);
                String fullLog = String(s) + ", pl=" + payloadHex;
                putToLogBuffer(fullLog);

                if (result != RADIOLIB_ERR_NONE) // Логируем только критические проблемы для ускорения
                {
                    snprintf(s, sizeof(s), "❌TX Error: code=%d, id=%u, len=%d, duration=%lums", result, pkt.packetId, (int)len, txDuration);
                    putToLogBuffer(String(s));
                }
                
                // Debug: Check if original packet was corrupted by radio.transmit()
                if (pkt.payloadLen != txPkt.payloadLen) {
                    char corruptionBuf[100];
                    snprintf(corruptionBuf, sizeof(corruptionBuf), "🚨 CORRUPTION: orig_len=%u, tx_len=%u", pkt.payloadLen, txPkt.payloadLen);
                    putToLogBuffer(corruptionBuf);
                }


            }
            uint32_t randomDelay = 19 + (esp_random() % 10);
            vTaskDelay(pdMS_TO_TICKS(randomDelay));
        }
    }

    void resendTask()
    {
        static char s[120];
        while (true)
        {
            uint32_t now = millis();
            if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(1500)) == pdTRUE) // Потокобезопасный доступ к pending
            {
                for (auto it = pending.begin(); it != pending.end();)
                {
                    if (now - it->timestamp > currentRetryTimeoutMs)
                    {
                        if (it->retries < currentMaxRetries)
                        {
                            if (xQueueSendToBack(outgoingQueue, &it->pkt, pdMS_TO_TICKS(1500)) == pdTRUE)
                            {
                                it->timestamp = now;
                                it->retries++;
                                if (it->retries >= currentMaxRetries - 1)// Логируем только критические повторы
                                {
                                    snprintf(s, sizeof(s), "🔄Retry: id=%u #%u, T=%c, to=%u", it->pkt.packetId, it->retries, it->pkt.packetType, it->pkt.getReceiverId());
                                    putToLogBuffer(String(s));
                                }
                                ++it;
                            }
                            else
                            {
                                ++it;// Если очередь заполнена, попробуем позже
                            }
                        }
                        else
                        {
                            snprintf(s, sizeof(s), "❌Drop: id=%u, T=%c, to=%u (max retries)", it->pkt.packetId, it->pkt.packetType, it->pkt.getReceiverId());
                            putToLogBuffer(String(s));
                            it = pending.erase(it);
                        }
                    }
                    else
                    {
                        ++it;
                    }
                }
                xSemaphoreGive(pendingMutex);
            }
            uint32_t randomDelay = 211 + (esp_random() % 100);
            vTaskDelay(pdMS_TO_TICKS(randomDelay));
        }
    }

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // STATIC INTERRUPT HANDLER
    // ═══════════════════════════════════════════════════════════════════════════
    static void onReceive()
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(receiverTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    LoRaCore(uint8_t deviceId, LogInterface *logger = nullptr)
        : myDeviceId(deviceId),
          _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY)),
          radio(_module),
          _log(logger)
    {
    }

    ~LoRaCore()
    {
        if (incomingQueue)
        {
            vQueueDelete(incomingQueue);
        }
        if (outgoingQueue)
        {
            vQueueDelete(outgoingQueue);
        }
        if (radioSemaphore)
        {
            vSemaphoreDelete(radioSemaphore);
        }
        if (pendingMutex)
        {
            vSemaphoreDelete(pendingMutex);
        }
        if (logMutex)
        {
            vSemaphoreDelete(logMutex);
        }

        delete _module;
    }

    bool begin()
    {
        _loraLong.sf = LORA_SF;
        _loraLong.cr = LORA_CODING_RATE;
        _loraLong.bw = LORA_BANDWIDTH;

        if (_log)
        {
            LLog("LoRaCore: Инициализация SPI для LoRa...");
        }

        // Initialize SPI
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
        digitalWrite(LORA_RST, LOW);  // Software reset of LoRa module
        delay(5);
        digitalWrite(LORA_RST, HIGH);
        delay(2);

        pinMode(LORA_BUSY, INPUT);
        if (_log)
        {
            LLog("LoRaCore: BUSY state before init: " + String(digitalRead(LORA_BUSY)));
        }

        if (!applyLoRa(_loraLong))
        {
            if (_log)
                LLog("LoRaCore: Failed to apply LoRa settings");
            return false;
        }

        radio.setDio1Action(onReceive);
        radio.startReceive();

        // Initialize FreeRTOS components - оптимизируем размеры очередей
        incomingQueue = xQueueCreate(LORA_INCOMING_QUEUE_SIZE, sizeof(LoRaPacket));
        outgoingQueue = xQueueCreate(LORA_OUTGOING_QUEUE_SIZE, sizeof(LoRaPacket));
        radioSemaphore = xSemaphoreCreateBinary();
        pendingMutex = xSemaphoreCreateMutex();
        logMutex = xSemaphoreCreateMutex();

        if (!incomingQueue || !outgoingQueue || !radioSemaphore || !pendingMutex || !logMutex)
        {
            if (_log)
                LLog("LoRaCore: Failed to create FreeRTOS objects");
            return false;
        }

        xSemaphoreGive(radioSemaphore);

        // Create tasks
        BaseType_t res1 = xTaskCreatePinnedToCore(receiveTaskWrapper, "LoRaRecv", 6144, this, 3, &receiverTaskHandle, 1);
        BaseType_t res2 = xTaskCreatePinnedToCore(sendTaskWrapper, "LoRaSend", 6144, this, 2, nullptr, 1);
        BaseType_t res3 = xTaskCreatePinnedToCore(resendTaskWrapper, "LoRaRetry", 4096, this, 1, nullptr, 0);
        BaseType_t res4 = xTaskCreatePinnedToCore(logTaskWrapper, "LoRaLog", 3072, this, 1, nullptr, 0);

        if (res1 != pdPASS || res2 != pdPASS || res3 != pdPASS || res4 != pdPASS)
        {
            if (_log)
                LLog("LoRaCore: Failed to create tasks");
            return false;
        }

        if (_log)
        {
            LLog("LoRaCore: LoRaCore инициализирован успешно.");
        }
        return true;
    }

    void applySettings(int sf, int cr, float bw)
    {
        if (radioSemaphore)
            xSemaphoreTake(radioSemaphore, portMAX_DELAY);
        radio.standby();
        radio.setSpreadingFactor(sf);
        radio.setCodingRate(cr);
        radio.setBandwidth(bw);
        currentSF = sf;
        currentCR = cr;
        currentBW = bw;
        updateRetryParameters(); // Обновляем адаптивные параметры
        radio.startReceive();
        if (radioSemaphore)
            xSemaphoreGive(radioSemaphore);

        LLog("LoRaCore: Применены настройки LoRa: SF=" + String(sf) +
             ", CR=" + String(cr) +
             ", BW=" + String(bw, 1) + "kHz");
    }

    void forceMode(RadioMode m)
    {
        _manual = true;
        switchTo(m);
    }

    void clearManualMode() { _manual = false; }
    bool isManualMode() const { return _manual; }
    RadioMode mode() const { return _mode; }
    SX1262 &getRadio() { return radio; }

    // ═══════════════════════════════════════════════════════════════════════════
    // PRIORITY SENDING FOR ACK PACKETS
    // ═══════════════════════════════════════════════════════════════════════════
    bool sendHighPriority(const LoRaPacket &pkt)
    {
        if (!outgoingQueue)
            return false;
        // Отправляем ACK пакеты в начало очереди с высоким приоритетом
        return xQueueSendToFront(outgoingQueue, &pkt, pdMS_TO_TICKS(20)) == pdTRUE;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // ACK CALLBACK MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════════
    void setAckCallback(std::function<void(PacketId_t, uint8_t, uint8_t)> callback)
    {
        ackCallback = callback;
    }

    void clearAckCallback()
    {
        ackCallback = nullptr;
    }

    // Метод для принудительного удаления пакета из pending списка (например, при успешном ASA ответе)
    bool removePendingPacket(PacketId_t packetId)
    {
        if (!pendingMutex)
            return false;

        bool removed = false;
        if (xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            auto it = std::find_if(pending.begin(), pending.end(), [packetId](const PendingSend &p)
                                   { return p.pkt.packetId == packetId; });
            if (it != pending.end())
            {
                char s[100];
                snprintf(s, sizeof(s), "🗑️ Manually removed pending packet: id=%u, type=%с", packetId, it->pkt.packetType);
                putToLogBuffer(String(s));

                pending.erase(it);
                removed = true;
            }
            xSemaphoreGive(pendingMutex);
        }
        return removed;
    }

    // PACKET SENDING / RECEIVING
    bool sendPacketBase(uint8_t receiverId, const PacketBase &base, const uint8_t *payload, bool waitForAck = true)
    {
        if (!outgoingQueue)
            return false;
        
        // Создаем копию базового пакета и автоматически присваиваем новый ID
        PacketBase modifiedBase = base;
        modifiedBase.packetId = ++nextPacketId;
        
        LoRaPacket frame = {}; // Initialize to zero
        packBaseIntoLoRa(frame, myDeviceId, receiverId, modifiedBase, payload);
        bool ok = xQueueSendToBack(outgoingQueue, &frame, 0) == pdTRUE;
        if (ok && waitForAck)
        {
            // Потокобезопасное добавление в pending с проверкой дублированных ID
            if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(1100)) == pdTRUE)
            {
                // Проверяем, нет ли уже пакета с таким ID в pending списке
                auto existingIt = std::find_if(pending.begin(), pending.end(),
                                               [&frame](const PendingSend &p)
                                               { return p.pkt.packetId == frame.packetId; });

                if (existingIt != pending.end())
                {
                    char s[100];
                    snprintf(s, sizeof(s), "⚠️ Duplicate packet ID detected: id=%u, type=%с, to=%u",frame.packetId, frame.packetType, frame.getReceiverId());
                    putToLogBuffer(String(s));
                    existingIt->timestamp = millis(); // Обновляем timestamp существующего пакета вместо добавления дубликата
                    existingIt->retries = 0;
                }
                else
                {
                    PendingSend pendingItem = {};
                    pendingItem.pkt = frame;
                    pendingItem.timestamp = millis();
                    pendingItem.retries = 0;
                    pending.push_back(pendingItem);    // ID уникальный, добавляем в pending
                }
                xSemaphoreGive(pendingMutex);
            }
        }
        return ok;
    }
    // Низкоуровневая отправка без ACK и повторов (для служебных пакетов)
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

    // DIAGNOSTIC AND MAINTENANCE METHODS
    
    // Получить текущий ID (без инкремента) - только для диагностики
    PacketId_t getCurrentPacketId() const {
        return nextPacketId;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // BULK ACK MANAGEMENT
    // ═══════════════════════════════════════════════════════════════════════════
    
    // Добавить ACK в bulk пакет (публичный интерфейс)
    void addAckToBulk(PacketId_t packetId, uint8_t targetDeviceId) {
        addToBulkAck(packetId, targetDeviceId);
    }
    
    // Принудительно отправить накопленные ACK
    void flushBulkAck(uint8_t targetDeviceId) {
        sendBulkAck(targetDeviceId);
    }
    
    // Проверить таймаут bulk ACK (вызывать в главном цикле)
    void processBulkAckTimeout(uint8_t targetDeviceId) {
        checkBulkAckTimeout(targetDeviceId);
    }
    
    size_t getPendingCount() const
    {
        if (!pendingMutex)
            return 0;
        size_t count = 0;
        if (xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            count = pending.size();
            xSemaphoreGive(pendingMutex);
        }
        return count;
    }

    void clearPending()
    {
        if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(1100)) == pdTRUE)
        {
            pending.clear();
            xSemaphoreGive(pendingMutex);
        }
    }

    size_t getLogBufferSize() const
    {
        if (!logMutex)
            return 0;
        size_t count = 0;
        if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            count = logBuffer.size();
            xSemaphoreGive(logMutex);
        }
        return count;
    }

    void clearLogBuffer()
    {
        if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            logBuffer.clear();
            xSemaphoreGive(logMutex);
        }
    }

    bool isHealthy() const
    {
        return incomingQueue != nullptr && outgoingQueue != nullptr && radioSemaphore != nullptr && pendingMutex != nullptr;
    }

    // Диагностические методы для мониторинга очередей
    size_t getIncomingQueueCount() const
    {
        return incomingQueue ? uxQueueMessagesWaiting(incomingQueue) : 0;
    }

    size_t getOutgoingQueueCount() const
    {
        return outgoingQueue ? uxQueueMessagesWaiting(outgoingQueue) : 0;
    }

    size_t getIncomingQueueFree() const
    {
        return incomingQueue ? uxQueueSpacesAvailable(incomingQueue) : 0;
    }

    size_t getOutgoingQueueFree() const
    {
        return outgoingQueue ? uxQueueSpacesAvailable(outgoingQueue) : 0;
    }

    String getQueueStatus() const
    {
        return "TX:" + String(getOutgoingQueueCount()) + "/" + String(getOutgoingQueueCount() + getOutgoingQueueFree()) + ", RX:" + String(getIncomingQueueCount()) + "/" + String(getIncomingQueueCount() + getIncomingQueueFree()) + ", Pending:" + String(getPendingCount());
    }

    // ENHANCED PROFILE METHODS
    bool applyProfileFromSettings(uint8_t profileIndex);
    String getCurrentProfileInfo() const;
    uint8_t getCurrentProfileIndex() const { return currentProfileIndex; }
    // ADAPTIVE RETRY PARAMETERS GETTERS
    uint8_t getCurrentMaxRetries() const { return currentMaxRetries; }
    uint32_t getCurrentRetryTimeout() const { return currentRetryTimeoutMs; }

    String getAdaptiveRetryInfo() const
    {
        return "Retries:" + String(currentMaxRetries) + ", Timeout:" + String(currentRetryTimeoutMs) + "ms" + " (" + ((_mode == RadioMode::LORA) ? "LoRa" : "FSK") + ")";
    }

    // Диагностический метод для отображения текущих pending пакетов
    String getPendingPacketsInfo() const
    {
        if (!pendingMutex)
            return "No pending mutex";

        String result = "";
        if (xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            result = "Pending(" + String(pending.size()) + "): ";
            for (size_t i = 0; i < pending.size(); i++)
            {
                if (i > 0)
                    result += ", ";
                result += String(pending[i].pkt.packetId) + "#" + String(pending[i].retries);
            }
            if (pending.empty())
            {
                result += "empty";
            }
            xSemaphoreGive(pendingMutex);
        }
        else
        {
            result = "Pending: mutex timeout";
        }
        return result;
    }

};

// Static member declaration (без inline для C++14)
// static TaskHandle_t LoRaCore::receiverTaskHandle;

// ENHANCED PROFILE APPLICATION
inline bool LoRaCore::applyProfileFromSettings(uint8_t profileIndex)
{
    if (profileIndex >= LORA_PROFILE_COUNT)
    {
        LLog("LoRaCore: Недопустимый индекс профиля: " + String(profileIndex));
        return false;
    }

    const auto &profile = loraProfiles[profileIndex];

    if (radioSemaphore && xSemaphoreTake(radioSemaphore, pdMS_TO_TICKS(3000)) == pdTRUE)
    {
        radio.standby();

        bool stat = false;

        if (profile.mode == RadioProfileMode::LORA)
        {
            stat = 
                radio.setModem(RADIOLIB_MODEM_LORA) == RADIOLIB_ERR_NONE &&
                radio.setFrequency(currentFreq) == RADIOLIB_ERR_NONE &&
                radio.setSpreadingFactor(profile.spreadingFactor) == RADIOLIB_ERR_NONE &&
                radio.setCodingRate(profile.codingRate) == RADIOLIB_ERR_NONE &&
                radio.setBandwidth(profile.bandwidth) == RADIOLIB_ERR_NONE &&
                radio.setPreambleLength(LORA_PREAMBLE_LEN) == RADIOLIB_ERR_NONE &&
                radio.setCRC(true) == RADIOLIB_ERR_NONE &&
                radio.setOutputPower(currentTX) == RADIOLIB_ERR_NONE &&
                radio.setSyncWord(LORA_SYNC_WORD) == RADIOLIB_ERR_NONE;

            if (stat)
            {
                _mode = RadioMode::LORA;
                currentSF = profile.spreadingFactor;
                currentCR = profile.codingRate;
                currentBW = profile.bandwidth;
                currentProfileIndex = profileIndex;
                updateRetryParameters(); // Обновляем адаптивные параметры
            }
        }
        else if (profile.mode == RadioProfileMode::FSK)
        {
            // Применяем GFSK профиль (SX1262 поддерживает GFSK, не классический FSK)
            int result;
            radio.standby(); // Убеждаемся что модуль в standby
            delay(5);
            // Для SX1262 попробуем альтернативный подход - сначала настроим модем FSK
            result = radio.setModem(RADIOLIB_MODEM_FSK);
            if (result != RADIOLIB_ERR_NONE)
            {
                LLog("LoRaCore: GFSK setModem error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            }
            result = radio.setFrequency(currentFreq);
            if (result != RADIOLIB_ERR_NONE)
            {
                LLog("LoRaCore: GFSK setFrequency error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            }
            // Попробуем использовать стандартные методы вместо beginFSK
            result = radio.setBitRate(profile.bitrate / 1000.0f);
            if (result == RADIOLIB_ERR_NONE)
            {
                // Если setBitRate работает, используем классический подход
                LLog("LoRaCore: GFSK setBitRate успешно, используем классический подход");

                result = radio.setFrequencyDeviation(profile.deviation / 1000.0f);
                if (result != RADIOLIB_ERR_NONE)
                {
                    LLog("LoRaCore: GFSK setFrequencyDeviation error: " + String(result));
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }

                result = radio.setRxBandwidth(profile.bandwidth);
                if (result != RADIOLIB_ERR_NONE)
                {
                    LLog("LoRaCore: GFSK setRxBandwidth error: " + String(result) +
                         " (trying to set " + String(profile.bandwidth, 1) + "kHz)");
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
            }
            else
            {
                // Если setBitRate не работает, попробуем beginFSK с проверенными параметрами
                LLog("LoRaCore: setBitRate не поддерживается, пробуем beginFSK...");

                // Проверим если битрейт ниже минимума SX1262
                if (profile.bitrate < 4800)
                {
                    LLog("LoRaCore: GFSK bitrate " + String(profile.bitrate) + " below SX1262 minimum (4800)");
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
                // Попробуем с увеличенной длиной преамбулы и другими параметрами
                result = radio.beginFSK(profile.bitrate / 1000.0f, profile.deviation / 1000.0f,
                                        profile.bandwidth, 32, 10.0f, false);
                if (result != RADIOLIB_ERR_NONE)
                {
                    LLog("LoRaCore: GFSK beginFSK error: " + String(result) +
                         " (bitrate=" + String(profile.bitrate / 1000.0f, 1) + "kbps" +
                         ", dev=" + String(profile.deviation / 1000.0f, 1) + "kHz" +
                         ", rxBw=" + String(profile.bandwidth, 1) + "kHz)");
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
            }
            // Включаем CRC
            result = radio.setCRC(true);
            if (result != RADIOLIB_ERR_NONE)
            {
                LLog("LoRaCore: GFSK setCRC error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            } else {
                LLog("LoRaCore: GFSK setCRC успешно");
            }

            // Если дошли сюда - всё успешно
            stat = true;
            _mode = RadioMode::FSK;
            currentBitrate = profile.bitrate;
            currentDeviation = profile.deviation;
            currentBW = profile.bandwidth;
            currentProfileIndex = profileIndex;
            updateRetryParameters(); // Обновляем адаптивные параметры
        }

        if (stat)
        {
            radio.startReceive();
        }
        else
        {
            LLog("LoRaCore: Ошибка применения профиля " + String(profileIndex));
        }

        xSemaphoreGive(radioSemaphore);
        return stat;
    }

    LLog("LoRaCore:failed to get semaphore for profile " + String(profileIndex));
    return false;
}

inline String LoRaCore::getCurrentProfileInfo() const
{
    if (_mode == RadioMode::LORA)
    {
        return "LoRa #" + String(currentProfileIndex) +": SF=" + String(currentSF) + ", CR=" + String(currentCR) +", BW=" + String(currentBW, 1) + "kHz";
    }
    else
    {
        return "FSK #" + String(currentProfileIndex) +": " + String(currentBitrate / 1000.0f, 1) + "kb/s" + ", dev=" + String(currentDeviation / 1000.0f, 1) + "k" + ", bw=" + String(currentBW, 1) + "k";
    }
}
