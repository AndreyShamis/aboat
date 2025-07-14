#pragma once
// LoRaCore.hpp - Unified LoRa Communication System
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <vector>
#include "settings.h"
#include "PacketClasses.hpp"
#include "LogInterface.hpp"

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
    LoRaPacket pkt;
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
    out.senderId = senderId;
    out.receiverId = receiverId;
    out.packetType = base.packetType;
    out.packetId = base.packetId;
    out.payloadLen = base.payloadLen;
    if (base.payloadLen > 0 && payload != nullptr)
    {
        memcpy(out.payload, payload, base.payloadLen);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// UNIFIED LORA COMMUNICATION CLASS
// ─────────────────────────────────────────────────────────────────────────────
class LoRaCore
{
private:
    // ═══════════════════════════════════════════════════════════════════════════
    // PRIVATE MEMBERS
    // ═══════════════════════════════════════════════════════════════════════════
    uint8_t myDeviceId;
    Module *_module;
    SX1262 radio;
    LogInterface *_log;
    
    // FreeRTOS components
    QueueHandle_t incomingQueue = nullptr;
    QueueHandle_t outgoingQueue = nullptr;
    SemaphoreHandle_t radioSemaphore = nullptr;
    
    // Retry mechanism
    std::vector<PendingSend> pending;
    SemaphoreHandle_t pendingMutex = nullptr;  // Мьютекс для pending vector
    static const uint8_t MAX_RETRIES = 3;
    static const uint32_t RETRY_TIMEOUT_MS = 8500;
    
    // Mode & profiles
    RadioMode _mode{RadioMode::LORA};
    bool _manual{false};
    LoRaProfile _loraLong;
    FSKProfile _fskFast;
    
    // Runtime tracking
    int currentSF{LORA_SF};
    int currentCR{LORA_CODING_RATE};
    float currentBW{LORA_BANDWIDTH};
    float currentFreq{LORA_FREQUENCY};
    int8_t currentTX{LORA_TX_POWER};
    
    // FSK runtime tracking
    uint32_t currentBitrate{0};
    uint32_t currentDeviation{0};
    uint8_t currentProfileIndex{0};
    
    // Thresholds
    static constexpr float RSSI_ENTER_FSK = -85.0f;
    static constexpr float RSSI_LEAVE_FSK = -92.0f;
    static constexpr float SNR_ENTER_FSK = 8.0f;
    
    // Static interrupt flag
    static volatile bool packetReceivedFlag;
    
    // ═══════════════════════════════════════════════════════════════════════════
    // PRIVATE METHODS
    // ═══════════════════════════════════════════════════════════════════════════
    void LLog(const String &s)
    {
        String safeCopy = s;
        if (_log)
            _log->addLog(safeCopy);
        else
            Serial.println(safeCopy);
    }
    
    bool applyLoRa(const LoRaProfile &p)
    {
        return radio.setModem(RADIOLIB_MODEM_LORA) == RADIOLIB_ERR_NONE &&
               radio.setFrequency(currentFreq) == RADIOLIB_ERR_NONE &&
               radio.setSpreadingFactor(p.sf) == RADIOLIB_ERR_NONE &&
               radio.setCodingRate(p.cr) == RADIOLIB_ERR_NONE &&
               radio.setBandwidth(p.bw) == RADIOLIB_ERR_NONE &&
               radio.setPreambleLength(8) == RADIOLIB_ERR_NONE &&
               radio.setCRC(true) == RADIOLIB_ERR_NONE &&
               radio.setOutputPower(currentTX) == RADIOLIB_ERR_NONE;
    }
    
    bool applyFSK(const FSKProfile &p)
    {
        // Для SX1262 используем beginFSK() - это GFSK модуляция
        // Параметры: bitrate (kbps), freqDev (kHz), rxBw (kHz), preambleLength
        int result = radio.beginFSK(p.bitrate / 1000.0f, p.deviation / 1000.0f, 
                                    p.rxBw / 1000.0f, 4);
        if (result != RADIOLIB_ERR_NONE) {
            return false;
        }
        
        // Включаем CRC
        return radio.setCRC(true) == RADIOLIB_ERR_NONE;
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
        if (pkt.payloadLen != sizeof(uint16_t))
            return;
        uint16_t ackedId;
        memcpy(&ackedId, pkt.payload, sizeof(ackedId));

        // Потокобезопасный доступ к pending
        if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            auto it = std::find_if(pending.begin(), pending.end(), [ackedId](const PendingSend &p)
                                   { return p.pkt.packetId == ackedId; });
            if (it != pending.end())
            {
                char logBuffer[100];
                snprintf(logBuffer, sizeof(logBuffer), "✅ACK received: id=%u, from=%u, type=%02X", 
                    ackedId, pkt.senderId, pkt.packetType);
                LLog(String(logBuffer));
                pending.erase(it);
            }
            xSemaphoreGive(pendingMutex);
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // FREERTOS TASKS
    // ═══════════════════════════════════════════════════════════════════════════
    static void receiveTaskWrapper(void *param)
    {
        static_cast<LoRaCore*>(param)->receiveTask();
    }
    
    static void sendTaskWrapper(void *param)
    {
        static_cast<LoRaCore*>(param)->sendTask();
    }
    
    static void resendTaskWrapper(void *param)
    {
        static_cast<LoRaCore*>(param)->resendTask();
    }
    
    void receiveTask()
    {
        static LoRaPacket pkt;
        static char logBuffer[200];
        static String fullLog = "";
        
        while (true)
        {
            if (packetReceivedFlag)
            {
                fullLog = "";
                if (radioSemaphore) xSemaphoreTake(radioSemaphore, portMAX_DELAY);
                packetReceivedFlag = false;
                unsigned long t0 = millis();
                int len = radio.getPacketLength();
                
                if (len > 0 && len <= sizeof(LoRaPacket))
                {
                    memset(&pkt, 0, sizeof(pkt));
                    radio.readData((uint8_t *)&pkt, len);
                    unsigned long t1 = millis();
                    radio.startReceive();
                    
                    if (pkt.senderId == myDeviceId)
                    {
                        if (radioSemaphore) xSemaphoreGive(radioSemaphore);
                        continue;
                    }
                    
                    String payloadHex = "";
                    for (int i = 0; i < pkt.payloadLen && i < 16; i++) {
                        char hexByte[4];
                        snprintf(hexByte, sizeof(hexByte), "%02X ", pkt.payload[i]);
                        payloadHex += hexByte;
                    }
                    if (pkt.payloadLen > 16) payloadHex += "...";
                    
                    snprintf(logBuffer, sizeof(logBuffer), 
                        "[LEN %d]RX %lums→[%u->%u], T=[%02X/%u], id=%u, plLen=%u",
                        len, t1 - t0, pkt.senderId, pkt.receiverId, 
                        pkt.packetType, pkt.packetType, pkt.packetId, pkt.payloadLen);
                    fullLog = String(logBuffer) + ", pl=" + payloadHex;
                    
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
                
                if (radioSemaphore) xSemaphoreGive(radioSemaphore);
                if (fullLog.length() > 0) {
                    LLog(fullLog);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    
    void sendTask()
    {
        static LoRaPacket pkt;
        static char logBuffer[200];
        
        while (true)
        {
            if (xQueueReceive(outgoingQueue, &pkt, portMAX_DELAY) == pdTRUE)
            {
                if (radioSemaphore) xSemaphoreTake(radioSemaphore, portMAX_DELAY);
                radio.standby();
                ssize_t len = offsetof(LoRaPacket, payload) + pkt.payloadLen;
                unsigned long t0 = millis();
                radio.transmit((uint8_t *)&pkt, len);
                unsigned long txDuration = millis() - t0;
                radio.startReceive();
                if (radioSemaphore) xSemaphoreGive(radioSemaphore);
                
                String payloadHex = "";
                for (int i = 0; i < pkt.payloadLen && i < 16; i++) {
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
            vTaskDelay(pdMS_TO_TICKS(20)); 
        }
    }
    
    void resendTask()
    {
        static char logBuffer[120];
        
        while (true)
        {
            uint32_t now = millis();
            
            // Потокобезопасный доступ к pending
            if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (auto it = pending.begin(); it != pending.end();)
                {
                    if (now - it->timestamp > RETRY_TIMEOUT_MS)
                    {
                        if (it->retries < MAX_RETRIES)
                        {
                            if (xQueueSendToBack(outgoingQueue, &it->pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
                                it->timestamp = now;
                                it->retries++;
                                
                                snprintf(logBuffer, sizeof(logBuffer), "🔄Retry: id=%u #%u, T=%02X, to=%u", 
                                    it->pkt.packetId, it->retries, it->pkt.packetType, it->pkt.receiverId);
                                LLog(String(logBuffer));
                            }
                            ++it;
                        }
                        else
                        {
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
                xSemaphoreGive(pendingMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

public:
    // ═══════════════════════════════════════════════════════════════════════════
    // STATIC INTERRUPT HANDLER
    // ═══════════════════════════════════════════════════════════════════════════
    static void onReceive()
    {
        packetReceivedFlag = true;
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR / DESTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════════
    LoRaCore(uint8_t deviceId, LogInterface *logger = nullptr) 
        : myDeviceId(deviceId),
          _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY)),
          radio(_module),
          _log(logger)
    {
    }
    
    ~LoRaCore()
    {
        // Очистка FreeRTOS ресурсов
        if (incomingQueue) {
            vQueueDelete(incomingQueue);
        }
        if (outgoingQueue) {
            vQueueDelete(outgoingQueue);
        }
        if (radioSemaphore) {
            vSemaphoreDelete(radioSemaphore);
        }
        if (pendingMutex) {
            vSemaphoreDelete(pendingMutex);
        }
        
        delete _module;
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // PUBLIC INTERFACE
    // ═══════════════════════════════════════════════════════════════════════════
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

        // Software reset of LoRa module
        digitalWrite(LORA_RST, LOW);
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
            if (_log) LLog("LoRaCore: Failed to apply LoRa settings");
            return false;
        }
        
        radio.setDio1Action(onReceive);
        radio.startReceive();
        
        // Initialize FreeRTOS components
        incomingQueue = xQueueCreate(20, sizeof(LoRaPacket));
        outgoingQueue = xQueueCreate(30, sizeof(LoRaPacket));
        radioSemaphore = xSemaphoreCreateBinary();
        pendingMutex = xSemaphoreCreateMutex();
        
        if (!incomingQueue || !outgoingQueue || !radioSemaphore || !pendingMutex) {
            if (_log) LLog("LoRaCore: Failed to create FreeRTOS objects");
            return false;
        }
        
        xSemaphoreGive(radioSemaphore);
        
        // Create tasks
        BaseType_t res1 = xTaskCreatePinnedToCore(receiveTaskWrapper, "LoRaRecv", 6144, this, 3, nullptr, 1);
        BaseType_t res2 = xTaskCreatePinnedToCore(sendTaskWrapper, "LoRaSend", 6144, this, 2, nullptr, 1);
        BaseType_t res3 = xTaskCreatePinnedToCore(resendTaskWrapper, "LoRaRetry", 4096, this, 1, nullptr, 1);
        
        if (res1 != pdPASS || res2 != pdPASS || res3 != pdPASS) {
            if (_log) LLog("LoRaCore: Failed to create tasks");
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
        if (radioSemaphore) xSemaphoreTake(radioSemaphore, portMAX_DELAY);
        radio.standby();
        radio.setSpreadingFactor(sf);
        radio.setCodingRate(cr);
        radio.setBandwidth(bw);
        currentSF = sf;
        currentCR = cr;
        currentBW = bw;
        radio.startReceive();
        if (radioSemaphore) xSemaphoreGive(radioSemaphore);
        
        LLog("LoRaCore: Применены настройки LoRa: SF=" + String(sf) +
             ", CR=" + String(cr) +
             ", BW=" + String(bw, 1) + "kHz");
    }
    
    void autoSwitch(float rssi, float snr)
    {
        if (_manual)
            return;
        if (_mode == RadioMode::LORA && rssi > RSSI_ENTER_FSK && snr > SNR_ENTER_FSK)
        {
            switchTo(RadioMode::FSK);
        }
        else if (_mode == RadioMode::FSK && rssi < RSSI_LEAVE_FSK)
        {
            switchTo(RadioMode::LORA);
        }
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
    // PACKET SENDING / RECEIVING
    // ═══════════════════════════════════════════════════════════════════════════
    bool sendPacketBase(uint8_t receiverId, const PacketBase &base, const uint8_t *payload, bool waitForAck = true)
    {
        if (!outgoingQueue)
            return false;
        LoRaPacket frame;
        packBaseIntoLoRa(frame, myDeviceId, receiverId, base, payload);
        bool ok = xQueueSendToBack(outgoingQueue, &frame, 0) == pdTRUE;
        if (ok && waitForAck)
        {
            // Потокобезопасное добавление в pending
            if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                pending.push_back({frame, millis(), 0});
                xSemaphoreGive(pendingMutex);
            }
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
    
    // ═══════════════════════════════════════════════════════════════════════════
    // DIAGNOSTIC AND MAINTENANCE METHODS
    // ═══════════════════════════════════════════════════════════════════════════
    size_t getPendingCount() const 
    {
        if (!pendingMutex) return 0;
        size_t count = 0;
        if (xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            count = pending.size();
            xSemaphoreGive(pendingMutex);
        }
        return count;
    }
    
    void clearPending() 
    {
        if (pendingMutex && xSemaphoreTake(pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            pending.clear();
            xSemaphoreGive(pendingMutex);
        }
    }
    
    bool isHealthy() const 
    {
        return incomingQueue != nullptr && 
               outgoingQueue != nullptr && 
               radioSemaphore != nullptr && 
               pendingMutex != nullptr;
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // ENHANCED PROFILE METHODS
    // ═══════════════════════════════════════════════════════════════════════════
    bool applyProfileFromSettings(uint8_t profileIndex);
    String getCurrentProfileInfo() const;
    uint8_t getCurrentProfileIndex() const { return currentProfileIndex; }
};

// ═══════════════════════════════════════════════════════════════════════════
// STATIC MEMBER DEFINITION
// ═══════════════════════════════════════════════════════════════════════════
volatile bool LoRaCore::packetReceivedFlag = false;

// ═══════════════════════════════════════════════════════════════════════════
// ENHANCED PROFILE APPLICATION
// ═══════════════════════════════════════════════════════════════════════════
bool LoRaCore::applyProfileFromSettings(uint8_t profileIndex)
{
    if (profileIndex >= LORA_PROFILE_COUNT) {
        LLog("LoRaCore: Недопустимый индекс профиля: " + String(profileIndex));
        return false;
    }
    
    const auto& profile = loraProfiles[profileIndex];
    
    if (radioSemaphore && xSemaphoreTake(radioSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
        radio.standby();
        
        bool success = false;
        
        if (profile.mode == RadioProfileMode::LORA) {
            // Применяем LoRa профиль
            success = radio.setModem(RADIOLIB_MODEM_LORA) == RADIOLIB_ERR_NONE &&
                     radio.setFrequency(currentFreq) == RADIOLIB_ERR_NONE &&
                     radio.setSpreadingFactor(profile.spreadingFactor) == RADIOLIB_ERR_NONE &&
                     radio.setCodingRate(profile.codingRate) == RADIOLIB_ERR_NONE &&
                     radio.setBandwidth(profile.bandwidth) == RADIOLIB_ERR_NONE &&
                     radio.setPreambleLength(8) == RADIOLIB_ERR_NONE &&
                     radio.setCRC(true) == RADIOLIB_ERR_NONE &&
                     radio.setOutputPower(currentTX) == RADIOLIB_ERR_NONE;
            
            if (success) {
                _mode = RadioMode::LORA;
                currentSF = profile.spreadingFactor;
                currentCR = profile.codingRate;
                currentBW = profile.bandwidth;
                currentProfileIndex = profileIndex;
                LLog("LoRaCore: Применён LoRa профиль " + String(profileIndex) + 
                     " (SF=" + String(profile.spreadingFactor) + 
                     ", CR=" + String(profile.codingRate) + 
                     ", BW=" + String(profile.bandwidth, 1) + "kHz)");
            }
        } 
        else if (profile.mode == RadioProfileMode::FSK) {
            // Применяем GFSK профиль (SX1262 поддерживает GFSK, не классический FSK)
            int result;
            
            // Убеждаемся что модуль в standby
            radio.standby();
            delay(5);
            
            // Для SX1262 попробуем альтернативный подход - сначала настроим модем FSK
            result = radio.setModem(RADIOLIB_MODEM_FSK);
            if (result != RADIOLIB_ERR_NONE) {
                LLog("LoRaCore: GFSK setModem error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            }
            
            // Устанавливаем частоту
            result = radio.setFrequency(currentFreq);
            if (result != RADIOLIB_ERR_NONE) {
                LLog("LoRaCore: GFSK setFrequency error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            }
            
            // Попробуем использовать стандартные методы вместо beginFSK
            result = radio.setBitRate(profile.bitrate / 1000.0f);
            if (result == RADIOLIB_ERR_NONE) {
                // Если setBitRate работает, используем классический подход
                LLog("LoRaCore: GFSK setBitRate успешно, используем классический подход");
                
                result = radio.setFrequencyDeviation(profile.deviation / 1000.0f);
                if (result != RADIOLIB_ERR_NONE) {
                    LLog("LoRaCore: GFSK setFrequencyDeviation error: " + String(result));
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
                
                result = radio.setRxBandwidth(profile.bandwidth);
                if (result != RADIOLIB_ERR_NONE) {
                    LLog("LoRaCore: GFSK setRxBandwidth error: " + String(result) + 
                         " (trying to set " + String(profile.bandwidth, 1) + "kHz)");
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
                
                LLog("LoRaCore: GFSK классический подход успешно настроен");
            } else {
                // Если setBitRate не работает, попробуем beginFSK с проверенными параметрами
                LLog("LoRaCore: setBitRate не поддерживается, пробуем beginFSK...");
                
                // Проверим если битрейт ниже минимума SX1262
                if (profile.bitrate < 4800) {
                    LLog("LoRaCore: GFSK bitrate " + String(profile.bitrate) + " below SX1262 minimum (4800)");
                    xSemaphoreGive(radioSemaphore);
                    return false;
                }
                
                // Попробуем с увеличенной длиной преамбулы и другими параметрами
                result = radio.beginFSK(profile.bitrate / 1000.0f, profile.deviation / 1000.0f, 
                                        profile.bandwidth, 32, 10.0f, false);
                if (result != RADIOLIB_ERR_NONE) {
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
            if (result != RADIOLIB_ERR_NONE) {
                LLog("LoRaCore: GFSK setCRC error: " + String(result));
                xSemaphoreGive(radioSemaphore);
                return false;
            }
            
            // Если дошли сюда - всё успешно
            success = true;
            _mode = RadioMode::FSK;
            currentBitrate = profile.bitrate;
            currentDeviation = profile.deviation;
            currentBW = profile.bandwidth;
            currentProfileIndex = profileIndex;
            LLog("LoRaCore: Применён GFSK профиль " + String(profileIndex) + 
                 " (Bitrate=" + String(profile.bitrate) + 
                 ", Dev=" + String(profile.deviation) + 
                 ", RxBW=" + String(profile.bandwidth, 1) + "kHz)");
        }
        
        if (success) {
            radio.startReceive();
        } else {
            LLog("LoRaCore: Ошибка применения профиля " + String(profileIndex));
        }
        
        xSemaphoreGive(radioSemaphore);
        return success;
    }
    
    LLog("LoRaCore: Не удалось захватить радио семафор для профиля " + String(profileIndex));
    return false;
}

// Получить информацию о текущем профиле
String LoRaCore::getCurrentProfileInfo() const
{
    if (_mode == RadioMode::LORA) {
        return "LoRa #" + String(currentProfileIndex) + 
               ": SF=" + String(currentSF) + 
               ", CR=" + String(currentCR) + 
               ", BW=" + String(currentBW, 1) + "kHz";
    } else {
        return "FSK #" + String(currentProfileIndex) + 
               ": " + String(currentBitrate/1000.0f, 1) + "kb/s" +
               ", dev=" + String(currentDeviation/1000.0f, 1) + "k" +
               ", bw=" + String(currentBW, 1) + "k";
    }
}
