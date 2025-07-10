#pragma once
// lora_comm.hpp
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "settings.h"       // Конфигурация LoRa модуля
#include "LogInterface.hpp" // Интерфейс для логирования
#include "PacketClasses.hpp"

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

    // out.crc = calcCRC16((uint8_t *)&out, offsetof(LoRaPacket, crc));
    //  /return;// offsetof(LoRaPacket, crc) + sizeof(out.crc);
}

class LoRaComm
{
private:
    uint8_t myDeviceId;
    Module *_module;
    SX1262 radio;
    LogInterface *_log; // Указатель на интерфейс логирования

public:
    // Mode & profiles -----------------------------------------------------------
    RadioMode _mode{RadioMode::LORA};
    bool _manual{false};
    LoRaProfile _loraLong; // текущий медленный профиль
    FSKProfile _fskFast;   // быстрый профиль

    // Runtime tracking ----------------------------------------------------------
    int currentSF{LORA_SF};
    int currentCR{LORA_CODING_RATE};
    float currentBW{LORA_BANDWIDTH};
    float currentFreq{LORA_FREQUENCY};
    int8_t currentTX{LORA_TX_POWER};

    // Thresholds (tweak if нужно) ----------------------------------------------
    static constexpr float RSSI_ENTER_FSK = -85.0f; // лучше → switch to FSK
    static constexpr float RSSI_LEAVE_FSK = -92.0f; // хуже  → fallback LoRa
    static constexpr float SNR_ENTER_FSK = 8.0f;    // dB

    // Internal helpers ----------------------------------------------------------
    bool applyLoRa(const LoRaProfile &p);
    bool applyFSK(const FSKProfile &p);
    bool switchTo(RadioMode m);
    void autoSwitch(float, float);
    void forceMode(RadioMode m)
    {
        _manual = true;
        switchTo(m);
    }
    void clearManualMode() { _manual = false; } //
    bool isManualMode() const { return _manual; }

    RadioMode mode() const { return _mode; } // Конструктор: принимает ID устройства и указатель на объект логирования

    LoRaComm(uint8_t deviceId, LogInterface *logger) : myDeviceId(deviceId),
                                                       _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY)), // Создаем модуль без указания SPI здесь
                                                       radio(_module),
                                                       _log(logger)
    {
    }
    SX1262 &getRadio() { return radio; }

    ~LoRaComm()
    {
        delete _module;
    }

    // Инициализация LoRa модуля
    bool begin()
    {
        _loraLong.sf = LORA_SF;
        _loraLong.cr = LORA_CODING_RATE;
        _loraLong.bw = LORA_BANDWIDTH;

        if (_log)
        {
            _log->addLog("LoRaComm: Инициализация SPI для LoRa...");
        }

        // Инициализация SPI, как в вашем рабочем коде из Boat
        // Это делается здесь, чтобы LoRaComm полностью отвечал за свои зависимости SPI
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

        // Программный сброс LoRa модуля
        digitalWrite(LORA_RST, LOW);
        delay(5);
        digitalWrite(LORA_RST, HIGH);
        delay(2);

        pinMode(LORA_BUSY, INPUT); // Устанавливаем BUSY пин как вход
        if (_log)
        {
            _log->addLog("LoRaComm: BUSY state before init: " + String(digitalRead(LORA_BUSY)));
        }
        applyLoRa(_loraLong); // Применяем настройки LoRa
        // bool ok = switchTo(RadioMode::LORA);
        // if (!ok)
        //     return false;
        radio.setDio1Action(LoRaComm::onReceive);
        radio.startReceive();
        if (_log)
        {
            _log->addLog("LoRaComm: LoRaComm инициализирован успешно.");
        }
        return true;
    }

    void applySettings(int sf, int cr, float bw)
    {
        radio.standby();
        radio.setSpreadingFactor(sf);
        radio.setCodingRate(cr);
        radio.setBandwidth(bw);
        currentSF = sf;
        currentCR = cr;
        currentBW = bw;
        radio.startReceive(); // обязательно после настройки
        _log->addLog("LoRaComm: Применены настройки LoRa: SF=" + String(sf) +
                     ", CR=" + String(cr) +
                     ", BW=" + String(bw, 1) + "kHz\n\n");
    }

    // Статический флаг и статическая функция обработчика прерывания
    static volatile bool packetReceivedFlag;
    static void onReceive()
    {
        packetReceivedFlag = true;
    }
};

volatile bool LoRaComm::packetReceivedFlag = false;

inline bool LoRaComm::applyLoRa(const LoRaProfile &p)
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

inline bool LoRaComm::applyFSK(const FSKProfile &p)
{
    return radio.setModem(RADIOLIB_MODEM_FSK) == RADIOLIB_ERR_NONE &&
           radio.setFrequency(currentFreq) == RADIOLIB_ERR_NONE &&
           radio.setBitRate(p.bitrate) == RADIOLIB_ERR_NONE &&
           radio.setFrequency(p.deviation) == RADIOLIB_ERR_NONE &&
           radio.setRxBandwidth(p.rxBw) == RADIOLIB_ERR_NONE &&
           radio.setPreambleLength(4) == RADIOLIB_ERR_NONE &&
           radio.setCRC(true) == RADIOLIB_ERR_NONE &&
           radio.setOutputPower(currentTX) == RADIOLIB_ERR_NONE;
}

inline bool LoRaComm::switchTo(RadioMode m)
{
    if (m == _mode)
        return true;
    bool ok = (m == RadioMode::LORA) ? applyLoRa(_loraLong) : applyFSK(_fskFast);
    if (ok)
    {
        _mode = m;
        if (_log)
            _log->addLog(String("LoRaCommDual: Switched to ") + (m == RadioMode::LORA ? "LoRa" : "FSK"));
        radio.startReceive(); // вернуться к приёму
    }
    return ok;
}

inline void LoRaComm::autoSwitch(float rssi, float snr)
{
    if (_manual)
        return; // пользователь зафиксировал режим
    if (_mode == RadioMode::LORA && rssi > RSSI_ENTER_FSK && snr > SNR_ENTER_FSK)
    {
        switchTo(RadioMode::FSK);
    }
    else if (_mode == RadioMode::FSK && rssi < RSSI_LEAVE_FSK)
    {
        switchTo(RadioMode::LORA);
    }
}