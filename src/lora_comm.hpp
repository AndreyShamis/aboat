#pragma once
// lora_comm.hpp
#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "settings.h"       // Конфигурация LoRa модуля
#include "LogInterface.hpp" // Интерфейс для логирования
#include "PacketClasses.hpp"

class LoRaComm
{
private:
    uint8_t myDeviceId;
    Module *_module;
    SX1262 radio;
    int currentSF = LORA_SF;
    int currentCR = LORA_CODING_RATE;
    float currentBW = LORA_BANDWIDTH;
    float currentFreq = LORA_FREQUENCY;
    int8_t currentTX = LORA_TX_POWER;

    LogInterface *_log; // Указатель на интерфейс логирования

public:
    // Конструктор: принимает ID устройства и указатель на объект логирования
    LoRaComm(uint8_t deviceId, LogInterface *logger) : myDeviceId(deviceId),
                                                       _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY)), // Создаем модуль без указания SPI здесь
                                                       radio(_module),
                                                       _log(logger)
    {
    }
    SX1262 &getRadio() { return radio; }

    // Деструктор для освобождения динамически выделенной памяти
    ~LoRaComm()
    {
        delete _module;
    }

    // Инициализация LoRa модуля
    bool begin()
    {
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

        // Инициализация RadioLib SX1262 с полным набором параметров
        int state = radio.begin(
            LORA_FREQUENCY,
            LORA_BANDWIDTH,
            LORA_SF,
            LORA_CODING_RATE,
            LORA_SYNC_WORD,
            LORA_TX_POWER,
            LORA_PREAMBLE_LEN);

        if (state != RADIOLIB_ERR_NONE)
        {
            if (_log)
            {
                _log->addLog("LoRaComm: Ошибка инициализации LoRa: " + String(state));
            }
            return false;
        }

        if (_log)
        {
            _log->addLog("LoRaComm: LoRa модуль успешно инициализирован.");
        }
        radio.setDio1Action(LoRaComm::onReceive); // Настройка прерывания на DIO1 для приема
        radio.startReceive();                     // Переходим в режим приема после инициализации
        if (_log)
        {
            _log->addLog("LoRaComm: Переход в режим приема.");
        }

        return true;
    }

    void applySettings(int sf, int cr, float bw)
    {
        radio.standby();
        radio.setSpreadingFactor(sf);
        radio.setCodingRate(cr);
        radio.setBandwidth(bw);
        radio.startReceive(); // обязательно после настройки
        currentSF = sf;
        currentCR = cr;
        currentBW = bw;
        _log->addLog("LoRaComm: Применены настройки LoRa: SF=" + String(sf) +
                     ", CR=" + String(cr) +
                     ", BW=" + String(bw, 1) + "kHz\n\n");
    }

    // Новый интерфейс: шлём любой Packet*
    template <typename P>
    bool sendPacket(const P &pkt, uint8_t receiverId)
    {
        // _log->addLog("[DEBUG] Sending packet to " + String(receiverId) + " packetType=" + String((char)pkt.packetType) + ", payloadLen=" + String(pkt.payloadLen));

        LoRaPacket frame;
        memset(&frame, 0, sizeof(frame));
        size_t len = packIntoLoRa(myDeviceId, receiverId, pkt, frame);
        // _log->addLog("[DEBUG] LoRa settings: SF=" + String(currentSF) +
        //              ", CR=" + String(currentCR) +
        //              ", BW=" + String(currentBW, 1) + "kHz" +
        //              ", TX=" + String(currentTX) +
        //              ", Freq=" + String(currentFreq, 3) + "MHz. Transmitt len=" + String(len));

        radio.standby();
        int st = radio.transmit((uint8_t *)&frame, len);
        radio.startReceive();
        // _log->addLog("PACKET DEBUG: type=" + String((char)pkt.packetType) +
        //              ", id=" + String(pkt.packetId) +
        //              ", payloadLen=" + String(pkt.payloadLen) +
        //              ", size=" + String(sizeof(pkt)));
        String hexDump;
        uint8_t *bytes = reinterpret_cast<uint8_t *>(&frame);
        for (size_t i = 0; i < len; ++i)
        {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", bytes[i]);
            hexDump += buf;
        }
        // _log->addLog("[DEBUG] Raw packet bytes: " + hexDump);

        _log->addLog("[DEBUG] sendPacket: Status=" + String(st) + ", len=" + String(len) + ":" + LoRaPacketToStr(frame) + "Raw packet bytes: " + hexDump + "\n\n");
        return (st == RADIOLIB_ERR_NONE);
    }

    // Парсит транспорт, возвращает raw hdr + payloadBuf
    bool parseReceivedPacket(uint8_t &senderId, PacketBase &hdr, uint8_t *payloadBuf)
    {
        if (!packetReceivedFlag)
            return false;
        packetReceivedFlag = false;

        int len = radio.getPacketLength();
        if (len > sizeof(LoRaPacket))
        {
            _log->addLog("[parseReceivedPacket] Invalid packet length=" + String(len));
            return false;
        }
        if (len <= 0)
        {
            return false; // Нет данных для чтения
        }

        uint8_t rawBuf[sizeof(LoRaPacket)];
        memset(rawBuf, 0, sizeof(rawBuf)); // 🔐 защита от мусора

        if (radio.readData(rawBuf, len) != RADIOLIB_ERR_NONE)
        {
            _log->addLog("[parseReceivedPacket] Failed to read raw data");
            return false;
        }

        radio.startReceive(); // обязательно сбрасываем

        LoRaPacket frame{};
        memcpy((uint8_t *)&frame, rawBuf, sizeof(LoRaPacket)); // ⚠️ копируем ВСЮ структуру, не len!

        //_log->addLog("MC: Received raw LoRa frame, trying to unpack..." + LoRaPacketToStr(frame));

        uint8_t receiverId = 0;
        if (!unpackLoRa(frame, senderId, receiverId, hdr, payloadBuf))
        {
            _log->addLog("[parseReceivedPacket]  unpackLoRa failed, senderId=" + String(senderId) +
                         ", receiverId=" + String(receiverId) +
                         ", packetType=" + String((char)hdr.packetType) +
                         ", packetId=" + String(hdr.packetId) +
                         ", payloadLen=" + String(hdr.payloadLen));
            return false;
        }
        else
        {
            if (receiverId == myDeviceId)
            {
                _log->addLog("[parseReceivedPacket] " + LoRaPacketToStr(frame));
            }
        }

        if (receiverId != myDeviceId && receiverId != 0xFF)
        {
            //_log->addLog("MC: Packet not for us (to=" + String(receiverId) + ")");
            return false;
        }

        return true;
    }

    // Статический флаг и статическая функция обработчика прерывания
    static volatile bool packetReceivedFlag;
    static void onReceive()
    {
        packetReceivedFlag = true;
    }
};

volatile bool LoRaComm::packetReceivedFlag = false;