#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "settings.h" // Конфигурация LoRa модуля
#include "LogInterface.hpp" // Интерфейс для логирования

// ВАЖНО: Эти определения должны быть доступны до компиляции LoRaComm.hpp.
// Их лучше разместить в отдельном файле settings.h, который включается здесь и в main .ino файле.
// Или же прямо здесь, но тогда убедитесь, что они не конфликтуют с другими определениями.

// --- СВОЙ ПРОТОКОЛ СООБЩЕНИЙ ---
enum CommandType : uint8_t {
    CMD_NONE = 0,
    CMD_SET_SPEED = 1,
    CMD_STOP_ENGINE = 2,
    CMD_START_ENGINE = 3,
    CMD_GET_BOAT_STATUS = 4,
    CMD_BOAT_STATUS_REPORT = 5,
    CMD_ACK = 6
};

// Структура для всех LoRa пакетов
struct LoRaPacket {
    uint8_t   senderId;
    uint8_t   receiverId;
    uint8_t   commandType;
    int16_t   value;
    uint16_t  packetId;
    uint8_t   checksum;
};

// Функция для расчета простой контрольной суммы XOR
static uint8_t calculateChecksum(const LoRaPacket& packet) {
    uint8_t cs = 0;
    cs ^= packet.senderId;
    cs ^= packet.receiverId;
    cs ^= packet.commandType;
    cs ^= (uint8_t)(packet.value & 0xFF);
    cs ^= (uint8_t)((packet.value >> 8) & 0xFF);
    cs ^= (uint8_t)(packet.packetId & 0xFF);
    cs ^= (uint8_t)((packet.packetId >> 8) & 0xFF);
    return cs;
}

class LoRaComm {
private:
    uint8_t myDeviceId;
    Module* _module;
    SX1262 radio; // Объект LoRa модуля

    LogInterface* _log; // Указатель на интерфейс логирования

public:
    // Конструктор: принимает ID устройства и указатель на объект логирования
    LoRaComm(uint8_t deviceId, LogInterface* logger) :
        myDeviceId(deviceId),
        _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY)), // Создаем модуль без указания SPI здесь
        radio(_module),
        _log(logger)
    {}

    // Деструктор для освобождения динамически выделенной памяти
    ~LoRaComm() {
        delete _module;
    }

    // Инициализация LoRa модуля
    bool begin() {
        if (_log) {
            _log->addLog("LoRaComm: Инициализация SPI для LoRa...");
        }

        // Инициализация SPI, как в вашем рабочем коде из Boat
        // Это делается здесь, чтобы LoRaComm полностью отвечал за свои зависимости SPI
        SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

        // Программный сброс LoRa модуля, как в вашем рабочем коде из Boat
        digitalWrite(LORA_RST, LOW);
        delay(10);
        digitalWrite(LORA_RST, HIGH);
        delay(10);

        pinMode(LORA_BUSY, INPUT); // Устанавливаем BUSY пин как вход
        if (_log) {
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
            LORA_PREAMBLE_LEN
        );

        if (state != RADIOLIB_ERR_NONE) {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка инициализации LoRa: " + String(state));
            }
            return false;
        }

        if (_log) {
            _log->addLog("LoRaComm: LoRa модуль успешно инициализирован.");
        }

        // Настройка прерывания на DIO1 для приема
        radio.setDio1Action(LoRaComm::onReceive);
        // Переходим в режим приема после инициализации
        radio.startReceive();
        if (_log) {
            _log->addLog("LoRaComm: Переход в режим приема.");
        }

        return true;
    }

    // Метод для отправки пакета
    bool sendPacket(LoRaPacket& packet) {
        packet.senderId = myDeviceId; // Устанавливаем ID отправителя
        packet.checksum = calculateChecksum(packet); // Рассчитываем контрольную сумму

        if (_log) {
            _log->addLog("LoRaComm: Отправка пакета. ID: " + String(packet.packetId) +
                         ", Тип: " + String(packet.commandType) + ", Значение: " + String(packet.value));
        }

        // Временно выключаем режим приема, чтобы отправить
        radio.standby();

        int state = radio.transmit((uint8_t*)&packet, sizeof(packet));

        // Сразу после отправки возвращаемся в режим приема
        radio.startReceive();

        if (state == RADIOLIB_ERR_NONE) {
            if (_log) {
                _log->addLog("LoRaComm: Пакет успешно отправлен.");
            }
            return true;
        } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка отправки: Таймаут (код: " + String(state) + ").");
            }
        } else {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка отправки: " + String(state));
            }
        }
        return false;
    }

    // Метод для обработки полученного пакета (вызывать в loop())
    bool parseReceivedPacket(LoRaPacket& outPacket) {
        if (!packetReceivedFlag) {
            return false; // Нет нового пакета
        }

        packetReceivedFlag = false; // Сбрасываем флаг

        // Проверяем наличие пакета и его размер
        int packetSize = radio.getPacketLength();
        if (packetSize == 0) {
            //_log->addLog("LoRaComm: Получен пустой пакет."); // Слишком много сообщений
            return false;
        }
        if (packetSize != sizeof(LoRaPacket)) {
            if (_log) {
                _log->addLog("LoRaComm: Неверный размер пакета: " + String(packetSize) + " (ожидалось: " + String(sizeof(LoRaPacket)) + ")");
            }
            return false;
        }

        // Читаем пакет
        int state = radio.readData((uint8_t*)&outPacket, sizeof(outPacket));

        if (state == RADIOLIB_ERR_NONE) {
            // Проверяем контрольную сумму
            uint8_t receivedChecksum = outPacket.checksum;
            outPacket.checksum = 0; // Временно обнуляем для расчета
            uint8_t calculatedChecksum = calculateChecksum(outPacket);
            outPacket.checksum = receivedChecksum; // Восстанавливаем

            if (receivedChecksum != calculatedChecksum) {
                if (_log) {
                    _log->addLog("LoRaComm: Ошибка контрольной суммы! Получено: " + String(receivedChecksum) + ", Ожидалось: " + String(calculatedChecksum));
                }
                return false;
            }

            if (_log) {
                _log->addLog("LoRaComm: Пакет получен. ID: " + String(outPacket.packetId) +
                             ", Отправитель: " + String(outPacket.senderId) +
                             ", Тип: " + String(outPacket.commandType) +
                             ", Значение: " + String(outPacket.value) +
                             ", RSSI: " + String(radio.getRSSI()) +
                             ", SNR: " + String(radio.getSNR()));
            }
            return true;
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            if (_log) {
                _log->addLog("LoRaComm: CRC ошибка в пакете (код: " + String(state) + ").");
            }
        } else {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка при чтении пакета: " + String(state));
            }
        }
        return false;
    }

    // Статический флаг и статическая функция обработчика прерывания
    static volatile bool packetReceivedFlag;
    static void onReceive() {
        packetReceivedFlag = true;
    }
};

volatile bool LoRaComm::packetReceivedFlag = false;