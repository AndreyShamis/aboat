#pragma once

#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include "settings.h" // Убедитесь, что LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE_LEN определены в settings.h
#include "LogInterface.hpp"

// --- ПИНЫ HELTEC WIRELESS STICK LITE V3 ДЛЯ SX1262 ---
// Эти дефайны должны быть в settings.h
// #define LORA_SS         8
// #define LORA_DIO1       14
// #define LORA_RST        12
// #define LORA_BUSY       13
// УПРАВЛЕНИЕ RF-ПЕРЕКЛЮЧАТЕЛЕМ ИНТЕГРИРОВАНО И НЕ ТРЕБУЕТ ВНЕШНЕГО GPIO

// --- ПАРАМЕТРЫ LORA ---
// Эти дефайны должны быть в settings.h
// #define LORA_FREQUENCY      868.0       // МГц (для Европы), 915.0 для США/Австралии
// #define LORA_BANDWIDTH      125.0       // кГц (можно попробовать 62.5 для большей дальности, но медленнее)
// #define LORA_SF             11          // Spreading Factor (от 6 до 12, 11/12 для дальности)
// #define LORA_CODING_RATE    7           // Coding Rate (от 5 до 8, 7/8 для надежности)
// #define LORA_SYNC_WORD      SX126X_SYNC_WORD_PRIVATE // Важно, чтобы совпадал на всех устройствах
// #define LORA_TX_POWER       20          // dBm (макс. 22 для SX1262, проверьте свои региональные ограничения)
// #define LORA_PREAMBLE_LEN   8           // Длина преамбулы (обычно 8)


// --- СВОЙ ПРОТОКОЛ СООБЩЕНИЙ ---
enum CommandType : uint8_t { // Явное указание размера для экономии памяти
  CMD_NONE = 0,
  CMD_SET_SPEED = 1,      // Установить скорость (value: 0-100%)
  CMD_STOP_ENGINE = 2,    // Остановить двигатель
  CMD_START_ENGINE = 3,   // Запустить двигатель
  CMD_GET_BOAT_STATUS = 4, // Запрос статуса лодки (от базовой станции)
  CMD_BOAT_STATUS_REPORT = 5, // Отчет о статусе лодки (от лодки, value может быть кодом статуса)
  CMD_ACK = 6             // Подтверждение получения пакета (value = packetId подтвержденного пакета)
};

// Структура для всех LoRa пакетов
struct LoRaPacket {
  uint8_t   senderId;     // ID отправителя (напр., 0x01 для лодки, 0x02 для базы)
  uint8_t   receiverId;   // ID получателя (напр., 0x01 для лодки, 0x02 для базы, 0xFF для Broadcast)
  uint8_t   commandType;  // Тип команды/сообщения (из CommandType)
  int16_t   value;        // Параметр команды (напр., значение скорости, или ID подтверждаемого пакета для ACK)
  uint16_t  packetId;     // Уникальный ID пакета для отслеживания (инкрементируется отправителем)
  uint8_t   checksum;     // Простая контрольная сумма XOR всех предыдущих байтов
};

// Функция для расчета простой контрольной суммы XOR
// Она должна быть определена один раз в глобальной области или статически в .cpp файле
// Если она в .h, то должна быть inline или static
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
    SX1262 radio;

    // Pointer to the Boat instance that created this LoRaComm object
    LogInterface* _log;
public:
    // Constructor now takes a pointer to the Boat instance
    LoRaComm(uint8_t deviceId, LogInterface* logger) :
        myDeviceId(deviceId),
        _module(new Module(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY, SPI)),
        radio(_module),
        _log(logger) // Initialize the Boat pointer
    {}

    // Destructor to free dynamically allocated Module
    ~LoRaComm() {
        delete _module;
    }

    bool begin() {
        pinMode(LORA_SS, OUTPUT);
        digitalWrite(LORA_SS, HIGH);

        // Use the _boat pointer to call its addLog method
        if (_log) { // Always good to check for nullptr
            _log->addLog("LoRaComm: Инициализация SPI и LoRa модуля...");
        }

        int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE_LEN);

        if (state != RADIOLIB_ERR_NONE) {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка инициализации LoRa: " + String(state));
            }
            return false;
        }

        if (_log) {
            _log->addLog("LoRaComm: LoRa модуль успешно инициализирован.");
        }
        
        radio.setDio1Action(LoRaComm::onReceive); 

        return true;
    }

    // Method to send a packet
    bool sendPacket(LoRaPacket& packet) {
        if (_log) {
            _log->addLog("LoRaComm: Отправка пакета. ID: " + String(packet.packetId) + ", Команда: " + String(packet.value));
        }
        int state = radio.transmit((uint8_t*)&packet, sizeof(packet));
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

    // Method to receive a packet
    bool receivePacket(LoRaPacket& outPacket) {
        if (_log) {
            _log->addLog("LoRaComm: Ожидание пакета...");
        }
        int state = radio.receive((uint8_t*)&outPacket, sizeof(outPacket));
        if (state == RADIOLIB_ERR_NONE) {
            if (_log) {
                _log->addLog("LoRaComm: Пакет получен. ID: " + String(outPacket.packetId) + ", Команда: " + String(outPacket.value));
            }
            return true;
        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            if (_log) {
                _log->addLog("LoRaComm: Таймаут приема (код: " + String(state) + ").");
            }
            return false;
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            if (_log) {
                _log->addLog("LoRaComm: CRC ошибка в пакете (код: " + String(state) + ").");
            }
            return false;
        } else {
            if (_log) {
                _log->addLog("LoRaComm: Ошибка приема: " + String(state));
            }
            return false;
        }
    }

    // Loop method to handle packet reception flag
    void loop() {
        if (packetReceivedFlag) {
            packetReceivedFlag = false;
            if (_log) {
                _log->addLog("LoRaComm: Packet received callback triggered.");
            }
            int state = radio.startReceive();
            if (state == RADIOLIB_ERR_NONE) {
                // if (_boat) { _boat->addLog("LoRaComm: Снова в режим приема."); } // Too chatty
            } else {
                if (_log) {
                    _log->addLog("LoRaComm: Ошибка при переходе в режим приема после коллбэка: " + String(state));
                }
            }
        }
    }

    // Static flag and static interrupt handler function
    static volatile bool packetReceivedFlag;
    static void onReceive() {
        packetReceivedFlag = true;
    }
};

volatile bool LoRaComm::packetReceivedFlag = false;

