// PacketClasses.hpp
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <type_traits>
#include "../include/settings.h"

// ---------------- CRC16-CCITT (полином 0x1021) ----------------
static uint16_t calcCRC16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static constexpr size_t MAX_LORA_PAYLOAD = 85;

// ----------------- Максимальные константы -------------------
static constexpr size_t MAX_ARGS = 6;                                             // до 6 аргументов в команде
static constexpr size_t CRC_SIZE = 2;                                             // 2 байта CRC
static constexpr size_t HEADER_SIZE = sizeof(uint8_t) * 1 + sizeof(uint16_t) * 2; // = 5

#pragma pack(push, 1)
struct LoRaPacket
{
    uint8_t senderId;
    uint8_t receiverId;
    uint8_t packetType;
    PacketId_t packetId;    // Унифицированный тип для ID пакета
    uint8_t payloadLen = 0;
    // uint16_t crc;
    uint8_t payload[MAX_LORA_PAYLOAD];

    // Getter functions
    uint8_t getSenderId() const { return senderId; }
    uint8_t getReceiverId() const { return receiverId; }
    uint8_t getPacketType() const { return packetType; }

    // Setter functions
    void setSenderId(uint8_t id) { senderId = id; }
    void setReceiverId(uint8_t id) { receiverId = id; }
};
static_assert(sizeof(LoRaPacket) <= 150, "LoRaPacket слишком большой!");
#pragma pack(pop)

class PacketBase
{
public:
    uint8_t packetType;     // 'C','T','I','S','A','G','H'=Heartbeat
    PacketId_t packetId;    // сквозной номер 0…255 (унифицированный тип)
    uint8_t payloadLen = 0; // длина тела (без заголовка и CRC)
    
    // Remove toString() method to prevent vtable pollution
    // Use free function instead: PacketBaseToString(base)
};

// Free function for converting PacketBase to string (safer than member function)
inline String PacketBaseToString(const PacketBase& base) {
    return "PacketBase[type=" + String((char)base.packetType) +
           ", id=" + String(base.packetId) +
           ", payloadLen=" + String(base.payloadLen) + "]";
}
// static_assert(sizeof(PacketBase) == HEADER_SIZE, "PacketBase size");

inline String LoRaPacketToStr(const LoRaPacket &pkt)
{
    String s;
    s += "[" + String(pkt.senderId) + "->" + String(pkt.receiverId) + "], T=[" + String((char)pkt.packetType);
    s += "/" + String((int)pkt.packetType) + "], id=" + String(pkt.packetId);
    s += ", plLen=" + String(pkt.payloadLen);
    // s += ", crc=0x" + String(pkt.crc, HEX);
    if (pkt.payloadLen > MAX_LORA_PAYLOAD)
    {
        s += ", pl=❌CORRUPTED_LEN=" + String(pkt.payloadLen);
    }
    else if (pkt.payloadLen > 0)
    {
        s += ", pl=";
        for (int i = 0; i < pkt.payloadLen && i < MAX_LORA_PAYLOAD; i++)
        {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", pkt.payload[i]);
            s += buf;
        }
    }

    s += "]";
    return s;
}

// ----------------- Коды команд -----------------------------
enum CommandID : uint8_t
{
    CMD_SET_MOTOR = 1,       // args: [motorIndex, power]
    CMD_SET_RUDDER = 2,      // args: [angle]
    CMD_START_TELEMETRY = 3, // args: []
    CMD_STOP_TELEMETRY = 4,
    CMD_REQUEST_STATUS = 5,
    CMD_REQUEST_ENGINE = 6,
    CMD_HEARTBEAT = 7,
    // … добавьте свои
};
inline bool parseAsaRequest(const uint8_t *buf, size_t len, uint8_t &profileIndex)
{
    if (len != sizeof(uint8_t))
        return false;
    profileIndex = *buf;
    return true;
}

// ---------------- Пакеты (упакованные без выравнивания) ----------------
#pragma pack(push, 1)

// Базовый заголовок любого пакета

class PacketAsaRequest : public PacketBase
{
public:
    uint8_t profileIndex;
    
    PacketAsaRequest() : profileIndex(0) {
        packetType = CMD_REQUEST_ASA;
        payloadLen = sizeof(profileIndex);
    }
};
class PacketAsaApprove : public PacketAsaRequest
{
public:
    PacketAsaApprove() {
        packetType = CMD_REPOSNCE_ASA; // Исправляем тип пакета
        // payloadLen наследуется от PacketAsaRequest
    }
};

// Пакет-команда: cmdId + произв.число аргументов
class PacketCommand : public PacketBase
{
public:
    uint8_t cmdId;         // из CommandID
    uint8_t argCount;      // реально используемых args
    int8_t args[MAX_ARGS]; // каждый –128…127
    
    PacketCommand() : cmdId(0), argCount(0) {
        packetType = CMD_COMMAND_STRING;
        payloadLen = sizeof(cmdId) + sizeof(argCount) + sizeof(args);
        for(int i = 0; i < MAX_ARGS; i++) args[i] = 0;
    }
};

// Телеметрия: пример – скорость, курс, уровень батареи
class PacketTelemetry : public PacketBase
{
public:
    uint8_t speed;     // 0–255
    uint8_t course;    // 0–179 (0–359°/2)
    uint8_t battLevel; // 0–100%
    
    PacketTelemetry() : speed(0), course(0), battLevel(0) {
        packetType = CMD_TELEMETRY_FRAGMENT;
        payloadLen = sizeof(speed) + sizeof(course) + sizeof(battLevel);
    }
};

// Инфо о двигателе: RPM и температура
class PacketInfoEngine : public PacketBase
{
public:
    int16_t rpm; // 0–20000
    int8_t temp; // –40…85
    
    PacketInfoEngine() : rpm(0), temp(0) {
        packetType = CMD_INFO_ENGINE;
        payloadLen = sizeof(rpm) + sizeof(temp);
    }
};


struct PacketRssiReport : PacketBase
{
public:
    float rawRssi = 0;
    float smoothedRssi = 0;
    
    PacketRssiReport() : rawRssi(0.0f), smoothedRssi(0.0f) {
        packetType = CMD_RSSI_REPORT;
        payloadLen = sizeof(rawRssi) + sizeof(smoothedRssi);
    }
};


// Статус: битовые флаги
class PacketStatus : public PacketBase
{
public:
    uint8_t statusFlags; // бит0=OK,1=LowBatt,2=SensorErr…
    
    PacketStatus() : statusFlags(0) {
        packetType = CMD_STATUS;
        payloadLen = sizeof(statusFlags);
    }
};
// static_assert(sizeof(PacketStatus) == HEADER_SIZE + 1,
//               "PacketStatus size");

// Подтверждение (ACK)
class PacketAck : public PacketBase
{
public:
    PacketId_t ackedId; // packetId того, что подтверждаем (унифицированный тип)
    
    PacketAck() : ackedId(0) {
        packetType = CMD_ACK;
        payloadLen = sizeof(ackedId);
    }
};

// Агрегированное подтверждение (BULK ACK) - до 10 ACK в одном пакете
class PacketBulkAck : public PacketBase
{
public:
    uint8_t count;                          // количество ACK (1-10)
    PacketId_t ackedIds[10];                // массив packetId для подтверждения (унифицированный тип)
    
    PacketBulkAck() : count(0) {
        packetType = CMD_BULK_ACK;
        payloadLen = sizeof(count);
        for(int i = 0; i < 10; i++) ackedIds[i] = 0;
    }
    
    bool addAck(PacketId_t packetId) {
        if (count >= 10) return false;
        
        // Проверяем, нет ли уже такого ID в списке (избегаем дублированных ACK)
        for (uint8_t i = 0; i < count; i++) {
            if (ackedIds[i] == packetId) {
                // ID уже есть в списке - не добавляем дубликат
                return true; // Возвращаем true, так как операция "успешна" (ID уже учтен)
            }
        }
        
        // ID уникальный, добавляем его
        ackedIds[count] = packetId;
        count++;
        payloadLen = sizeof(count) + (count * sizeof(PacketId_t));
        return true;
    }
    
    void clear() {
        count = 0;
        payloadLen = sizeof(count);
        for(int i = 0; i < 10; i++) ackedIds[i] = 0;
    }
    
    bool isFull() const { return count >= 10; }
    bool isEmpty() const { return count == 0; }

    // Диагностический метод для отображения содержимого BULK ACK
    String getDebugInfo() const {
        String result = "BulkACK(" + String(count) + "): ";
        for (uint8_t i = 0; i < count; i++) {
            if (i > 0) result += ",";
            result += String(ackedIds[i]);
        }
        if (count == 0) result += "empty";
        return result;
    }

    // Проверка на наличие дублированных ID (для отладки)
    bool hasDuplicates() const {
        for (uint8_t i = 0; i < count; i++) {
            for (uint8_t j = i + 1; j < count; j++) {
                if (ackedIds[i] == ackedIds[j]) {
                    return true;
                }
            }
        }
        return false;
    }
};

// Конфиг-пакет (изменение параметра)
class PacketConfig : public PacketBase
{
public:
    uint8_t paramId;
    int8_t value;
    
    PacketConfig() : paramId(0), value(0) {
        packetType = CMD_CONFIG;
        payloadLen = sizeof(paramId) + sizeof(value);
    }
};

// Навигация: GPS
class PacketNav : public PacketBase
{
public:
    int32_t lat;   // 1e-7°
    int32_t lon;   // 1e-7°
    uint16_t hdop; // HDOP×100
    
    PacketNav() : lat(0), lon(0), hdop(0) {
        packetType = CMD_NAV;
        payloadLen = sizeof(lat) + sizeof(lon) + sizeof(hdop);
    }
};

class PacketHeartbeat : public PacketBase
{
public:
    uint32_t count; // произвольный счётчик
    
    PacketHeartbeat() : count(0) {
        packetType = CMD_PING; // или другой подходящий тип для heartbeat
        payloadLen = sizeof(count);
    }
};

// Отдельные классы для PING и PONG
class PacketPing : public PacketBase
{
public:
    PacketPing() {
        packetType = CMD_PING;
        payloadLen = 0; // PING не содержит данных
    }
};

class PacketPong : public PacketBase
{
public:
    PacketPong() {
        packetType = CMD_PONG;
        payloadLen = 0; // PONG не содержит данных
    }
};

// Пакет для запроса информации
class PacketRequestInfo : public PacketBase
{
public:
    uint8_t requestType; // тип запрашиваемой информации
    
    PacketRequestInfo() : requestType(0) {
        packetType = CMD_REQUEST_INFO;
        payloadLen = sizeof(requestType);
    }
};

// Пакет для ответов на команды
class PacketCommandResponse : public PacketBase
{
public:
    PacketCommandResponse() {
        packetType = CMD_COMMAND_RESPONSE;
        payloadLen = 0; // размер будет установлен при отправке
    }
};

// Пакет для фрагментов телеметрии (JSON данные)
class PacketTelemetryFragment : public PacketBase
{
public:
    PacketTelemetryFragment() {
        packetType = CMD_TELEMETRY_FRAGMENT;
        payloadLen = 0; // размер будет установлен при отправке
    }
};

#pragma pack(pop)
