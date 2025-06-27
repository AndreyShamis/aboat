// PacketClasses.hpp
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <type_traits>


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


static constexpr size_t MAX_LORA_PAYLOAD = 40;



#pragma pack(push, 1)
struct LoRaPacket
{
    uint8_t senderId;
    uint8_t receiverId;
    uint8_t packetType;
    uint16_t packetId;
    uint16_t payloadLen;
    uint16_t crc; 
    uint8_t payload[MAX_LORA_PAYLOAD];
    
};
static_assert(sizeof(LoRaPacket) <= 64, "LoRaPacket слишком большой!");
#pragma pack(pop)


class PacketBase
{
public:
    uint8_t packetType;      // 'C','T','I','S','A','G','H'=Heartbeat
    uint8_t packetId;        // сквозной номер 0…255
    uint16_t payloadLen = 0; // длина тела (без заголовка и CRC)
    String toString() const {
        return "PacketBase[type=" + String((char)packetType) +
               ", id=" + String(packetId) +
               ", payloadLen=" + String(payloadLen) + "]";
    }
};
// static_assert(sizeof(PacketBase) == HEADER_SIZE, "PacketBase size");



String LoRaPacketToStr(const LoRaPacket& pkt) {
    String s;
    s += "LoRaPacket[";
    s += "sender=" + String(pkt.senderId);
    s += ", receiver=" + String(pkt.receiverId);
    s += ", type=" + String((char)pkt.packetType);
    s += ", id=" + String(pkt.packetId);
    s += ", payloadLen=" + String(pkt.payloadLen);
    s += ", crc=0x" + String(pkt.crc, HEX);
    s += ", packageId=" + String(pkt.packetId);
    s += ", payload=";

    for (int i = 0; i < pkt.payloadLen && i < MAX_LORA_PAYLOAD; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", pkt.payload[i]);
        s += buf;
    }

    s += "]";
    return s;
}

// Упаковка любого Packet* в LoRaPacket
template <typename P>
size_t packIntoLoRa(uint8_t senderId,
                    uint8_t receiverId,
                    const P &pkt,
                    LoRaPacket &out)
{
    memset(&out, 0, sizeof(out));
    // Копируем метаданные
    out.senderId = senderId;
    out.receiverId = receiverId;
    out.packetType = pkt.packetType;
    out.packetId = pkt.packetId;
    out.payloadLen = pkt.payloadLen;
    // ⚠️ Безопасная проверка
    if (pkt.payloadLen > MAX_LORA_PAYLOAD) {
        out.payloadLen = 0;
        return 0;
    }
    // Serial.println("[packIntoLoRa] Packing PacketBase: " + String((char)pkt.packetType) +
    //                ", id=" + String(pkt.packetId) +
    //                ", payloadLen=" + String(pkt.payloadLen));
    // Копируем тело (только payload)
    memcpy(out.payload,
           reinterpret_cast<const uint8_t *>(&pkt) + sizeof(PacketBase),
           pkt.payloadLen);
    // Считаем CRC16 по всему пакету до конца payload
    size_t crcLen = offsetof(LoRaPacket, payload) + pkt.payloadLen;
    out.crc = calcCRC16(reinterpret_cast<const uint8_t *>(&out), crcLen);
    // Serial.println("packIntoLoRa] We have crc" + String(out.crc) + " and PACK:"  + LoRaPacketToStr(out));
    uint8_t pSize = crcLen + sizeof(out.crc);
    //Serial.println("packIntoLoRa] We have crc" + String(out.crc) + " LEN:" + String(pSize) + " and PACK:"  + LoRaPacketToStr(out));
    return pSize; //
}

// Распаковка LoRaPacket обратно в PacketBase + payloadBuf
inline bool unpackLoRa(const LoRaPacket &in,
                       uint8_t &outSender,
                       uint8_t &outReceiver,
                       PacketBase &hdr,
                       uint8_t *payloadBuf)
{
    // 🔒 Проверка на превышение размера
    if (in.payloadLen > MAX_LORA_PAYLOAD) {
        return false;
    }
    //Serial.println("Unpacking LoRaPacket: " + LoRaPacketToStr(in));
    // ✅ Проверка CRC
    // size_t crcLen = offsetof(LoRaPacket, payload) + in.payloadLen;
    // Serial.printf("CRC check: len=%zu, crc=0x%04X\n", crcLen, in.crc);
    // if (calcCRC16(reinterpret_cast<const uint8_t *>(&in), crcLen) != in.crc) {
    //     return false;
    // }

    // 📦 Извлекаем метаданные
    outSender = in.senderId;
    outReceiver = in.receiverId;
    hdr.packetType = in.packetType;
    hdr.packetId = in.packetId;
    hdr.payloadLen = in.payloadLen;

    // 🔄 Копируем payload
    memcpy(payloadBuf, in.payload, in.payloadLen);

    return true;
}



// ----------------- Максимальные константы -------------------
static constexpr size_t MAX_ARGS = 6;    // до 6 аргументов в команде
static constexpr size_t CRC_SIZE = 2;    // 2 байта CRC
static constexpr size_t HEADER_SIZE = 4; // размер PacketBase

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
bool parseAsaRequest(const uint8_t *buf, size_t len, uint8_t &profileIndex) {
    if (len != sizeof(uint8_t)) return false;
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
};
class PacketAsaApprove : public PacketAsaRequest{};

// Пакет-команда: cmdId + произв.число аргументов
class PacketCommand : public PacketBase
{
public:
    uint8_t cmdId;         // из CommandID
    uint8_t argCount;      // реально используемых args
    int8_t args[MAX_ARGS]; // каждый –128…127
};
static_assert(sizeof(PacketCommand) == HEADER_SIZE + 2 + MAX_ARGS,
              "PacketCommand size");

// Телеметрия: пример – скорость, курс, уровень батареи
class PacketTelemetry : public PacketBase
{
public:
    uint8_t speed;     // 0–255
    uint8_t course;    // 0–179 (0–359°/2)
    uint8_t battLevel; // 0–100%
};
static_assert(sizeof(PacketTelemetry) == HEADER_SIZE + 3,
              "PacketTelemetry size");

// Инфо о двигателе: RPM и температура
class PacketInfoEngine : public PacketBase
{
public:
    int16_t rpm; // 0–20000
    int8_t temp; // –40…85
};
static_assert(sizeof(PacketInfoEngine) == HEADER_SIZE + 3,
              "PacketInfoEngine size");

// Статус: битовые флаги
class PacketStatus : public PacketBase
{
public:
    uint8_t statusFlags; // бит0=OK,1=LowBatt,2=SensorErr…
};
static_assert(sizeof(PacketStatus) == HEADER_SIZE + 1,
              "PacketStatus size");

// Подтверждение (ACK)
class PacketAck : public PacketBase
{
public:
    uint8_t packetType = CMD_ACK; // универсальное «ок»
    uint8_t ackedId; // packetId того, что подтверждаем
};

// Конфиг-пакет (изменение параметра)
class PacketConfig : public PacketBase
{
public:
    uint8_t paramId;
    int8_t value;
};
static_assert(sizeof(PacketConfig) == HEADER_SIZE + 2,
              "PacketConfig size");

// Навигация: GPS
class PacketNav : public PacketBase
{
public:
    int32_t lat;   // 1e-7°
    int32_t lon;   // 1e-7°
    uint16_t hdop; // HDOP×100
};
static_assert(sizeof(PacketNav) == HEADER_SIZE + 10,
              "PacketNav size");

class PacketHeartbeat : public PacketBase
{
public:
    uint32_t count; // произвольный счётчик
};
static_assert(sizeof(PacketHeartbeat) == HEADER_SIZE + 4,
              "PacketHeartbeat size");

#pragma pack(pop)
// ----------------- Сериализация / Десериализация -----------------
template <typename T>
struct PacketSerializer
{
    // Копируем объект + дописываем CRC16
    static size_t serialize(const T &pkt, uint8_t *buf)
    {
        size_t pktSize = sizeof(T);
        memcpy(buf, &pkt, pktSize);
        uint16_t crc = calcCRC16(buf, pktSize);
        buf[pktSize + 0] = uint8_t(crc >> 8);
        buf[pktSize + 1] = uint8_t(crc & 0xFF);
        return pktSize + CRC_SIZE;
    }
    // Проверяем CRC и возвращаем true, если корректен
    static bool validate(const uint8_t *buf, size_t totalLen)
    {
        if (totalLen < CRC_SIZE)
            return false;
        size_t pktSize = totalLen - CRC_SIZE;
        uint16_t recvCrc = (uint16_t(buf[pktSize]) << 8) | buf[pktSize + 1];
        return recvCrc == calcCRC16(buf, pktSize);
    }
};
