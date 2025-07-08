// -----------------------------------------------------------------------------
// PacketAsaExchange — универсальный пакет для ASA
// Используется как для CMD_REQUEST_ASA, так и CMD_REPOSNCE_ASA
// -----------------------------------------------------------------------------
#pragma once
#include <stdint.h>
#include "PacketClasses.hpp"
#include "settings.h"

#include "LoRaCore.hpp"

// Для ASA-запросов и подтверждений (структура одна, тип разный)
struct PacketAsaExchange : public PacketBase {
    uint8_t profileIndex;

    PacketAsaExchange(CommandType type = CMD_REQUEST_ASA) {
        packetType = type;
        packetId = 0;
        payloadLen = sizeof(profileIndex);
        profileIndex = 0;
    }

    void setProfile(uint8_t index) {
        profileIndex = index;
        payloadLen = sizeof(profileIndex);
    }

    uint8_t getProfile() const {
        return profileIndex;
    }
};

// -----------------------------------------------------------------------------
// Утилиты ASA: отправка через очередь (второе ядро)
// -----------------------------------------------------------------------------

inline void sendAsaRequest(uint16_t packetId, uint8_t profileIndex, uint8_t receiver) {
    PacketAsaExchange pkt(CMD_REQUEST_ASA);
    pkt.packetId = packetId;
    pkt.setProfile(profileIndex);
    LoRaCore::sendPacketBase(receiver, pkt, &pkt.profileIndex, false);  // передаём payload
}

inline void sendAsaResponse(uint16_t packetId, uint8_t profileIndex, uint8_t receiver) {
    PacketAsaExchange pkt(CMD_REPOSNCE_ASA);
    pkt.packetId = packetId;
    pkt.setProfile(profileIndex);
    LoRaCore::sendPacketBase(receiver, pkt, &pkt.profileIndex, false);
}

// Пример разбора входящего ASA пакета
inline bool parseAsaPacket(const PacketBase& hdr, const uint8_t* buf, uint8_t& outProfileIndex) {
    if (hdr.payloadLen != sizeof(uint8_t)) return false;
    outProfileIndex = buf[0];
    return true;
}
