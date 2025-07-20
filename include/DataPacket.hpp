#pragma once
#include <Arduino.h>
#include "BoatSettings.hpp"
#include "settings.h"
#include "PacketClasses.hpp"
#include <cstring>

// 🚀 Новые CommandType для структурированных данных (добавляем к существующим)
// Используем существующую систему CommandType из settings.h
constexpr CommandType CMD_STRUCTURED_HEARTBEAT = (CommandType)'H';  // Структурированный heartbeat
constexpr CommandType CMD_STRUCTURED_GPS = (CommandType)'P';        // GPS данные (Position)
constexpr CommandType CMD_STRUCTURED_MOTORS = (CommandType)'M';     // Моторы и управление
constexpr CommandType CMD_STRUCTURED_SENSORS = (CommandType)'E';    // Датчики (sEnsors)
constexpr CommandType CMD_STRUCTURED_NAVIGATION = (CommandType)'N'; // Навигация (используем существующий)
constexpr CommandType CMD_STRUCTURED_FRAGMENT = (CommandType)'X';   // Фрагмент больших данных

// Заголовок для фрагментированной передачи структурированных данных
struct StructuredFragmentHeader {
    uint16_t messageId = 0;      // Идентификатор сообщения
    uint8_t fragmentIndex = 0;   // Номер фрагмента (0-based)
    uint8_t totalFragments = 0;  // Общее количество фрагментов
    uint16_t totalSize = 0;      // Общий размер данных
    
    static constexpr size_t size() { return 6; }
    
    void serialize(uint8_t* buffer) const {
        buffer[0] = (uint8_t)(messageId & 0xFF);
        buffer[1] = (uint8_t)(messageId >> 8);
        buffer[2] = fragmentIndex;
        buffer[3] = totalFragments;
        buffer[4] = (uint8_t)(totalSize & 0xFF);
        buffer[5] = (uint8_t)(totalSize >> 8);
    }
    
    void deserialize(const uint8_t* buffer) {
        messageId = buffer[0] | (buffer[1] << 8);
        fragmentIndex = buffer[2];
        totalFragments = buffer[3];
        totalSize = buffer[4] | (buffer[5] << 8);
    }
};

// 🚀 ULTRA-COMPACT heartbeat - только критически важные данные (26 байт)
struct UltraCompactHeartbeat {
    uint32_t timestamp = 0;        // 4 байта
    float latitude = 0.0f;         // 4 байта  
    float longitude = 0.0f;        // 4 байта
    uint8_t loraProfile = 0;       // 1 байт
    int8_t rssi = -120;           // 1 байт (-120 до -1 dBm)
    uint8_t batteryPercent = 0;    // 1 байт
    uint8_t systemHealth = 100;    // 1 байт
    uint8_t flags = 0;            // 1 байт: GPS_FIX|EMERGENCY|NAV_ACTIVE|MC_CONNECTED
    
    void setGPSFix(bool value) { flags = (flags & ~0x01) | (value ? 0x01 : 0x00); }
    void setEmergencyStop(bool value) { flags = (flags & ~0x02) | (value ? 0x02 : 0x00); }
    void setNavigationActive(bool value) { flags = (flags & ~0x04) | (value ? 0x04 : 0x00); }
    void setMissionControlConnected(bool value) { flags = (flags & ~0x08) | (value ? 0x08 : 0x00); }
    
    bool getGPSFix() const { return (flags & 0x01) != 0; }
    bool getEmergencyStop() const { return (flags & 0x02) != 0; }
    bool getNavigationActive() const { return (flags & 0x04) != 0; }
    bool getMissionControlConnected() const { return (flags & 0x08) != 0; }
    
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        memcpy(buffer + offset, &timestamp, 4); offset += 4;
        memcpy(buffer + offset, &latitude, 4); offset += 4;
        memcpy(buffer + offset, &longitude, 4); offset += 4;
        buffer[offset++] = loraProfile;
        buffer[offset++] = (uint8_t)rssi;
        buffer[offset++] = batteryPercent;
        buffer[offset++] = systemHealth;
        buffer[offset++] = flags;
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        memcpy(&timestamp, buffer + offset, 4); offset += 4;
        memcpy(&latitude, buffer + offset, 4); offset += 4;
        memcpy(&longitude, buffer + offset, 4); offset += 4;
        loraProfile = buffer[offset++];
        rssi = (int8_t)buffer[offset++];
        batteryPercent = buffer[offset++];
        systemHealth = buffer[offset++];
        flags = buffer[offset++];
    }
    
    static constexpr size_t serializedSize() { return 17; } // Всего 17 байт!
    
    void fillFromBoatSettings(const BoatSettings& settings) {
        timestamp = settings.timestamp;
        latitude = settings.gps.position.latitude;
        longitude = settings.gps.position.longitude;
        loraProfile = settings.lora.currentProfile;
        rssi = (int8_t)max(-120, min(-1, (int)settings.lora.rssi));
        batteryPercent = settings.sensors.batteryPercent;
        systemHealth = settings.systemHealth;
        setGPSFix(settings.gps.hasFix);
        setEmergencyStop(settings.motors.emergencyStop);
        setNavigationActive(settings.navigation.navigationActive);
        setMissionControlConnected(settings.lora.missionControlConnected);
    }
    
    String toString() const {
        return "UHB: " + String(latitude, 6) + "," + String(longitude, 6) + 
               ", P:" + String(loraProfile) + 
               ", RSSI:" + String(rssi) + 
               ", Bat:" + String(batteryPercent) + "%" +
               ", Health:" + String(systemHealth) + "%";
    }
};

// Класс для работы со структурированными данными через существующую систему LoRaCore
class StructuredDataManager {
private:
    static uint16_t messageIdCounter;
    
public:
    // Максимальный размер данных в одном пакете (с учетом заголовка LoRaPacket)
    static constexpr size_t MAX_STRUCTURED_DATA_SIZE = 85; // MAX_LORA_PAYLOAD из PacketClasses.hpp
    static constexpr size_t MAX_FRAGMENT_DATA = MAX_STRUCTURED_DATA_SIZE - 6; // StructuredFragmentHeader::size()
    
    // Создать ультра-компактный heartbeat пакет (17 байт данных!)
    static bool createHeartbeatPacket(const BoatSettings& settings, PacketBase& packetBase, uint8_t* payload, size_t& payloadSize) {
        UltraCompactHeartbeat hb;
        hb.fillFromBoatSettings(settings);
        
        packetBase.packetType = CMD_STRUCTURED_HEARTBEAT;
        packetBase.payloadLen = UltraCompactHeartbeat::serializedSize();
        
        hb.serialize(payload);
        payloadSize = packetBase.payloadLen;
        return true;
    }
    
    // Создать пакет с GPS данными
    static bool createGPSPacket(const GPSStatus& gps, PacketBase& packetBase, uint8_t* payload, size_t& payloadSize) {
        packetBase.packetType = CMD_STRUCTURED_GPS;
        packetBase.payloadLen = GPSStatus::serializedSize();
        
        if (packetBase.payloadLen > MAX_STRUCTURED_DATA_SIZE) {
            return false;
        }
        
        gps.serialize(payload);
        payloadSize = packetBase.payloadLen;
        return true;
    }
    
    // Создать пакет с данными моторов
    static bool createMotorPacket(const MotorStatus& motors, PacketBase& packetBase, uint8_t* payload, size_t& payloadSize) {
        packetBase.packetType = CMD_STRUCTURED_MOTORS;
        packetBase.payloadLen = MotorStatus::serializedSize();
        
        if (packetBase.payloadLen > MAX_STRUCTURED_DATA_SIZE) {
            return false;
        }
        
        motors.serialize(payload);
        payloadSize = packetBase.payloadLen;
        return true;
    }
    
    // Создать пакет с данными датчиков
    static bool createSensorPacket(const SensorStatus& sensors, PacketBase& packetBase, uint8_t* payload, size_t& payloadSize) {
        packetBase.packetType = CMD_STRUCTURED_SENSORS;
        packetBase.payloadLen = SensorStatus::serializedSize();
        
        if (packetBase.payloadLen > MAX_STRUCTURED_DATA_SIZE) {
            return false;
        }
        
        sensors.serialize(payload);
        payloadSize = packetBase.payloadLen;
        return true;
    }
    
    // Создать фрагментированный пакет для больших данных
    static bool createFragmentPacket(const uint8_t* data, size_t dataSize, 
                                     uint8_t fragmentIndex, uint8_t totalFragments,
                                     PacketBase& packetBase, uint8_t* payload, size_t& payloadSize) {
        if (fragmentIndex >= totalFragments) {
            return false;
        }
        
        StructuredFragmentHeader fragHeader;
        fragHeader.messageId = messageIdCounter;
        fragHeader.fragmentIndex = fragmentIndex;
        fragHeader.totalFragments = totalFragments;
        fragHeader.totalSize = dataSize;
        
        // Если это последний фрагмент, увеличиваем messageId для следующего сообщения
        if (fragmentIndex == totalFragments - 1) {
            messageIdCounter++;
        }
        
        // Вычисляем размер данных для этого фрагмента
        size_t fragmentOffset = fragmentIndex * MAX_FRAGMENT_DATA;
        size_t fragmentDataSize = min((size_t)MAX_FRAGMENT_DATA, dataSize - fragmentOffset);
        
        // Создаем пакет
        packetBase.packetType = CMD_STRUCTURED_FRAGMENT;
        packetBase.payloadLen = StructuredFragmentHeader::size() + fragmentDataSize;
        
        if (packetBase.payloadLen > MAX_STRUCTURED_DATA_SIZE) {
            return false;
        }
        
        // Записываем заголовок фрагмента и данные
        fragHeader.serialize(payload);
        memcpy(payload + StructuredFragmentHeader::size(), data + fragmentOffset, fragmentDataSize);
        
        payloadSize = packetBase.payloadLen;
        return true;
    }
    
    // Вычислить количество фрагментов для данных заданного размера
    static size_t getFragmentCount(size_t dataSize) {
        return (dataSize + MAX_FRAGMENT_DATA - 1) / MAX_FRAGMENT_DATA;
    }
    
    // Разбор ультра-компактного heartbeat
    static bool parseHeartbeat(const uint8_t* payload, size_t payloadSize, UltraCompactHeartbeat& heartbeat) {
        if (payloadSize != UltraCompactHeartbeat::serializedSize()) {
            return false;
        }
        
        heartbeat.deserialize(payload);
        return true;
    }
    
    // Разбор GPS данных
    static bool parseGPS(const uint8_t* payload, size_t payloadSize, GPSStatus& gps) {
        if (payloadSize != GPSStatus::serializedSize()) {
            return false;
        }
        
        gps.deserialize(payload);
        return true;
    }
    
    // 🚀 NEW: Разбор данных моторов
    static bool parseMotors(const uint8_t* payload, size_t payloadSize, MotorStatus& motors) {
        if (payloadSize != MotorStatus::serializedSize()) {
            return false;
        }
        
        motors.deserialize(payload);
        return true;
    }
    
    // 🚀 NEW: Разбор данных датчиков
    static bool parseSensors(const uint8_t* payload, size_t payloadSize, SensorStatus& sensors) {
        if (payloadSize != SensorStatus::serializedSize()) {
            return false;
        }
        
        sensors.deserialize(payload);
        return true;
    }
    
    // 🚀 NEW: Разбор навигационных данных
    static bool parseNavigation(const uint8_t* payload, size_t payloadSize, NavigationStatus& navigation) {
        if (payloadSize != NavigationStatus::serializedSize()) {
            return false;
        }
        
        navigation.deserialize(payload);
        return true;
    }
    
    // Разбор фрагментированного пакета
    static bool parseFragment(const uint8_t* payload, size_t payloadSize, 
                              StructuredFragmentHeader& header, const uint8_t*& fragmentData, size_t& fragmentSize) {
        if (payloadSize < StructuredFragmentHeader::size()) {
            return false;
        }
        
        header.deserialize(payload);
        fragmentData = payload + StructuredFragmentHeader::size();
        fragmentSize = payloadSize - StructuredFragmentHeader::size();
        
        return true;
    }
    
    // Проверка, является ли CommandType структурированным пакетом
    static bool isStructuredPacket(CommandType type) {
        return type == CMD_STRUCTURED_HEARTBEAT || 
               type == CMD_STRUCTURED_GPS ||
               type == CMD_STRUCTURED_MOTORS ||
               type == CMD_STRUCTURED_SENSORS ||
               type == CMD_STRUCTURED_NAVIGATION ||
               type == CMD_STRUCTURED_FRAGMENT;
    }
    
    // Получить статистику размеров пакетов
    static String getStats() {
        return "UltraHB:" + String(UltraCompactHeartbeat::serializedSize()) + "b" +
               ", GPS:" + String(GPSStatus::serializedSize()) + "b" +
               ", Motor:" + String(MotorStatus::serializedSize()) + "b" + 
               ", Sensor:" + String(SensorStatus::serializedSize()) + "b" +
               ", MaxData:" + String(MAX_STRUCTURED_DATA_SIZE) + "b";
    }
};

// Статические переменные
uint16_t StructuredDataManager::messageIdCounter = 1;
