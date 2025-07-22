#pragma once
#include <Arduino.h>
#include "GPSPoint.hpp"
#include <cstring>

// Состояние моторов лодки
struct MotorStatus {
    enum MotorState {
        MOTOR_STOP = 0,
        MOTOR_FORWARD = 1,
        MOTOR_REVERSE = 2
    };
    
    MotorState state = MOTOR_STOP;
    int16_t leftPower = 0;      // Мощность левого мотора (-100 до 100)
    int16_t rightPower = 0;     // Мощность правого мотора (-100 до 100)
    int16_t rudderAngle = 0;    // Угол руля (-90 до 90 градусов)
    uint8_t throttleLimit = 100; // Ограничение газа (0-100%)
    bool emergencyStop = false; // Состояние экстренной остановки
    
    // Сериализация
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        buffer[offset++] = (uint8_t)state;
        memcpy(buffer + offset, &leftPower, sizeof(leftPower)); offset += sizeof(leftPower);
        memcpy(buffer + offset, &rightPower, sizeof(rightPower)); offset += sizeof(rightPower);
        memcpy(buffer + offset, &rudderAngle, sizeof(rudderAngle)); offset += sizeof(rudderAngle);
        buffer[offset++] = throttleLimit;
        buffer[offset++] = emergencyStop ? 1 : 0;
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        state = (MotorState)buffer[offset++];
        memcpy(&leftPower, buffer + offset, sizeof(leftPower)); offset += sizeof(leftPower);
        memcpy(&rightPower, buffer + offset, sizeof(rightPower)); offset += sizeof(rightPower);
        memcpy(&rudderAngle, buffer + offset, sizeof(rudderAngle)); offset += sizeof(rudderAngle);
        throttleLimit = buffer[offset++];
        emergencyStop = buffer[offset++] != 0;
    }
    
    static constexpr size_t serializedSize() {
        return 1 + sizeof(leftPower) + sizeof(rightPower) + sizeof(rudderAngle) + 1 + 1;
    }
    
    String toString() const {
        String stateStr[] = {"STOP", "FWD", "REV"};
        return "Motor: " + stateStr[state] + 
               ", L:" + String(leftPower) + 
               ", R:" + String(rightPower) + 
               ", Rudder:" + String(rudderAngle) + "°" +
               ", Limit:" + String(throttleLimit) + "%";
    }
};

// Состояние LoRa радио
struct LoRaStatus {
    uint8_t currentProfile = 0;     // Текущий профиль (0-12)
    float rssi = -120.0f;          // RSSI в dBm
    float snr = 0.0f;              // SNR в dB
    bool adaptiveMode = true;       // Включен ли адаптивный режим
    bool missionControlConnected = false; // Подключен ли пульт
    uint32_t lastPacketTime = 0;   // Время последнего пакета
    uint16_t packetsReceived = 0;  // Счетчик принятых пакетов
    uint16_t packetsSent = 0;      // Счетчик отправленных пакетов
    
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        buffer[offset++] = currentProfile;
        memcpy(buffer + offset, &rssi, sizeof(rssi)); offset += sizeof(rssi);
        memcpy(buffer + offset, &snr, sizeof(snr)); offset += sizeof(snr);
        buffer[offset++] = adaptiveMode ? 1 : 0;
        buffer[offset++] = missionControlConnected ? 1 : 0;
        memcpy(buffer + offset, &lastPacketTime, sizeof(lastPacketTime)); offset += sizeof(lastPacketTime);
        memcpy(buffer + offset, &packetsReceived, sizeof(packetsReceived)); offset += sizeof(packetsReceived);
        memcpy(buffer + offset, &packetsSent, sizeof(packetsSent));
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        currentProfile = buffer[offset++];
        memcpy(&rssi, buffer + offset, sizeof(rssi)); offset += sizeof(rssi);
        memcpy(&snr, buffer + offset, sizeof(snr)); offset += sizeof(snr);
        adaptiveMode = buffer[offset++] != 0;
        missionControlConnected = buffer[offset++] != 0;
        memcpy(&lastPacketTime, buffer + offset, sizeof(lastPacketTime)); offset += sizeof(lastPacketTime);
        memcpy(&packetsReceived, buffer + offset, sizeof(packetsReceived)); offset += sizeof(packetsReceived);
        memcpy(&packetsSent, buffer + offset, sizeof(packetsSent));
    }
    
    static constexpr size_t serializedSize() {
        return 1 + sizeof(rssi) + sizeof(snr) + 1 + 1 + sizeof(lastPacketTime) + sizeof(packetsReceived) + sizeof(packetsSent);
    }
    
    String toString() const {
        return "LoRa: Profile:" + String(currentProfile) + 
               ", RSSI:" + String(rssi, 1) + "dBm" +
               ", SNR:" + String(snr, 1) + "dB" +
               ", Adaptive:" + String(adaptiveMode ? "ON" : "OFF") +
               ", MC:" + String(missionControlConnected ? "CONN" : "DISC");
    }
};

// Температуры и датчики
struct SensorStatus {
    float motor1Temp = 0.0f;       // Температура мотора 1 (°C)
    float motor2Temp = 0.0f;       // Температура мотора 2 (°C)
    float radiatorTemp = 0.0f;     // Температура радиатора (°C)
    float oilTemp = 0.0f;          // Температура масла (°C)
    float ambientTemp = 0.0f;      // Температура окружающей среды (°C)
    float batteryVoltage = 0.0f;   // Напряжение батареи (V)
    float batteryCurrent = 0.0f;   // Ток батареи (A)
    uint8_t batteryPercent = 0;    // Уровень заряда батареи (%)
    bool lowVoltageWarning = false; // Предупреждение низкого напряжения
    bool overtemperatureWarning = false; // Предупреждение перегрева
    
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        memcpy(buffer + offset, &motor1Temp, sizeof(motor1Temp)); offset += sizeof(motor1Temp);
        memcpy(buffer + offset, &motor2Temp, sizeof(motor2Temp)); offset += sizeof(motor2Temp);
        memcpy(buffer + offset, &radiatorTemp, sizeof(radiatorTemp)); offset += sizeof(radiatorTemp);
        memcpy(buffer + offset, &oilTemp, sizeof(oilTemp)); offset += sizeof(oilTemp);
        memcpy(buffer + offset, &ambientTemp, sizeof(ambientTemp)); offset += sizeof(ambientTemp);
        memcpy(buffer + offset, &batteryVoltage, sizeof(batteryVoltage)); offset += sizeof(batteryVoltage);
        memcpy(buffer + offset, &batteryCurrent, sizeof(batteryCurrent)); offset += sizeof(batteryCurrent);
        buffer[offset++] = batteryPercent;
        buffer[offset++] = lowVoltageWarning ? 1 : 0;
        buffer[offset++] = overtemperatureWarning ? 1 : 0;
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        memcpy(&motor1Temp, buffer + offset, sizeof(motor1Temp)); offset += sizeof(motor1Temp);
        memcpy(&motor2Temp, buffer + offset, sizeof(motor2Temp)); offset += sizeof(motor2Temp);
        memcpy(&radiatorTemp, buffer + offset, sizeof(radiatorTemp)); offset += sizeof(radiatorTemp);
        memcpy(&oilTemp, buffer + offset, sizeof(oilTemp)); offset += sizeof(oilTemp);
        memcpy(&ambientTemp, buffer + offset, sizeof(ambientTemp)); offset += sizeof(ambientTemp);
        memcpy(&batteryVoltage, buffer + offset, sizeof(batteryVoltage)); offset += sizeof(batteryVoltage);
        memcpy(&batteryCurrent, buffer + offset, sizeof(batteryCurrent)); offset += sizeof(batteryCurrent);
        batteryPercent = buffer[offset++];
        lowVoltageWarning = buffer[offset++] != 0;
        overtemperatureWarning = buffer[offset++] != 0;
    }
    
    static constexpr size_t serializedSize() {
        return sizeof(motor1Temp) + sizeof(motor2Temp) + sizeof(radiatorTemp) + 
               sizeof(oilTemp) + sizeof(ambientTemp) + sizeof(batteryVoltage) + 
               sizeof(batteryCurrent) + 1 + 1 + 1;
    }
    
    String toString() const {
        return "Sensors: M1:" + String(motor1Temp, 1) + "°C" +
               ", M2:" + String(motor2Temp, 1) + "°C" +
               ", Bat:" + String(batteryVoltage, 1) + "V/" + String(batteryPercent) + "%";
    }
};

// Навигационное состояние
struct NavigationStatus {
    enum NavigationMode {
        MANUAL = 0,
        WAYPOINT_FOLLOWING = 1,
        RETURN_TO_HOME = 2,
        STATION_KEEPING = 3
    };
    
    NavigationMode mode = MANUAL;
    GPSPoint homePosition;          // Домашняя позиция
    GPSPoint targetPosition;        // Целевая позиция
    uint8_t currentWaypoint = 0;    // Номер текущей точки маршрута
    uint8_t totalWaypoints = 0;     // Общее количество точек маршрута
    float distanceToTarget = 0.0f;  // Расстояние до цели (м)
    float bearingToTarget = 0.0f;   // Азимут до цели (градусы)
    bool navigationActive = false;   // Активна ли навигация
    
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        buffer[offset++] = (uint8_t)mode;
        homePosition.serialize(buffer + offset); offset += GPSPoint::serializedSize();
        targetPosition.serialize(buffer + offset); offset += GPSPoint::serializedSize();
        buffer[offset++] = currentWaypoint;
        buffer[offset++] = totalWaypoints;
        memcpy(buffer + offset, &distanceToTarget, sizeof(distanceToTarget)); offset += sizeof(distanceToTarget);
        memcpy(buffer + offset, &bearingToTarget, sizeof(bearingToTarget)); offset += sizeof(bearingToTarget);
        buffer[offset++] = navigationActive ? 1 : 0;
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        mode = (NavigationMode)buffer[offset++];
        homePosition.deserialize(buffer + offset); offset += GPSPoint::serializedSize();
        targetPosition.deserialize(buffer + offset); offset += GPSPoint::serializedSize();
        currentWaypoint = buffer[offset++];
        totalWaypoints = buffer[offset++];
        memcpy(&distanceToTarget, buffer + offset, sizeof(distanceToTarget)); offset += sizeof(distanceToTarget);
        memcpy(&bearingToTarget, buffer + offset, sizeof(bearingToTarget)); offset += sizeof(bearingToTarget);
        navigationActive = buffer[offset++] != 0;
    }
    
    static constexpr size_t serializedSize() {
        return 1 + (GPSPoint::serializedSize() * 2) + 1 + 1 + sizeof(distanceToTarget) + sizeof(bearingToTarget) + 1;
    }
    
    String toString() const {
        String modeStr[] = {"MANUAL", "WAYPOINT", "RTH", "STATION"};
        return "Nav: " + modeStr[mode] + 
               ", WP:" + String(currentWaypoint) + "/" + String(totalWaypoints) +
               ", Dist:" + String(distanceToTarget, 1) + "m";
    }
};

// Системная информация
struct SystemInfo {
    uint32_t uptime = 0;           // Время работы системы (секунды)
    uint32_t freeHeap = 0;         // Свободная память (байты)
    uint8_t systemHealth = 100;    // Общее состояние системы (0-100%)
    bool firmwareUpdateMode = false; // Режим обновления прошивки
    
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        memcpy(buffer + offset, &uptime, sizeof(uptime)); offset += sizeof(uptime);
        memcpy(buffer + offset, &freeHeap, sizeof(freeHeap)); offset += sizeof(freeHeap);
        buffer[offset++] = systemHealth;
        buffer[offset++] = firmwareUpdateMode ? 1 : 0;
    }
    
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        memcpy(&uptime, buffer + offset, sizeof(uptime)); offset += sizeof(uptime);
        memcpy(&freeHeap, buffer + offset, sizeof(freeHeap)); offset += sizeof(freeHeap);
        systemHealth = buffer[offset++];
        firmwareUpdateMode = buffer[offset++] != 0;
    }
    
    static constexpr size_t serializedSize() {
        return sizeof(uptime) + sizeof(freeHeap) + 1 + 1;
    }
    
    String toString() const {
        return "System: Uptime:" + String(uptime) + "s" +
               ", Free:" + String(freeHeap) + " bytes" +
               ", Health:" + String(systemHealth) + "%" +
               ", Update:" + String(firmwareUpdateMode ? "ON" : "OFF");
    }
};

// Основная структура состояния лодки
struct BoatSettings {
    uint32_t timestamp = 0;         // Время последнего обновления
    uint16_t version = 1;           // Версия структуры данных
    
    GPSStatus gps;                  // GPS статус и позиция
    MotorStatus motors;             // Состояние моторов
    LoRaStatus lora;               // Состояние LoRa
    SensorStatus sensors;           // Датчики и температуры
    NavigationStatus navigation;    // Навигационное состояние
    
    // Системная информация
    uint32_t uptime = 0;           // Время работы системы (секунды)
    uint32_t freeHeap = 0;         // Свободная память (байты)
    uint8_t systemHealth = 100;    // Общее состояние системы (0-100%)
    bool firmwareUpdateMode = false; // Режим обновления прошивки
    
    // Конструктор
    BoatSettings() {
        updateTimestamp();
    }
    
    // Обновить временную метку
    void updateTimestamp() {
        timestamp = millis();
    }
    
    // Проверка актуальности данных
    bool isDataFresh(uint32_t maxAge = 10000) const {
        return (millis() - timestamp) < maxAge;
    }
    
    // Полная сериализация (может потребоваться фрагментация для LoRa)
    size_t serialize(uint8_t* buffer) const {
        size_t offset = 0;
        
        memcpy(buffer + offset, &timestamp, sizeof(timestamp)); offset += sizeof(timestamp);
        memcpy(buffer + offset, &version, sizeof(version)); offset += sizeof(version);
        
        gps.serialize(buffer + offset); offset += GPSStatus::serializedSize();
        motors.serialize(buffer + offset); offset += MotorStatus::serializedSize();
        lora.serialize(buffer + offset); offset += LoRaStatus::serializedSize();
        sensors.serialize(buffer + offset); offset += SensorStatus::serializedSize();
        navigation.serialize(buffer + offset); offset += NavigationStatus::serializedSize();
        
        memcpy(buffer + offset, &uptime, sizeof(uptime)); offset += sizeof(uptime);
        memcpy(buffer + offset, &freeHeap, sizeof(freeHeap)); offset += sizeof(freeHeap);
        buffer[offset++] = systemHealth;
        buffer[offset++] = firmwareUpdateMode ? 1 : 0;
        
        return offset;
    }
    
    size_t deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        
        memcpy(&timestamp, buffer + offset, sizeof(timestamp)); offset += sizeof(timestamp);
        memcpy(&version, buffer + offset, sizeof(version)); offset += sizeof(version);
        
        gps.deserialize(buffer + offset); offset += GPSStatus::serializedSize();
        motors.deserialize(buffer + offset); offset += MotorStatus::serializedSize();
        lora.deserialize(buffer + offset); offset += LoRaStatus::serializedSize();
        sensors.deserialize(buffer + offset); offset += SensorStatus::serializedSize();
        navigation.deserialize(buffer + offset); offset += NavigationStatus::serializedSize();
        
        memcpy(&uptime, buffer + offset, sizeof(uptime)); offset += sizeof(uptime);
        memcpy(&freeHeap, buffer + offset, sizeof(freeHeap)); offset += sizeof(freeHeap);
        systemHealth = buffer[offset++];
        firmwareUpdateMode = buffer[offset++] != 0;
        
        return offset;
    }
    
    // Размер полной структуры
    static constexpr size_t serializedSize() {
        return sizeof(timestamp) + sizeof(version) +
               GPSStatus::serializedSize() + MotorStatus::serializedSize() +
               LoRaStatus::serializedSize() + SensorStatus::serializedSize() +
               NavigationStatus::serializedSize() +
               sizeof(uptime) + sizeof(freeHeap) + 1 + 1;
    }
    
    // Краткое описание состояния
    String getSummary() const {
        return "Boat: " + gps.toString() + " | " + 
               motors.toString() + " | " + 
               lora.toString() + " | Health:" + String(systemHealth) + "%";
    }
    
    // Проверка критических состояний
    bool hasCriticalAlerts() const {
        return sensors.lowVoltageWarning || 
               sensors.overtemperatureWarning || 
               motors.emergencyStop ||
               systemHealth < 50;
    }
};
