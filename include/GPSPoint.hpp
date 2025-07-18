#pragma once
#include <Arduino.h>
#include <cstring>

// GPS точка с координатами и базовой информацией
struct GPSPoint {
    double latitude = 0.0;      // Широта в градусах
    double longitude = 0.0;     // Долгота в градусах
    float altitude = 0.0f;      // Высота над уровнем моря в метрах
    uint32_t timestamp = 0;     // Unix timestamp
    
    // Конструкторы
    GPSPoint() = default;
    
    GPSPoint(double lat, double lon, float alt = 0.0f, uint32_t ts = 0) 
        : latitude(lat), longitude(lon), altitude(alt), timestamp(ts) {}
    
    // Проверка валидности координат
    bool isValid() const {
        return (latitude != 0.0 || longitude != 0.0) && 
               latitude >= -90.0 && latitude <= 90.0 && 
               longitude >= -180.0 && longitude <= 180.0;
    }
    
    // Расстояние до другой точки в метрах (приближенное)
    float distanceTo(const GPSPoint& other) const {
        if (!isValid() || !other.isValid()) return -1.0f;
        
        const float R = 6371000.0f; // Радиус Земли в метрах
        float lat1_rad = latitude * PI / 180.0f;
        float lat2_rad = other.latitude * PI / 180.0f;
        float dlat_rad = (other.latitude - latitude) * PI / 180.0f;
        float dlon_rad = (other.longitude - longitude) * PI / 180.0f;
        
        float a = sin(dlat_rad/2) * sin(dlat_rad/2) +
                  cos(lat1_rad) * cos(lat2_rad) *
                  sin(dlon_rad/2) * sin(dlon_rad/2);
        float c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
    
    // Конвертация в строку для отладки
    String toString() const {
        return String(latitude, 6) + "," + String(longitude, 6) + 
               ",alt:" + String(altitude, 1) + "m";
    }
    
    // Сериализация для передачи по LoRa
    void serialize(uint8_t* buffer) const {
        memcpy(buffer, &latitude, sizeof(latitude));
        memcpy(buffer + sizeof(latitude), &longitude, sizeof(longitude));
        memcpy(buffer + sizeof(latitude) + sizeof(longitude), &altitude, sizeof(altitude));
        memcpy(buffer + sizeof(latitude) + sizeof(longitude) + sizeof(altitude), &timestamp, sizeof(timestamp));
    }
    
    // Десериализация
    void deserialize(const uint8_t* buffer) {
        memcpy(&latitude, buffer, sizeof(latitude));
        memcpy(&longitude, buffer + sizeof(latitude), sizeof(longitude));
        memcpy(&altitude, buffer + sizeof(latitude) + sizeof(longitude), sizeof(altitude));
        memcpy(&timestamp, buffer + sizeof(latitude) + sizeof(longitude) + sizeof(altitude), sizeof(timestamp));
    }
    
    // Размер для сериализации
    static constexpr size_t serializedSize() {
        return sizeof(latitude) + sizeof(longitude) + sizeof(altitude) + sizeof(timestamp);
    }
};

// GPS статус с информацией о состоянии GPS
struct GPSStatus {
    GPSPoint position;          // Текущая позиция
    bool hasFix = false;        // Есть ли GPS фикс
    uint8_t satelliteCount = 0; // Количество спутников
    float hdop = 99.9f;         // Horizontal Dilution of Precision
    float speed = 0.0f;         // Скорость в м/с
    float course = 0.0f;        // Курс в градусах (0-360)
    uint32_t lastUpdateTime = 0; // Время последнего обновления
    
    // Качество сигнала GPS
    enum FixQuality {
        NO_FIX = 0,
        GPS_FIX = 1,
        DGPS_FIX = 2,
        RTK_FIX = 3
    };
    
    FixQuality fixQuality = NO_FIX; // Текущее качество фикса
    
    // Конструкторы
    GPSStatus() = default;
    
    // Проверка актуальности данных GPS (не старше 5 секунд)
    bool isDataFresh() const {
        return (millis() - lastUpdateTime) < 5000;
    }
    
    FixQuality getFixQuality() const {
        return fixQuality; // Возвращаем сохраненное значение
    }
    
    // Установить качество фикса
    void setFixQuality(FixQuality quality) {
        fixQuality = quality;
    }
    
    // Автоматическое определение качества фикса на основе данных
    void updateFixQuality() {
        if (!hasFix) {
            fixQuality = NO_FIX;
        } else if (hdop < 1.0f && satelliteCount >= 8) {
            fixQuality = RTK_FIX;
        } else if (hdop < 2.0f && satelliteCount >= 6) {
            fixQuality = DGPS_FIX;
        } else {
            fixQuality = GPS_FIX;
        }
    }
    
    // Конвертация в строку для отладки
    String toString() const {
        String quality[] = {"NO_FIX", "GPS", "DGPS", "RTK"};
        return "GPS: " + position.toString() + 
               ", Fix:" + String(hasFix ? "YES" : "NO") + 
               ", Sats:" + String(satelliteCount) + 
               ", HDOP:" + String(hdop, 1) + 
               ", Quality:" + quality[getFixQuality()];
    }
    
    // Сериализация
    void serialize(uint8_t* buffer) const {
        size_t offset = 0;
        position.serialize(buffer + offset);
        offset += GPSPoint::serializedSize();
        
        buffer[offset++] = hasFix ? 1 : 0;
        buffer[offset++] = satelliteCount;
        buffer[offset++] = (uint8_t)fixQuality;  // Добавляем fixQuality
        memcpy(buffer + offset, &hdop, sizeof(hdop)); offset += sizeof(hdop);
        memcpy(buffer + offset, &speed, sizeof(speed)); offset += sizeof(speed);
        memcpy(buffer + offset, &course, sizeof(course)); offset += sizeof(course);
        memcpy(buffer + offset, &lastUpdateTime, sizeof(lastUpdateTime));
    }
    
    // Десериализация
    void deserialize(const uint8_t* buffer) {
        size_t offset = 0;
        position.deserialize(buffer + offset);
        offset += GPSPoint::serializedSize();
        
        hasFix = buffer[offset++] != 0;
        satelliteCount = buffer[offset++];
        fixQuality = (FixQuality)buffer[offset++];  // Добавляем fixQuality
        memcpy(&hdop, buffer + offset, sizeof(hdop)); offset += sizeof(hdop);
        memcpy(&speed, buffer + offset, sizeof(speed)); offset += sizeof(speed);
        memcpy(&course, buffer + offset, sizeof(course)); offset += sizeof(course);
        memcpy(&lastUpdateTime, buffer + offset, sizeof(lastUpdateTime));
    }
    
    // Размер для сериализации (добавляем 1 байт для fixQuality)
    static constexpr size_t serializedSize() {
        return GPSPoint::serializedSize() + 3 + sizeof(hdop) + sizeof(speed) + sizeof(course) + sizeof(lastUpdateTime);
    }
};
