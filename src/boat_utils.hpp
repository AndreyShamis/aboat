#pragma once
#include <Arduino.h>
#include <time.h>

// Вспомогательные утилиты для оптимизации лодки
namespace BoatUtils {
    
    // Интерполяция для плавных переходов
    template<typename T>
    T lerp(T a, T b, float t) {
        return a + t * (b - a);
    }
    
    // Сглаживание значений
    class ExponentialSmoother {
    private:
        float alpha;
        float lastValue;
        bool initialized;
        
    public:
        ExponentialSmoother(float smoothing = 0.1f) : alpha(smoothing), initialized(false) {}
        
        float update(float newValue) {
            if (!initialized) {
                lastValue = newValue;
                initialized = true;
            } else {
                lastValue = alpha * newValue + (1.0f - alpha) * lastValue;
            }
            return lastValue;
        }
        
        float getValue() const { return lastValue; }
        void reset() { initialized = false; }
    };
    
    // Ограничение значений
    template<typename T>
    T constrain_range(T value, T min_val, T max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
    
    // Конвертация углов
    float degreesToRadians(float degrees) {
        return degrees * PI / 180.0f;
    }
    
    float radiansToDegrees(float radians) {
        return radians * 180.0f / PI;
    }
    
    // Нормализация угла в диапазон [-180, 180]
    float normalizeAngle(float angle) {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }
    
    // Проверка валидности значений датчиков
    bool isValidTemperature(float temp) {
        return temp > -50.0f && temp < 150.0f && !isnan(temp);
    }
    
    bool isValidVoltage(float voltage) {
        return voltage > 5.0f && voltage < 30.0f && !isnan(voltage);
    }
    
    bool isValidRSSI(float rssi) {
        return rssi >= -150.0f && rssi <= 0.0f && !isnan(rssi);
    }
    
    // Форматирование строк для отладки
    String formatFloat(float value, int decimals = 2) {
        if (isnan(value)) return "NaN";
        return String(value, decimals);
    }
    
    String formatTemperature(float temp) {
        if (!isValidTemperature(temp)) return "INVALID";
        return formatFloat(temp, 1) + "°C";
    }
    
    String formatVoltage(float voltage) {
        if (!isValidVoltage(voltage)) return "INVALID";
        return formatFloat(voltage, 2) + "V";
    }
    
    // Расчет времени работы
    String formatUptime(unsigned long millis) {
        unsigned long seconds = millis / 1000;
        unsigned long minutes = seconds / 60;
        unsigned long hours = minutes / 60;
        unsigned long days = hours / 24;
        
        String result = "";
        if (days > 0) result += String(days) + "d ";
        if (hours % 24 > 0) result += String(hours % 24) + "h ";
        if (minutes % 60 > 0) result += String(minutes % 60) + "m ";
        result += String(seconds % 60) + "s";
        
        return result;
    }
    
    // Форматирование времени с миллисекундами
    String formatTimeWithMillis(unsigned long currentMillis = 0) {
        if (currentMillis == 0) {
            currentMillis = millis();
        }
        
        time_t now = time(nullptr);
        uint16_t milliseconds = currentMillis % 1000;
        
        if (now > 1600000000) { // Если системное время синхронизировано
            struct tm *t = localtime(&now);
            char buf[9];
            strftime(buf, sizeof(buf), "%H:%M:%S", t);
            char fullBuf[13];
            snprintf(fullBuf, sizeof(fullBuf), "%s.%03d", buf, milliseconds);
            return String(fullBuf);
        } else {
            // Если время не синхронизировано, используем uptime
            unsigned long totalSeconds = currentMillis / 1000;
            uint8_t hours = (totalSeconds / 3600) % 24;
            uint8_t minutes = (totalSeconds / 60) % 60;
            uint8_t seconds = totalSeconds % 60;
            
            char buf[13];
            snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
            return String(buf);
        }
    }
    
    // Создание timestamp для логирования
    String createTimestamp() {
        return formatTimeWithMillis();
    }
    
    // Расчет расстояния между GPS координатами (в метрах)
    float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
        const float R = 6371000.0f; // Радиус Земли в метрах
        float dLat = degreesToRadians(lat2 - lat1);
        float dLon = degreesToRadians(lon2 - lon1);
        
        float a = sin(dLat/2) * sin(dLat/2) +
                  cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) *
                  sin(dLon/2) * sin(dLon/2);
        float c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }
    
    // Расчет пеленга между GPS координатами (в градусах)
    float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
        float dLon = degreesToRadians(lon2 - lon1);
        float lat1_rad = degreesToRadians(lat1);
        float lat2_rad = degreesToRadians(lat2);
        
        float y = sin(dLon) * cos(lat2_rad);
        float x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon);
        
        float bearing = atan2(y, x);
        return normalizeAngle(radiansToDegrees(bearing));
    }
}
