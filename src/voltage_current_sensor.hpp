
#ifndef VOLTAGE_CURRENT_SENSOR_HPP
#define VOLTAGE_CURRENT_SENSOR_HPP

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

class VoltageCurrentSensor {
public:
    VoltageCurrentSensor(uint8_t i2cAddress = 0x48) : ads(), _i2cAddress(i2cAddress) {}

    void begin() {
        Wire.begin();
        
        if (!ads.begin(_i2cAddress)) {
            // Можно добавить обработку ошибки
            Serial.print("Failed to initialize ADS1115 at address 0x");
            Serial.println(_i2cAddress, HEX);
            //while (true); // глушим, или сделай флаг
            return;
        }
        _initialized = true;
        ads.setGain(GAIN_ONE); // ±4.096 V вход
    }

    void read(float &voltage, float &current) {
        if (!_initialized) {
            Serial.println("Sensor not initialized. Call begin() first.");
            voltage = 0;
            current = 0;
            return;
        }
        int16_t rawV = ads.readADC_SingleEnded(2); // A2 = VT
        int16_t rawA = ads.readADC_SingleEnded(3); // A3 = AT

        voltage = (rawV * 4.096f / 32768.0f) * VOLTAGE_DIVIDER_RATIO;
        current = (rawA * 4.096f / 32768.0f) * SHUNT_CONVERSION;
    }

private:
    Adafruit_ADS1115 ads;
    uint8_t _i2cAddress;
    bool _initialized = false;
    static constexpr float VOLTAGE_DIVIDER_RATIO = 17.819f;                                                                       
    static constexpr float SHUNT_CONVERSION = 0.196f;
};

#endif // VOLTAGE_CURRENT_SENSOR_HPP
