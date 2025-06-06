#ifndef BATTERY_MONITOR_HPP
#define BATTERY_MONITOR_HPP

#include <Arduino.h>

class BatteryMonitor {
public:
    // GPIO1 = ADC1_CH0 = VBAT_Read на Heltec ESP32-S3 Stick Lite V3
    static constexpr int BATTERY_ADC_PIN = 1;  
    static constexpr float VREF = 3.3; // опорное напряжение
    static constexpr float DIVIDER_RATIO = 4.9; // 390k + 100k

    BatteryMonitor() {
        analogReadResolution(12);
    }

    float readVoltage() const {
        int raw = analogRead(BATTERY_ADC_PIN);
        return ((float)raw / 4095.0f) * VREF * DIVIDER_RATIO;
    }
};

#endif // BATTERY_MONITOR_HPP
