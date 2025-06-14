#pragma once

#include "settings.h"
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>

class BatteryMonitor {
public:

    BatteryMonitor() {
        analogReadResolution(12);                // 0..4095
        //analogSetClockDiv(1);                    // максимальная скорость
        //analogSetAttenuation(ADC_11db);          // для всех пинов
        //analogSetPinAttenuation(VBAT_READ_PIN, ADC_11db);
    }

    void setup() {
        pinMode(ADC_CTRL_PIN, OUTPUT);
        digitalWrite(ADC_CTRL_PIN, LOW);         // питание по умолчанию отключено
        adcAttachPin(VBAT_READ_PIN);             // подключаем ADC к пину
    }

    void prepareForRead() {
        digitalWrite(ADC_CTRL_PIN, LOW);        // включаем питание
        delay(10);                                // ждём стабилизацию
    }

    void readVoltage() {
        rawValue = analogRead(VBAT_READ_PIN);
        millivolts = analogReadMilliVolts(VBAT_READ_PIN);
        voltageCache = millivolts * (490.0f / 100.0f) / 1000.0f;  // из mV в V с учётом делителя
        digitalWrite(ADC_CTRL_PIN, HIGH);         // отключаем питание
    }

    float getVoltage() const {
        return voltageCache;
    }

    int getRaw() const {
        return rawValue;
    }

    int getMillivolts() const {
        return millivolts;
    }

    void toJson(JsonObject& json) const {
        json["voltage"] = voltageCache;
        json["raw"] = rawValue;
        json["millivolts"] = millivolts;
    }

private:
    int rawValue = 0;
    int millivolts = 0;
    float voltageCache = 0.0f;
};
