#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"

class OilPumpController {
public:
    void begin() {
        ledcSetup(OILPUMP_PWM_CHANNEL, OILPUMP_PWM_FREQ, OILPUMP_PWM_RES);
        ledcAttachPin(OIL_PUMP_PWM_PIN, OILPUMP_PWM_CHANNEL);
        stop(); // по умолчанию выключен
        enabled = true;
    }

    void setSpeed(uint8_t percent) {
        percent = constrain(percent, 0, 100);
        currentSpeed = map(percent, 0, 100, 0, 255);
        ledcWrite(OILPUMP_PWM_CHANNEL, currentSpeed);
        Serial.printf("🛢️ Oil pump speed set to %u%% (PWM: %d)\n", percent, currentSpeed);
    }

    void stop() {
        currentSpeed = 0;
        ledcWrite(OILPUMP_PWM_CHANNEL, 0);
        Serial.println("🛑 Oil pump stopped");
    }

    bool isEnabled() const {
        return enabled;
    }

    int getSpeedPercent() const {
        return map(currentSpeed, 0, 255, 0, 100);
    }

    void toJSON(JsonObject& json) const {
        json["enabled"] = enabled;
        json["speed_percent"] = getSpeedPercent();
    }

private:
    bool enabled = false;
    int currentSpeed = 0;  // 0..255
};
