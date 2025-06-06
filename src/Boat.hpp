#pragma once
#include <Arduino.h>

class Boat {
public:
    float motor1Temp = -127.0;
    float motor2Temp = -127.0;
    float radiatorTemp = -127.0;
    float oilTemp = -127.0;
    float envTemp = -127.0;

    void updateMotor1Temp(float t) { motor1Temp = t; }
    void updateMotor2Temp(float t) { motor2Temp = t; }
    void updateRadiatorTemp(float t) { radiatorTemp = t; }
    void updateOilTemp(float t) { oilTemp = t; }
    void updateEnvTemp(float t) { envTemp = t; }

    void printStatus() const {
        Serial.println("=== Boat Status ===");
        Serial.printf("Motor1:    %.2f °C\n", motor1Temp);
        Serial.printf("Motor2:    %.2f °C\n", motor2Temp);
        Serial.printf("Radiator:  %.2f °C\n", radiatorTemp);
        Serial.printf("Oil:       %.2f °C\n", oilTemp);
        Serial.printf("Ambient:   %.2f °C\n", envTemp);
        Serial.println("====================");
    }

};
