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

    void printStatus() const;
};
