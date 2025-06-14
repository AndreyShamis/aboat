#pragma once
#include <Arduino.h>
#include "motor.hpp"

// Constants shared across the system
constexpr int LOW_ZONE = 1090;
constexpr int HIGH_ZONE = 1910;
constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 1800;
constexpr int PWM_BROKER = 1010;

class MotorEngineControl
{
public:
    enum MotorState
    {
        MOTOR_STOP,
        MOTOR_FORWARD,
        MOTOR_REVERSE
    };

    void begin(uint8_t leftPin, uint8_t rightPin)
    {
        left.begin(leftPin);
        right.begin(rightPin);
    }

    void setState(MotorState state)
    {
        motorState = state;
    }

    void setLimit(int limit)
    {
        pwmLimit = constrain(limit, PWM_MIN, PWM_MAX);
    }

    int convertThrottleToPWM(BoatMotor &motor, int inputThrottle, ESCType escType) {
        inputThrottle = constrain(inputThrottle, 1000, 2000);

        if (motor.escType ==  ESCType::ESC_UNIDIRECTIONAL) {
            // Простой ESC: просто передаём как есть
            return inputThrottle;
        }
        if(motor.escType == ESCType::ESC_BIDIRECTIONAL) {
            switch (motorState)
            {
                case MOTOR_STOP:
                    return 1500;
                case MOTOR_FORWARD:
                    return map(inputThrottle, 1000, 2000, 1500, 2000);
                case MOTOR_REVERSE:
                    return map(inputThrottle, 1000, 2000, 1500, 1000);
            }
        }
        return inputThrottle; // По умолчанию, если ESC не определён
    }

    void apply(int throttle, int steer)
    {
        static unsigned long lastDbg = 0;

        switch (motorState)
        {
        case MOTOR_STOP:
            left.setTargetPwm(PWM_MIN);
            right.setTargetPwm(PWM_MIN);
            break;

        case MOTOR_FORWARD:
        {

            int base = mapWithDeadzone(throttle);
            int delta = map(steer, 1000, 2000, -200, 200);
            int lV = constrain(base + delta, PWM_MIN, pwmLimit);
            int rV = constrain(base - delta, PWM_MIN, pwmLimit);
            left.setTargetPwm(convertThrottleToPWM(left, lV, left.escType));
            right.setTargetPwm(convertThrottleToPWM(right, rV, right.escType));
            //right.setTargetPwm(constrain(base - delta, PWM_MIN, pwmLimit));
            break;
        }

        case MOTOR_REVERSE:
            int base = mapWithDeadzone(throttle);
            int delta = map(steer, 1000, 2000, -200, 200);
            int lV = constrain(base + delta, PWM_MIN, pwmLimit);
            int rV = constrain(base - delta, PWM_MIN, pwmLimit);
            left.setTargetPwm(convertThrottleToPWM(left, lV, left.escType));
            right.setTargetPwm(convertThrottleToPWM(right, rV, right.escType));
            break;
        }
        if (millis() - lastDbg > 3900) {
            //Serial.printf("[PWM] %s [L: %d→%d] | R: [%d→%d] LIMIT: %d\n", getMotorStateStr(), left.getCurrentPwm(), left.getTargetPwm(), right.getCurrentPwm(), right.getTargetPwm(), pwmLimit);
            lastDbg = millis();
        }
        left.update();
        right.update();
    }

    MotorState getState() const
    {
        return motorState;
    }

    void toJSON(JsonObject &json) const
    {
        json["state"] = getMotorStateStr();
        JsonObject leftJson = json["left"].to<JsonObject>();
        left.toJSON(leftJson);
        JsonObject rightJson = json["right"].to<JsonObject>();
        right.toJSON(rightJson);
    }

    const char *getMotorStateStr() const
    {
        switch (motorState)
        {
        case MOTOR_STOP:
            return "STOP";
        case MOTOR_FORWARD:
            return "FORWARD";
        case MOTOR_REVERSE:
            return "REVERSE";
        default:
            return "UNKNOWN";
        }
    }

private:
    BoatMotor left, right;
    MotorState motorState = MOTOR_STOP;
    int pwmLimit = PWM_MAX;

    int mapWithDeadzone(int val)
    {
        if (val <= PWM_BROKER)
            return PWM_MIN;
        return map(val, PWM_BROKER + 1, 2000, PWM_BROKER, PWM_MAX);
    }
};
