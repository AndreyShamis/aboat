#pragma once
// motor.hpp
#include <Arduino.h>
#include <ESP32Servo.h>
#include "settings.h"
#include <ArduinoJson.h>

#define motorStepInterval  5  // ms
#define motorStepSize  2

#define MIN_BIDIRECTIONAL 1500

enum ESCType {
    ESC_UNIDIRECTIONAL,
    ESC_BIDIRECTIONAL
};
struct MotorConfig {
    ESCType escType;
};

class BoatMotor {
public:


    ESCType escType;
    void begin(uint8_t pin) {
        _servo.setPeriodHertz(50);
        _servo.attach(pin, 1000, 2000);
        ESCType escType = ESCType::ESC_BIDIRECTIONAL; // Default to unidirectional
    }

    void setTargetPwm(int pwm) {
        _targetPwm = constrain(pwm, 1000, 2000);
        if (_currentPwm < 0) {
            _currentPwm = _targetPwm;
            _servo.writeMicroseconds(_currentPwm);
        }
    }

    int getTargetPwm() const { return _targetPwm; }
    int getCurrentPwm() const { return _currentPwm; }

    void update() {
        if (_targetPwm < 0 || _currentPwm < 0) return;

        if (millis() - _lastStep >= motorStepInterval) {
            _lastStep = millis();
            if (_currentPwm != _targetPwm) {
                if (_targetPwm < _currentPwm) {
                    _currentPwm = _targetPwm;
                } else {
                    int delta = _targetPwm - _currentPwm;
                    int step = (abs(delta) < motorStepSize) ? delta : motorStepSize;
                    _currentPwm += step;
                }
                _servo.writeMicroseconds(_currentPwm);
            }
        }
    }
    void toJSON(JsonObject& json) const {
        json["target"] = _targetPwm;
        json["current"] = _currentPwm;
        json["esc_type"] = (escType == ESC_UNIDIRECTIONAL) ? "uni" : "bi";
    }
private:
    Servo _servo;
    int _targetPwm = 1000;
    int _currentPwm = -1;
    unsigned long _lastStep = 0;
};



// // enum MotorState {
// //   MOTOR_STOP,
// //   MOTOR_FORWARD,
// //   MOTOR_REVERSE
// // };

// // ===== Gesture detection thresholds =====
// const int LOW_ZONE = 1090;
// const int HIGH_ZONE = 1910;

// const int PWM_BROKER = 1010;


// MotorState motorState;
// Servo escLeft;
// Servo escRight;
// int targetLeftPwm = PWM_MIN;
// int targetRightPwm = PWM_MIN;
// int currentLeftPwm = PWM_MIN;
// int currentRightPwm = PWM_MIN;
// int pwmLimit = PWM_MAX;  // Это заменит PWM_MAX в расчётах

// const int motorStepInterval = 5;  // ms
// const int motorStepSize = 2;      //
// unsigned long stateHoldStart = 0;
// bool comboDetected = false;
// void updatePwmLimit(uint16_t ch5) {
//   if (ch5 < 1250) {
//     pwmLimit = PWM_MIN + 0.20 * (PWM_MAX - PWM_MIN);  // 20%
//   } else if (ch5 < 1750) {
//     pwmLimit = PWM_MIN + 0.50 * (PWM_MAX - PWM_MIN);  // 50%
//   } else {
//     pwmLimit = PWM_MIN + 0.80 * (PWM_MAX - PWM_MIN);  // 80%
//   }
// }

// // === Utility: map with deadzone ===
// int mapWithDeadzone(int rawPwm, int broker) {
//   if (rawPwm <= 1010) return PWM_MIN;

//   // мапим [1021–2000] в [broker–PWM_MAX]
//   return map(rawPwm, 1011, 2000, broker, PWM_MAX);
// }

// // === Handle gesture-based state change ===
// void checkMotorStateChange(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
//   bool gestureToForward = ch1 < LOW_ZONE && ch2 > HIGH_ZONE && ch3 < LOW_ZONE && ch4 > HIGH_ZONE;
//   bool gestureToStop = ch1 < LOW_ZONE && ch2 < LOW_ZONE && ch3 < LOW_ZONE && ch4 > HIGH_ZONE;

//   if (motorState == MOTOR_STOP && gestureToForward) {
//     if (!comboDetected) {
//       comboDetected = true;
//       stateHoldStart = millis();
//     } else if (millis() - stateHoldStart >= 3000) {
//       motorState = MOTOR_FORWARD;
//       Serial.println("➡️ Motor State: FORWARD");
//       comboDetected = false;
//     }
//   } else if (motorState == MOTOR_FORWARD && gestureToForward) {
//     if (!comboDetected) {
//       comboDetected = true;
//       stateHoldStart = millis();
//     } else if (millis() - stateHoldStart >= 3000) {
//       motorState = MOTOR_STOP;
//       Serial.println("⏹ Motor State: STOP");
//       comboDetected = false;
//     }
//   } else if (!gestureToForward && !gestureToStop) {
//     comboDetected = false;
//   }
// }
// const char* motorStateToString(MotorState state) {
//   switch (state) {
//     case MOTOR_STOP: return "STOP";
//     case MOTOR_FORWARD: return "FORWARD";
//     case MOTOR_REVERSE: return "REVERSE";
//     default: return "UNKNOWN";
//   }
// }
// void updateMotorPWM() {
//   static unsigned long lastStep = 0;
//   if (millis() - lastStep >= motorStepInterval) {
//     lastStep = millis();

//     // === Left motor ===
//     if (currentLeftPwm != targetLeftPwm) {
//       if (targetLeftPwm < currentLeftPwm) {
//         currentLeftPwm = targetLeftPwm;  // 💥 Мгновенное снижение
//       } else {
//         int delta = targetLeftPwm - currentLeftPwm;
//         int step = (abs(delta) < motorStepSize) ? delta : motorStepSize;
//         currentLeftPwm += step;
//       }
//       escLeft.writeMicroseconds(currentLeftPwm);
//     }

//     // === Right motor ===
//     if (currentRightPwm != targetRightPwm) {
//       if (targetRightPwm < currentRightPwm) {
//         currentRightPwm = targetRightPwm;  // 💥 Мгновенное снижение
//       } else {
//         int delta = targetRightPwm - currentRightPwm;
//         int step = (abs(delta) < motorStepSize) ? delta : motorStepSize;
//         currentRightPwm += step;
//       }
//       escRight.writeMicroseconds(currentRightPwm);
//     }
//   }
//   static unsigned long lastDbg = 0;
//   if (millis() - lastDbg > 900) {
//     Serial.printf("[PWM] %s L: %d → %d | R: %d → %d --- LIMIT: %d\n", motorStateToString(motorState), currentLeftPwm, targetLeftPwm, currentRightPwm, targetRightPwm, pwmLimit);
//     lastDbg = millis();
//   }
// }

// // === Called from main loop ===
// void applyMotorControl(int throttleInput, int steerInput) {
//   switch (motorState) {
//     case MOTOR_STOP:
//       targetLeftPwm = PWM_MIN;
//       targetRightPwm = PWM_MIN;
//       break;

//     case MOTOR_FORWARD:
//       {
//         int basePwm = mapWithDeadzone(throttleInput, PWM_BROKER);

//         if (throttleInput > 1010) {
//           // Учитываем delta только при "газе"
//           int basePwmDelta = map(steerInput, 1000, 2000, -200, 200);
//           targetLeftPwm = constrain(basePwm + basePwmDelta, PWM_MIN, pwmLimit);
//           targetRightPwm = constrain(basePwm - basePwmDelta, PWM_MIN, pwmLimit);

//         } else {
//           // Равномерное значение без учёта поворота
//           targetLeftPwm = constrain(basePwm, PWM_MIN, pwmLimit);
//           targetRightPwm = constrain(basePwm, PWM_MIN, pwmLimit);
//         }
//         break;
//       }
//     case MOTOR_REVERSE:
//       targetLeftPwm = PWM_MIN;
//       targetRightPwm = PWM_MIN;
//       break;
//   }

//   updateMotorPWM();
// }
