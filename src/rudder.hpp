// rudder.hpp
#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
using namespace ArduinoJson;

class RudderController {
public:
    RudderController(uint8_t pin) : _pin(pin) {}
    
  void toJSON(ArduinoJson::JsonObject& json) const {
      json[F("target")] = getTargetAngle();
      json[F("current")] = getCurrentAngle();
      json[F("tPwm")] = _targetPwm;
      json[F("cPwm")] = _currentPwm;
  }
    // Вернуть целевой угол руля в градусах
    int getTargetAngle() const {
        return pwmToAngle(_targetPwm);
    }

    // Вернуть текущий угол руля в градусах
    int getCurrentAngle() const {
        return pwmToAngle(_currentPwm);
    }

    void begin() {
        servo.setPeriodHertz(50);
        servo.attach(_pin, 1000, 2000);
        Serial.println("✅ Rudder initialized");
    }

    void setTrim(int trim) {
        _leftTrim = constrain(trim, -30, 30);
    }

    void setAngle(int angle) {
        angle = -angle;
        angle = constrain(angle, -90, 90);
        int center = 1500;
        int offset = map(angle, -90, 90, -500, 500);
        int trimOffset = map(_leftTrim, -90, 90, -500, 500);
        int pwm = constrain(center + offset + trimOffset, 500, 2500);
        _targetPwm = pwm;
        if (_currentPwm < 0) {
            _currentPwm = pwm;
            writeMicroseconds(pwm);
        }
    }

    void update() {
        if (_targetPwm < 0 || _currentPwm < 0) return;
        if (millis() - _lastStepTime >= _stepInterval) {
            _lastStepTime = millis();
            if (_currentPwm != _targetPwm) {
                int delta = _targetPwm - _currentPwm;
                int step = (abs(delta) < _stepSize) ? delta : (delta > 0 ? _stepSize : -_stepSize);
                _currentPwm += step;
                if (_currentPwm != _lastPwm) {
                    writeMicroseconds(_currentPwm);
                    _lastPwm = _currentPwm;
                    //Serial.printf("[PWM] RUD: %d - Tar: %d\n", _currentPwm, _targetPwm);
                }
            }
        }
    }

private:
    uint8_t _pin;
    Servo servo;
    int _leftTrim = 0;
    int _lastPwm = -1;
    int _targetPwm = -1;
    int _currentPwm = -1;
    const int _stepSize = 20;
    const int _stepInterval = 2;
    unsigned long _lastStepTime = 0;

    void writeMicroseconds(int us) {
        us = constrain(us, 1000, 2000);
        servo.writeMicroseconds(us);
    }
        
    // Преобразует PWM-сигнал в угол (с учётом триммера)
    int pwmToAngle(int pwm) const {
        if (pwm < 1000 || pwm > 2500) return 0;  // safety
        int center = 1500 + map(_leftTrim, -90, 90, -500, 500);
        int delta = pwm - center;
        return constrain(map(delta, -500, 500, -90, 90), -90, 90);
    }
};
