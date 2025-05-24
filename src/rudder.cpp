#include <Arduino.h>
#include <ESP32Servo.h>
#include "rudder.h"
#include "settings.h"

int leftTrim = 0;
Servo rudder;
int lastRudderPwm = -1;
int sameAngleCounter = 0;
const int maxSameAngleCycles = 10;

int targetRudderPwm = -1;
int currentRudderPwm = -1;
unsigned long lastRudderStepTime = 0;
const int rudderStepInterval = 10;  // мс между шагами
const int rudderStepSize = 10;      // шаг в микросекундах

void updateRudder() {
  if (targetRudderPwm < 0 || currentRudderPwm < 0) return;

  if (millis() - lastRudderStepTime >= rudderStepInterval) {
    lastRudderStepTime = millis();

    if (currentRudderPwm != targetRudderPwm) {
      int delta = targetRudderPwm - currentRudderPwm;
      int step = (abs(delta) < rudderStepSize) ? delta : (delta > 0 ? rudderStepSize : -rudderStepSize);

      currentRudderPwm += step;
      static unsigned long lastDbg = 0;
      if (millis() - lastDbg > 900) {
        Serial.printf("[PWM]RUD: %d - Tar: %d\n",currentRudderPwm, targetRudderPwm);
        lastDbg = millis();
      }
      rudder.writeMicroseconds(currentRudderPwm);
    }
  }
}



void setRudderTrim(int leftDeg, int rightDeg) {
  leftTrim = constrain(leftDeg, -90, 90);
}

void setRudderAngle(int angle) {
  angle = -angle;  // Инвертируем, если серво установлено "вниз"

  angle = constrain(angle, -60, 60);
  int centerPulse = 1500;
  int angleOffset = map(angle, -90, 90, -500, 500);
  int trimOffset = map(leftTrim, -90, 90, -500, 500);
  int pwmValue = constrain(centerPulse + angleOffset + trimOffset, 500, 2500);

  targetRudderPwm = pwmValue;
  if (currentRudderPwm < 0) {
    // первый запуск — устанавливаем сразу
    currentRudderPwm = pwmValue;
    rudder.writeMicroseconds(pwmValue);
  }
}
