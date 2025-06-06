#include <Arduino.h>
#include <ESP32Servo.h>
#include "settings.h"

int leftTrim = 0;
Servo rudder;
int lastRudderPwm = -1;
int sameAngleCounter = 0;
const int maxSameAngleCycles = 10;

int targetRudderPwm = -1;
int currentRudderPwm = -1;
unsigned long lastRudderStepTime = 0;
const int rudderStepInterval = 5;  // мс между шагами
const int rudderStepSize = 10;      // шаг в микросекундах


void setupRudderPwm() {
  rudder.setPeriodHertz(60);
  rudder.attach(RUDDER, 1000, 2000);  // pin, min, max pulse width in microseconds
  Serial.println("✅ RUDDER attached using ESP32Servo");
// bool ok = ledcSetup(RUDDER_PWM_CHANNEL, RUDDER_PWM_FREQ, RUDDER_PWM_RES);
// if (!ok) {
//   Serial.println("❌ ledcSetup failed for RUDDER");
// }
// ledcAttachPin(RUDDER, RUDDER_PWM_CHANNEL);

}

void writeRudderMicroseconds(int us) {
  // Переводим длительность импульса в значение duty для ledcWrite
  // int maxDuty = (1 << RUDDER_PWM_RES) - 1; // например, 2^16 - 1 = 65535
  // int duty = map(us, 1000, 2000, maxDuty * 5 / 20, maxDuty * 10 / 20);  // ~5%–10% от 20мс периода
  // duty = constrain(duty, 0, maxDuty);
  // ledcWrite(RUDDER_PWM_CHANNEL, duty);
  // Serial.printf("[PWM]set RUD: %d - Tar: %d\n",currentRudderPwm, duty);
  us = constrain(us, 1000, 2000);
  rudder.writeMicroseconds(us);
  //Serial.printf("[PWM]set RUD: %d μs\n", us);  
}



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
      if (currentRudderPwm != lastRudderPwm) {
      writeRudderMicroseconds(currentRudderPwm);
      Serial.printf("[PWM]RUD: %d - Tar: %d\n",currentRudderPwm, targetRudderPwm);
      }

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
    writeRudderMicroseconds(pwmValue);
    Serial.printf("[PWM]RUD: %d - Tar: %d\n",currentRudderPwm, targetRudderPwm);
  }
}
