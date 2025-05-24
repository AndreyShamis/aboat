#include <Arduino.h>
#include "EncoderHandler.h"

// Пины для первого энкодера (S1 = CLK, S2 = DT, KEY = кнопка)
#define ENC_CLK 13
#define ENC_DT 16
#define ENC_SW 4

extern void setMotorPower(int percent);
extern void setRudderAngle(int angle);  // или setRudderFromEncoder



volatile int powerPercent = 0;   // 0-100
volatile int rudderAngle = 0;     // -100 to 100

int lastClk = HIGH;
bool lastButtonState = HIGH;
unsigned long lastDebounce = 0;

EncoderMode currentMode = MODE_POWER;  // ✅ настоящее определение только здесь

void setEncoderMode(EncoderMode mode) {
  currentMode = mode;
}

void initEncoder() {
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
}

void updateEncoder() {
  int clk = digitalRead(ENC_CLK);
  int dt = digitalRead(ENC_DT);
  int sw = digitalRead(ENC_SW);

  // Обработка кнопки с антидребезгом
  if (sw == LOW && lastButtonState == HIGH && millis() - lastDebounce > 200) {
    currentMode = (currentMode == MODE_POWER) ? MODE_RUDDER : MODE_POWER;
    lastDebounce = millis();
  }
  lastButtonState = sw;

  // Обработка поворота энкодера
  if (clk != lastClk && clk == LOW) {
    if (dt != clk) {
      // Вправо
      if (currentMode == MODE_POWER) {
        powerPercent = min(100, powerPercent + 1);
        setMotorPower(powerPercent);
      } else {
        rudderAngle = min(90, rudderAngle + 1);
        setRudderAngle(rudderAngle);
      }
    } else {
      // Влево
      if (currentMode == MODE_POWER) {
        powerPercent = max(0, powerPercent - 1);
        setMotorPower(powerPercent);
      } else {
        rudderAngle = max(-90, rudderAngle - 1);
        setRudderAngle(rudderAngle);
      }
    }

    // Выводим одно сообщение со всеми данными
    Serial.print("Mode: ");
    Serial.print(currentMode == MODE_POWER ? "POWER" : "RUDDER");
    Serial.print(" | Power: ");
    Serial.print(powerPercent);
    Serial.print("% | Rudder: ");
    Serial.println(rudderAngle);
  }

  lastClk = clk;
}
