#include "fly_sky.h"
#include "settings.h"

volatile uint16_t channels[10];

void IRAM_ATTR ppmInterrupt() {
  static uint32_t lastTime = 0;
  static uint8_t channelIndex = 0;

  uint32_t now = micros();
  uint32_t duration = now - lastTime;
  lastTime = now;

  if (duration > 5000) {
    channelIndex = 0;
  } else if (channelIndex < 10) {
    channels[channelIndex++] = duration;
  }
}

void setupFlySkyPPM() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

uint16_t getFlySkyChannel(uint8_t channelIndex) {
  if (channelIndex >= 10) return 1500;
  noInterrupts();
  uint16_t value = channels[channelIndex];
  interrupts();
  return constrain(value, 0, 2000);
}
