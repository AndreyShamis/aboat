#include "fly_sky.h"
#include "settings.h"

volatile uint16_t channels[10];


#ifdef USE_IBUS

IBusBM ibus;
HardwareSerial FlySkySerial(1);

void setupFlySky() {
  pinMode(IBUS_CONTROL_PIN, OUTPUT);
  digitalWrite(IBUS_CONTROL_PIN, HIGH); 
  FlySkySerial.begin(115200, SERIAL_8N1, FLY_SKY_IBUS_RX_PIN, -1);
  ibus.begin(FlySkySerial);
}

uint16_t getFlySkyChannel(uint8_t ch) {
  return ibus.readChannel(ch); // 0..9 каналы, 1000–2000
}

#else

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

void setupFlySky() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

uint16_t getFlySkyChannel(uint8_t ch) {
  if (ch >= 10) return 1500;
  noInterrupts();
  uint16_t value = channels[ch];
  interrupts();
  return constrain(value, 0, 2000);
}

#endif
