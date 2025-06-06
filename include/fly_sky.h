#pragma once

#include <Arduino.h>
#include "settings.h"
void setupFlySky();
extern volatile uint16_t channels[10];
uint16_t getFlySkyChannel(uint8_t ch);
#ifdef USE_IBUS
#include <IBusBM.h>
#else
void IRAM_ATTR ppmInterrupt();
#endif


