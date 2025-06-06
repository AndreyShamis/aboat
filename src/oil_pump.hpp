#pragma once
#include <Arduino.h>
#include "Boat.hpp"
#include "settings.h"

extern Boat boat;

bool isOilPumpEnabled = false;

void setupOilPump() {
  ledcSetup(OILPUMP_PWM_CHANNEL, OILPUMP_PWM_FREQ, OILPUMP_PWM_RES);
  ledcAttachPin(OIL_PUMP_PWM_PIN, OILPUMP_PWM_CHANNEL);
  ledcWrite(OILPUMP_PWM_CHANNEL, 0);
  isOilPumpEnabled = true;
}




void updateOilPumpLogic() {
  if (isOilPumpEnabled) {
    float tMotorAvg = (boat.motor1Temp + boat.motor2Temp) / 2.0;
    float tRad = boat.radiatorTemp;

    int speed = 0;

    if (tMotorAvg > 60.0) {
      speed = 0;
      Serial.println("Oil pump OFF: motor overheat");
    } else if (tMotorAvg >= 40.0) {
      speed = map((int)tMotorAvg, 40, 60, 100, 255);
      if (tRad > 50.0) {
        speed *= 0.7;
        Serial.println("Oil pump speed reduced by radiator temp");
      }
      Serial.printf("Oil pump speed set to %d\n", speed);
    } else {
      Serial.println("Oil pump OFF: motor cool");
    }

    ledcWrite(OILPUMP_PWM_CHANNEL, speed);
  }

}
