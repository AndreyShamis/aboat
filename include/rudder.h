#pragma once

extern int leftTrim;
extern Servo rudder;

void setRudderTrim(int leftDeg, int rightDeg);
void setRudderAngle(int angle);
void updateRudder();