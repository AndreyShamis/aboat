#pragma once

#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

enum EncoderMode {
  MODE_POWER,
  MODE_RUDDER
};
extern EncoderMode currentMode;
void setEncoderMode(EncoderMode mode);


void initEncoder();
void updateEncoder();

#endif
