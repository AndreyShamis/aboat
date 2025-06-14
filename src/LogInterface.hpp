#pragma once
#include <Arduino.h>

class LogInterface {
public:
    virtual void addLog(const String& msg) = 0;
};
