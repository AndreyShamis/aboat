#pragma once
#include <Arduino.h>
#include "battery_monitor.hpp"
#include "voltage_current_sensor.hpp"
#include <MPU9250_asukiaaa.h>
#include "fly_sky.hpp"
#include "system_status.hpp"
#include "temp_sensors.hpp"
#include "oil_pump.hpp"


class Boat
{
public:

    MPU9250_asukiaaa mpu;
    FusionAhrs ahrs;
    BatteryMonitor battery;
    VoltageCurrentSensor sensor;
    TinyGPSPlus gps;
    HardwareSerial GPSserial;


    float motor1Temp = -127.0;
    float motor2Temp = -127.0;
    float radiatorTemp = -127.0;
    float oilTemp = -127.0;
    float envTemp = -127.0;
    Boat() : sensor(0x48), GPSserial(Serial2) {}
    void updateMotor1Temp(float t) { motor1Temp = t; }
    void updateMotor2Temp(float t) { motor2Temp = t; }
    void updateRadiatorTemp(float t) { radiatorTemp = t; }
    void updateOilTemp(float t) { oilTemp = t; }
    void updateEnvTemp(float t) { envTemp = t; }

    void setup()
    {
        GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // GPS UART

        SystemStatus::printResetReason();
        SystemStatus::printWakeupReason();
        SystemStatus::printUptime();
        sensor.begin();
        Serial.println("Strarting setup iBUS FlySky...");
        setupFlySky();

        Serial.println("Starting I2C bus scan...");
        Wire.begin(I2C_SDA, I2C_SCL);
        Serial.println("Scanning I2C bus...");

        for (uint8_t address = 1; address < 127; ++address)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                Serial.print("Found I2C device at 0x");
                Serial.println(address, HEX);
            }
        }

        mpu.setWire(&Wire);
        mpu.beginAccel();
        mpu.beginGyro();
        mpu.beginMag();
        FusionAhrsInitialise(&ahrs);

        Serial.println("MPU9250 + Madgwick initialized");

        setupTempSensors();
        Serial.println("Dallas sensors initialized");
        setupOilPump();
        Serial.println("Oil pump initialized");
    }

    void printStatus()
    {
        float v = 0, a = 0;
        sensor.read(v, a);
        Serial.printf("V: %.2f V\tI: %.3f A\n", v, a);
        Serial.println("=== Boat Status ===");
        Serial.printf("Motor1:    %.2f °C\n", motor1Temp);
        Serial.printf("Motor2:    %.2f °C\n", motor2Temp);
        Serial.printf("Radiator:  %.2f °C\n", radiatorTemp);
        Serial.printf("Oil:       %.2f °C\n", oilTemp);
        Serial.printf("Ambient:   %.2f °C\n", envTemp);
        Serial.printf("Voltage:   %.2f V\n", battery.readVoltage());
        Serial.println("====================");
    }


    void keep(){
        updateOilPumpLogic(motor1Temp, motor2Temp, radiatorTemp);
    }
};
