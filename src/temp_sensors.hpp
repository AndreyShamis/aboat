#pragma once
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "settings.h"

#define MAX_SENSORS 8

enum SensorRole {
  MOTOR1,
  MOTOR2,
  MOTOR_RAD,
  OIL,
  ENV,
  SENSOR_ROLE_COUNT
};

static OneWire oneWire(DALLAS_ONEWIRE_PIN);
static DallasTemperature sensors(&oneWire);
static DeviceAddress sensorAddresses[MAX_SENSORS];
static DeviceAddress roleAddresses[SENSOR_ROLE_COUNT] = {0};
static float latestTemperatures[SENSOR_ROLE_COUNT] = {-127.0};

static int sensorCount = 0;
static TaskHandle_t tempTaskHandle = nullptr;

inline float getTemp(SensorRole role) {
  if (role < SENSOR_ROLE_COUNT) {
    return latestTemperatures[role];
  }
  return -127.0;
}

inline void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

inline void temperatureTask(void *param) {
  while (true) {
    sensors.requestTemperatures();
    for (int i = 0; i < SENSOR_ROLE_COUNT; i++) {
      latestTemperatures[i] = sensors.getTempC(roleAddresses[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

inline void setupTempSensors() {
  sensors.begin();
  sensorCount = sensors.getDeviceCount();
  Serial.printf("Found %d DS18B20 sensors\n", sensorCount);

  for (int i = 0; i < sensorCount && i < MAX_SENSORS; i++) {
    if (sensors.getAddress(sensorAddresses[i], i)) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" address: ");
      printAddress(sensorAddresses[i]);
      Serial.println();

      if (i < SENSOR_ROLE_COUNT) {
        memcpy(roleAddresses[i], sensorAddresses[i], 8);
      }
    }
  }

  xTaskCreatePinnedToCore(
    temperatureTask,
    "TempTask",
    4096,
    NULL,
    1,
    &tempTaskHandle,
    0
  );
}

inline void updateTemperatures() {
  sensors.requestTemperatures();
  for (int i = 0; i < SENSOR_ROLE_COUNT; i++) {
    latestTemperatures[i] = sensors.getTempC(roleAddresses[i]);
  }
}

inline void updateTempsOneByOne() {
  static uint8_t currentSensor = 0;
  static unsigned long lastRequest = 0;

  if (millis() - lastRequest > 1000) {
    sensors.requestTemperaturesByAddress(roleAddresses[currentSensor]);
    latestTemperatures[currentSensor] = sensors.getTempC(roleAddresses[currentSensor]);
    currentSensor = (currentSensor + 1) % SENSOR_ROLE_COUNT;
    lastRequest = millis();
  }
}
