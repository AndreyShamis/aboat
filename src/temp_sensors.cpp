#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Boat.hpp"
#include "settings.h"

extern Boat boat;

#define MAX_SENSORS 8

OneWire oneWire(DALLAS_ONEWIRE_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddresses[MAX_SENSORS];
int sensorCount = 0;

enum SensorRole {
  MOTOR1,
  MOTOR2,
  MOTOR_RAD,
  OIL,
  ENV,
  SENSOR_ROLE_COUNT
};

DeviceAddress roleAddresses[SENSOR_ROLE_COUNT] = {0};

TaskHandle_t tempTaskHandle;

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void temperatureTask(void *param) {
  while (true) {
    sensors.requestTemperatures();

    boat.updateMotor1Temp(sensors.getTempC(roleAddresses[MOTOR1]));
    boat.updateMotor2Temp(sensors.getTempC(roleAddresses[MOTOR2]));
    boat.updateRadiatorTemp(sensors.getTempC(roleAddresses[MOTOR_RAD]));
    boat.updateOilTemp(sensors.getTempC(roleAddresses[OIL]));
    boat.updateEnvTemp(sensors.getTempC(roleAddresses[ENV]));

    vTaskDelay(pdMS_TO_TICKS(1000));  // Ждём 1 сек между опросами
  }
}


void setupTempSensors() {
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

  // Создаём задачу на другом ядре (0)
  xTaskCreatePinnedToCore(
    temperatureTask,     // Функция задачи
    "TempTask",          // Имя задачи
    4096,                // Размер стека
    NULL,                // Параметры
    1,                   // Приоритет
    &tempTaskHandle,     // Хэндл
    0                    // Ядро 0 (Arduino по дефолту — ядро 1)
  );  
}


void updateTemperatures() {
  sensors.requestTemperatures();
  boat.updateMotor1Temp(sensors.getTempC(roleAddresses[MOTOR1]));
  boat.updateMotor2Temp(sensors.getTempC(roleAddresses[MOTOR2]));
  boat.updateRadiatorTemp(sensors.getTempC(roleAddresses[MOTOR_RAD]));
  boat.updateOilTemp(sensors.getTempC(roleAddresses[OIL]));
  boat.updateEnvTemp(sensors.getTempC(roleAddresses[ENV]));
}


static uint8_t currentSensor = 0;
static unsigned long lastRequest = 0;

void updateTempsOneByOne() {
  if (millis() - lastRequest > 1000) {  // один датчик в секунду
    sensors.requestTemperaturesByAddress(roleAddresses[currentSensor]);
    float t = sensors.getTempC(roleAddresses[currentSensor]);

    switch (currentSensor) {
      case MOTOR1: boat.updateMotor1Temp(t); break;
      case MOTOR2: boat.updateMotor2Temp(t); break;
      case MOTOR_RAD: boat.updateRadiatorTemp(t); break;
      case OIL: boat.updateOilTemp(t); break;
      case ENV: boat.updateEnvTemp(t); break;
    }

    currentSensor = (currentSensor + 1) % SENSOR_ROLE_COUNT;
    lastRequest = millis();
  }
}