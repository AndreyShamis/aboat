#pragma once

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "settings.h"

#define MAX_SENSORS 8

enum SensorRole
{
  MOTOR1,
  MOTOR2,
  MOTOR_RAD,
  OIL,
  ENV,
  SENSOR_ROLE_COUNT
};

class TempSensorManager
{
public:
  void begin()
  {
    shouldRun = true;
    oneWire = new OneWire(DALLAS_ONEWIRE_PIN);
    sensors = new DallasTemperature(oneWire);
    sensors->begin();

    sensorCount = sensors->getDeviceCount();
    Serial.printf("Found %d DS18B20 sensors\n", sensorCount);

    for (int i = 0; i < sensorCount && i < MAX_SENSORS; i++)
    {
      if (sensors->getAddress(sensorAddresses[i], i))
      {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" address: ");
        printAddress(sensorAddresses[i]);
        Serial.println();

        if (i < SENSOR_ROLE_COUNT)
        {
          memcpy(roleAddresses[i], sensorAddresses[i], 8);
        }
      }
    }

    xTaskCreatePinnedToCore(
        TempSensorManager::temperatureTaskWrapper,
        "TempTask",
        4096,
        this,
        1,
        &tempTaskHandle,
        0);

    //  создаём задачу через таймер, когда FreeRTOS точно уже жив
    // xTimerHandle tmr = xTimerCreate("TempStartDelay", pdMS_TO_TICKS(3000), pdFALSE, this, [](TimerHandle_t xTimer)
    //                                 {
    //   TempSensorManager* self = (TempSensorManager*)pvTimerGetTimerID(xTimer);
    //   xTaskCreatePinnedToCore(
    //       TempSensorManager::temperatureTaskWrapper,
    //       "TempTask",
    //       4096,
    //       self,
    //       1,
    //       &self->tempTaskHandle,
    //       0); });
    // xTimerStart(tmr, 0);
  }

  void stop()
  {
    shouldRun = false;
    if (tempTaskHandle)
    {
      vTaskDelete(tempTaskHandle);
      tempTaskHandle = nullptr;
    }
  }

  float get(SensorRole role)
  {
    if (role < SENSOR_ROLE_COUNT)
    {
      return latestTemperatures[role];
    }
    return -127.0f;
  }
  float getByAddressString(const String &addrStr)
  {
    if (addrStr.length() != 16)
      return -127.0f;

    DeviceAddress addr;
    for (uint8_t i = 0; i < 8; i++)
    {
      char byteStr[3] = {addrStr[i * 2], addrStr[i * 2 + 1], 0};
      addr[i] = strtoul(byteStr, nullptr, 16);
    }
    return sensors->getTempC(addr);
  }

  std::vector<String> getAllAddresses()
  {
    std::vector<String> result;
    for (int i = 0; i < sensorCount && i < MAX_SENSORS; i++)
    {
      char buffer[17];
      for (uint8_t j = 0; j < 8; j++)
        sprintf(buffer + j * 2, "%02X", sensorAddresses[i][j]);
      result.push_back(String(buffer));
    }
    return result;
  }
private:
  OneWire *oneWire = nullptr;
  DallasTemperature *sensors = nullptr;
  DeviceAddress sensorAddresses[MAX_SENSORS];
  DeviceAddress roleAddresses[SENSOR_ROLE_COUNT] = {0};
  float latestTemperatures[SENSOR_ROLE_COUNT] = {-127.0};
  int sensorCount = 0;
  TaskHandle_t tempTaskHandle = nullptr;
  bool shouldRun = true;


  void temperatureTask()
  {
    while (shouldRun)
    {
      sensors->requestTemperatures();
      for (int i = 0; i < SENSOR_ROLE_COUNT; i++)
      {
        latestTemperatures[i] = sensors->getTempC(roleAddresses[i]);
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
    }

    vTaskDelete(nullptr); // Завершаем текущую задачу корректно
  }
  static void temperatureTaskWrapper(void *param)
  {
    reinterpret_cast<TempSensorManager *>(param)->temperatureTask();
  }

  void printAddress(const DeviceAddress address)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      if (address[i] < 16)
        Serial.print("0");
      Serial.print(address[i], HEX);
    }
  }
};
