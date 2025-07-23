#include <Arduino.h>
#include <ESP32Servo.h>
#include "settings.h"
#include "Boat.hpp"
#include "web_interface.hpp"
#include "mission_control.hpp"
#include <SPIFFS.h>
#include <time.h>

unsigned long lastPrint = 0;
const unsigned long printInterval = 15000; // Печатаем каждые 15 секунд

Boat boat;

WebInterface *webInterface = nullptr;

// uint16_t microsecondsToTicks(uint16_t us)
// {
//   return us * 4096L / 20000;
// }
#if defined(ROLE_MC)
MissionControl missionControl;
#endif

void setup()
{
  Serial.begin(921600);
  Serial.println("\n\n\n");
  esp_log_level_set("*", ESP_LOG_VERBOSE);
#ifdef ROLE_BOAT
  boat.sensorsPowerOff(); // Выключаем питание датчиков
  if (!SPIFFS.begin(true))
  {
    Serial.println("- SPIFFS failed to mount");
  }
  else
  {
    Serial.println("+ SPIFFS PK");
  }
  Serial.println("Boat Control System Starting...\n\n");
  boat.sensorsPowerOn();
  // autoCalibrateESC(escLeft,escRight);

  webInterface = new WebInterface(boat);
  webInterface->begin();
  Serial.println("Web Server started");
  delay(10);
  boat.setup();

  Serial.println("System Ready: ESC x2 + Rudders Initialized");

#elif defined(ROLE_MC)
  missionControl.begin();
  Serial.println("Mission Control System Starting...\n");

  Serial.println("=== MISSION CONTROL READY ===");
  Serial.println("Available commands:");
  Serial.println("  D:S, D:P, D:T, D:full, D:ext - Diagnostics");
  Serial.println("  L:S, L:P[0-8], L:adapt       - LoRa control");
  Serial.println("  N:M, N:R, N:S, N:H           - Navigation");
  Serial.println("  W:A<lat,lon>, W:S, W:C       - Waypoints");
  Serial.println("  E, R, P, HELP, demo, SCAN    - Other commands");
  Serial.println("Type 'HELP' for detailed reference");
  Serial.println("================================\n");
#endif

  ;
}

// ==== Loop ====
void loop()
{
#ifdef ROLE_BOAT
  webInterface->handle();
  if (boat.updateStarted)
  {
    if (Update.isRunning())
    {
      Update.printError(Serial);
      Serial.println("Update in progress, please wait...");
      return;
    }
    boat.updateStarted = false;
  }
  boat.keep();

  // Периодический вывод статуса для отладки
  if (millis() - lastPrint >= printInterval)
  {
    lastPrint = millis();
    
    // Получаем ориентацию из IMU модуля для отладки
    const IMUModule::Orientation& orientation = boat.imu.getOrientation();
    
    // Можно раскомментировать для детального логирования:
    // boat.addLog("🔄 Roll: " + String(orientation.roll, 1) + "° Pitch: " + 
    //             String(orientation.pitch, 1) + "° Yaw: " + String(orientation.yaw, 1) + "°");

    boat.printStatus();
  }
#elif defined(ROLE_MC)
  // Обрабатываем входящие LoRa-пакеты и внутренние таймеры MissionControl
  missionControl.loop();

  // Обрабатываем команды из Serial Monitor (включая SCAN)
  missionControl.handleSerialInput();
#endif
}
