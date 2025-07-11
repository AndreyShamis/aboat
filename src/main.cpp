#include <Arduino.h>
#include "Fusion.h"
#include <ESP32Servo.h>
#include "settings.h"
#include "Boat.hpp"
#include "web_interface.hpp"
#include "mission_control.hpp"
#include <SPIFFS.h>
#include <time.h>

unsigned long lastIMUUpdate = 0;
const unsigned long imuInterval = 10; // 100Hz обновление фильтра
unsigned long lastPrint = 0;
const unsigned long printInterval = 15000; // Печатаем каждые 5 секунд

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
  Serial.begin(115200);
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

  // Для дебага:
  static unsigned long lastStatusTime = 0;
  static unsigned long lastHeadingTime = 0;

  if (millis() - lastIMUUpdate >= imuInterval)
  {
    lastIMUUpdate = millis();
    // Delta time
    static unsigned long lastFusionTime = 0;
    unsigned long now = millis();
    float deltaTimeSeconds = (now - lastFusionTime) / 1000.0f;
    lastFusionTime = now;

    boat.mpu.accelUpdate();
    boat.mpu.gyroUpdate();
    boat.mpu.magUpdate();

    float ax = boat.mpu.accelX();
    float ay = boat.mpu.accelY();
    float az = boat.mpu.accelZ();
    float gx = boat.mpu.gyroX() * DEG_TO_RAD;
    float gy = boat.mpu.gyroY() * DEG_TO_RAD;
    float gz = boat.mpu.gyroZ() * DEG_TO_RAD;
    float mx = boat.mpu.magX();
    float my = boat.mpu.magY();
    float mz = boat.mpu.magZ();

    FusionVector gyro = {gx, gy, gz};
    FusionVector accel = {ax, ay, az};
    FusionVector mag = {mx, my, mz};

    FusionAhrsUpdate(&boat.ahrs, gyro, accel, mag, deltaTimeSeconds);
  }

  if (millis() - lastPrint >= printInterval)
  {

    lastPrint = millis();
    FusionQuaternion quat = FusionAhrsGetQuaternion(&boat.ahrs);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    // char buf[128];
    // snprintf(buf, sizeof(buf), "🔄 Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
    //          euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    // boat.addLog(buf);

    boat.printStatus();
  }
#elif defined(ROLE_MC)
  // Обрабатываем входящие LoRa-пакеты и внутренние таймеры MissionControl
  missionControl.loop();

  // Обрабатываем команды из Serial Monitor (включая SCAN)
  missionControl.handleSerialInput();
#endif
}
