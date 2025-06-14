#include <Arduino.h>
#include <ESP32Servo.h>
#include "Fusion.h"
#include <ESP32Servo.h>
#include "settings.h"
#include "Boat.hpp"
#include <Adafruit_PWMServoDriver.h>
#include "web_interface.hpp"
#include <SPIFFS.h>




unsigned long lastIMUUpdate = 0;
const unsigned long imuInterval = 10; // 100Hz обновление фильтра
unsigned long lastPrint = 0;
const unsigned long printInterval = 15000; // Печатаем каждые 5 секунд
static unsigned long lastChannelPrint = 0;
const unsigned long channelPrintInterval = 2000; // 2.5 секунды
static unsigned long lastControlUpdate = 0;
const unsigned long controlInterval = 100; // 100 мс = 10 Гц

Boat boat;
String inputBuffer;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
WebInterface *webInterface = nullptr;



uint16_t microsecondsToTicks(uint16_t us)
{
  return us * 4096L / 20000;
}

void setup()
{
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  boat.sensorsPowerOff(); // Выключаем питание датчиков

  Serial.begin(115200);
  Serial.println("\n\n\n");

    if (!SPIFFS.begin(true)) {
        Serial.println("- SPIFFS failed to mount");
  } else {
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
  delay(10);
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Гц для серво
;


}


static unsigned long lastRandomRudderTime = 0;
static unsigned long nextRandomRudderDelay = 0;
// ==== Loop ====
void loop()
{
  
  webInterface->handle();
  if(boat.updateStarted){
    if (Update.isRunning()) {
      Update.printError(Serial);
      Serial.println("Update in progress, please wait...");
      return;
    }
    boat.updateStarted = false;
  }
  boat.keep();
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      Serial.println("Command processed: " + inputBuffer);
      boat.parser.processLine(inputBuffer);
      inputBuffer = "";
    }
    else if (c >= 32 && c <= 126)
    {
      inputBuffer += c;
    }
  }
 
  uint16_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;

  ch1 = boat.flysky.getChannel(0); // Правый stick X
  ch2 = boat.flysky.getChannel(1); // Правый stick Y
  ch3 = boat.flysky.getChannel(2); // Левый stick Y
  ch4 = boat.flysky.getChannel(3); // Левый stick X
  ch5 = boat.flysky.getChannel(4);
  ch6 = boat.flysky.getChannel(5);
  ch7 = boat.flysky.getChannel(6);
  ch8 = boat.flysky.getChannel(7);
  ch9 = boat.flysky.getChannel(8);
  ch10 = boat.flysky.getChannel(9);
  //}

if (millis() - lastRandomRudderTime >= nextRandomRudderDelay) {
  lastRandomRudderTime = millis();
  nextRandomRudderDelay = random(20, 2500); // 3–5 секунд

  int randomAngle = random(-91, 91); // Угол от -60 до 60
  boat.rudder.setAngle(randomAngle);

  //Serial.printf("🎲 Random rudder angle set to: %d°\n", randomAngle);
}
  if (ch5 < 1900)
  {
    boat.engine.setState(boat.engine.MOTOR_STOP);
  }
  else
  {
    if (ch6 < 1500 && ch6 >= 1000)
    {
      boat.engine.setState(boat.engine.MOTOR_FORWARD);
    }
    else if (ch6 > 1500 && ch6 <= 2000)
    {
      boat.engine.setState(boat.engine.MOTOR_REVERSE);
    }
    else
    {
      boat.engine.setState(boat.engine.MOTOR_STOP);
    }
  }
  if (millis() - lastChannelPrint > channelPrintInterval)
  {
    lastChannelPrint = millis();
    //  Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u\n", ch1, ch2, ch3, ch4);
    // Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u -- CH5: %u | CH6: %u | CH7: %u | CH8: %u :::: CH9: %u - CH10: %u\n", ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,ch9, ch10);
  }

  lastControlUpdate = millis();
  boat.setThrottleLimit(ch7);
  boat.engine.apply(ch3, ch4);

  if (ch1 >= 1000 && ch1 <= 2000 && boat.flysky.transmitter_on)
  {
    boat.rudder.setAngle(map(ch1, 1000, 2000, -90, 90)); // Обновляем руль только если есть валидный сигнал
  }

  boat.rudder.update();
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

  
}
