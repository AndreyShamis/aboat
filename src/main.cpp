#include <Arduino.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <MPU9250_asukiaaa.h>
#include "Fusion.h"
#include <ESP32Servo.h>
#include "settings.h"
#include "rudder.hpp"
#include "motor.hpp"
// #include "EncoderHandler.h"
#include "fly_sky.hpp"
#include "Boat.hpp"
#include "temp_sensors.hpp"
#include "oil_pump.hpp"
#include <Adafruit_PWMServoDriver.h>

FusionAhrs ahrs;
TinyGPSPlus gps;
MPU9250_asukiaaa mpu;
HardwareSerial GPSserial(2);
Boat boat;


// ==== Global state ====

static uint32_t lastPpmCounter = 0;
static uint32_t lastPpmCheckTime = 0;
bool ppmSignalAlive = true;
unsigned long lastIMUUpdate = 0;
const unsigned long imuInterval = 10; // 100Hz обновление фильтра
unsigned long lastPrint = 0;
const unsigned long printInterval = 5000; // Печатаем каждые 5 секунд
static unsigned long lastChannelPrint = 0;
const unsigned long channelPrintInterval = 200; // 2.5 секунды

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#include "voltage_current_sensor.hpp"

VoltageCurrentSensor sensor(0x48); // <-- укажи адрес ADS1115


uint16_t microsecondsToTicks(uint16_t us)
{
  return us * 4096L / 20000;
}

void setup()
{
  pinMode(VextCtrl, OUTPUT);
  digitalWrite(VextCtrl, LOW); // включить питание на Ve
  delay(10);
  Serial.begin(115200);
  Serial.println("\n\n\n");
  Serial.println("Boat Control System Starting...\n\n");
  GPSserial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); // GPS UART
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
    delay(4);
  }

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  FusionAhrsInitialise(&ahrs);

  Serial.println("MPU9250 + Madgwick initialized");
  // ESCs
  escLeft.setPeriodHertz(50);
  escLeft.attach(MOTOR_LEFT, 1000, 2000);
  escRight.setPeriodHertz(50);
  escRight.attach(MOTOR_RIGHT, 1000, 2000);
  // autoCalibrateESC(escLeft,escRight);
  // Rudders
  setupRudderPwm();

  setRudderTrim(0, 0);
  setRudderAngle(0);
  setupTempSensors();
  // setupOilPump();
  Serial.println("System Ready: ESC x2 + Rudders Initialized");
  delay(1000);
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Гц для серво

  sensor.begin();
  // // Пример: установить сигнал на 0-й канал
  // pwm.setPWM(0, 0, microsecondsToTicks(0));


    //
}
 static unsigned long lastControlUpdate = 0;
const unsigned long controlInterval = 100;  // 100 мс = 10 Гц

// ==== Loop ====
void loop()
{
  // updateEncoder();

  uint16_t ch1 = getFlySkyChannel(0); // Правый stick X
  uint16_t ch2 = getFlySkyChannel(1); // Правый stick Y
  uint16_t ch3 = getFlySkyChannel(2); // Левый stick Y
  uint16_t ch4 = getFlySkyChannel(3); // Левый stick X

  uint16_t ch5 = getFlySkyChannel(4);
  uint16_t ch6 = getFlySkyChannel(5);
  uint16_t ch7 = getFlySkyChannel(6);
  uint16_t ch8 = getFlySkyChannel(7);
  uint16_t ch9 = getFlySkyChannel(8);
  uint16_t ch10 = getFlySkyChannel(9);
  if (ch5 < 1900)
  {
    motorState = MOTOR_STOP;
  }
  else
  {
    motorState = MOTOR_FORWARD;
  }
  if (millis() - lastChannelPrint > channelPrintInterval)
  {
    lastChannelPrint = millis();
    //  Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u\n", ch1, ch2, ch3, ch4);
    Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u -- CH5: %u | CH6: %u | CH7: %u | CH8: %u :::: CH9: %u - CH10: %u\n", ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,ch9, ch10);
  }
  // updatePwmLimit(ch7);
  // checkMotorStateChange(ch1, ch2, ch3, ch4);
  // applyMotorControl(ch3, ch4);

  // setRudderAngle(map(ch1, 1000, 2000, -60, 60));
  // updateRudder();
  

//if (millis() - lastControlUpdate >= controlInterval) {
  lastControlUpdate = millis();

  updatePwmLimit(ch7);
  checkMotorStateChange(ch1, ch2, ch3, ch4);
  applyMotorControl(ch3, ch4);
  if (ch1 >=1000 && ch1 <= 2000)
  {
  setRudderAngle(map(ch1, 1000, 2000, -60, 60));
  }
  

 
updateRudder();
  // Для дебага:
  static unsigned long lastStatusTime = 0;
  static unsigned long lastHeadingTime = 0;

  // ==== Read GPS ====
  while (GPSserial.available())
  {
    char c = GPSserial.read();
    if (millis() - lastStatusTime >= 15000)
    {
      Serial.write(c); //  NMEA
    }
    gps.encode(c);
  }

  // Каждые 15 секунд
  if (millis() - lastStatusTime >= 15000)
  {
    lastStatusTime = millis();
    if (gps.location.isValid())
    {
      Serial.printf("✅ GPS FIX: Lat: %.6f, Lon: %.6f, Alt: %.1f m, Sats: %u\n",
                    gps.location.lat(),
                    gps.location.lng(),
                    gps.altitude.meters(),
                    gps.satellites.value());
    }
    else
    {
      Serial.println("🔄 Waiting for GPS fix...");
      if (gps.satellites.isValid())
      {
        Serial.printf("Satellites in view: %d\n", gps.satellites.value());
      }
      else
      {
        Serial.printf("Satellites info not yet available.\n");
      }
      if (gps.altitude.isValid())
      {
        Serial.printf("Altitude: %d\n", gps.altitude.meters());
      }
      if (gps.speed.isValid())
      {
        Serial.printf("Speed: %d  km/h\n", gps.speed.kmph());
      }
    }
  }

  if (millis() - lastIMUUpdate >= imuInterval)
  {
    lastIMUUpdate = millis();
    // Delta time
    static unsigned long lastFusionTime = 0;
    unsigned long now = millis();
    float deltaTimeSeconds = (now - lastFusionTime) / 1000.0f;
    lastFusionTime = now;

    mpu.accelUpdate();
    mpu.gyroUpdate();
    mpu.magUpdate();

    float ax = mpu.accelX();
    float ay = mpu.accelY();
    float az = mpu.accelZ();
    float gx = mpu.gyroX() * DEG_TO_RAD;
    float gy = mpu.gyroY() * DEG_TO_RAD;
    float gz = mpu.gyroZ() * DEG_TO_RAD;
    float mx = mpu.magX();
    float my = mpu.magY();
    float mz = mpu.magZ();

    FusionVector gyro = {gx, gy, gz};
    FusionVector accel = {ax, ay, az};
    FusionVector mag = {mx, my, mz};

    FusionAhrsUpdate(&ahrs, gyro, accel, mag, deltaTimeSeconds);
  }

  if (millis() - lastPrint >= printInterval)
  {
    
    
    lastPrint = millis();
    FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
    FusionEuler euler = FusionQuaternionToEuler(quat);
    Serial.printf("🔄 Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n",
                  euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    boat.printStatus();

      float v = 0, a = 0;
  sensor.read(v, a);
  Serial.printf("V: %.2f V\tI: %.3f A\n", v, a);
  }
  //updateTempsOneByOne();
  // updateOilPumpLogic();
}
