#pragma once
#include <RadioLib.h>

#define BOAT_TMR_OIL_PUMP_UPDATE_TIME   360000
#define BOAT_TMR_STSTEM_PRINT_TIME      3600000


#ifdef HW_HELTEC
  #define I2C_SDA 21
  #define I2C_SCL 20
  #define GPS_RX 18
  #define GPS_TX 17
  #define GPS_PWR_PIN 4
  #define FLY_SKY_IBUS_RX_PIN 6
  #define IBUS_CONTROL_PIN 47

  #define MOTOR_LEFT 45
  #define MOTOR_RIGHT 42
  #define RUDDER 2
  #define DALLAS_ONEWIRE_PIN 7
  #define OIL_PUMP_PWM_PIN 3
  #define OILPUMP_PWM_CHANNEL 4
  #define OILPUMP_PWM_FREQ 1000
  #define OILPUMP_PWM_RES 8
  #define SONAR_HORISONT_SERVO 39
  #define SONAR_VERTICAL_SERVO 40
  #define SERVO_HOOK 41

  #define USE_IBUS
  #define USE_MDNS
  #define WIFI_SSID "Boat-ESP32"
  #define WIFI_PASS "polkalol"
  #define ADC_CTRL_PIN 37
  #define VBAT_READ_PIN 1
  #define VextCtrl 36
  #define SERVO_PWR_PIN 48

  #define LORA_SCK   9
  #define LORA_MISO  11
  #define LORA_MOSI  10
  #define LORA_SS    8
  #define LORA_RST   12
  #define LORA_DIO1  14
  #define LORA_BUSY  13


  // --- ПАРАМЕТРЫ LORA ---
  #define LORA_FREQUENCY      868.0 // МГц (для Европы), 915.0 для США/Австралии
  #define LORA_BANDWIDTH      125.0 // кГц (можно попробовать 62.5 для большей дальности)
  #define LORA_SF             11    // Spreading Factor (от 6 до 12, 11/12 для дальности)
  #define LORA_CODING_RATE    7     // Coding Rate (от 5 до 8, 7/8 для надежности)
  #define LORA_SYNC_WORD      RADIOLIB_SX126X_SYNC_WORD_PRIVATE // Важно, чтобы совпадал на всех устройствах
  #define LORA_TX_POWER       22    // dBm (макс. 22 для SX1262, проверьте свои региональные ограничения)
  #define LORA_PREAMBLE_LEN   8     // Длина преамбулы (обычно 8)

  #define DEVICE_ID_BOAT     0x01
  #define DEVICE_ID_BASE     0x02

  // Для лодки
  #ifdef IS_BOAT
    const uint8_t MY_DEVICE_ID = DEVICE_ID_BOAT;
    const uint8_t TARGET_DEVICE_ID = DEVICE_ID_BASE;
  #else // Для базовой станции
    const uint8_t MY_DEVICE_ID = DEVICE_ID_BASE;
    const uint8_t TARGET_DEVICE_ID = DEVICE_ID_BOAT;
  #endif  

#elif defined(HW_WROOM)
  #define I2C_SDA 40
  #define I2C_SCL 41
  #define GPS_RX 16
  #define GPS_TX 17
  #define MOTOR_LEFT 26
  #define MOTOR_RIGHT 27
  #define RUDDER 2
  #define DALLAS_ONEWIRE_PIN 4
  #define GPS_PWR_PIN 6
  #define OIL_PUMP_PWM_PIN 14
  #define OILPUMP_PWM_CHANNEL 4
  #define OILPUMP_PWM_FREQ 1000
  #define OILPUMP_PWM_RES 8
  #define SONAR_HORISONT_SERVO 12
  #define SONAR_VERTICAL_SERVO 13
  #define SERVO_HOOK 15
  #define FLY_SKY_IBUS_RX_PIN 33
  #define IBUS_CONTROL_PIN 5
  #define USE_IBUS
  #define USE_MDNS
  #define WIFI_SSID "Boat-WROOM"
  #define WIFI_PASS "defaultpass"
  #define ADC_CTRL_PIN 35
  #define VBAT_READ_PIN 1
  #define VextCtrl 5
  #define SERVO_PWR_PIN 48
#else
  #error "❌ Unknown hardware target! Define HW_HELTEC or HW_WROOM."
#endif


