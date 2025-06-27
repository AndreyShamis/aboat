// File include/settings.h
#pragma once
#include <RadioLib.h>
#define LORA_PROFILE_COUNT 20

// Пресеты: от максимально надёжного до максимально быстрого
static constexpr struct {
    float bandwidth;
    int spreadingFactor;
    int codingRate;
} loraProfiles[LORA_PROFILE_COUNT] = {
    {  62.5, 12, 8 },  // 0: 🛟 Ultra Rescue     (~0.28 kbps)
    {  62.5, 12, 7 },  // 1: Rescue slow        (~0.35 kbps)
    { 125.0, 12, 7 },  // 2: Rescue             (~0.7 kbps)
    { 125.0, 11, 7 },  // 3: Remote control     (~1.1 kbps)
    { 125.0, 11, 5 },  // 4: Stable             (~1.6 kbps)
    { 125.0, 10, 6 },  // 5: Fallback           (~2.5 kbps)
    { 125.0, 10, 5 },  // 6: Long range         (~2.9 kbps)
    { 125.0,  9, 5 },  // 7: Balanced           (~5.2 kbps)
    { 250.0, 10, 5 },  // 8: Balanced+          (~5.8 kbps)
    { 250.0,  9, 6 },  // 9: Stable mid         (~7.5 kbps)
    { 250.0,  9, 5 },  //10: Short link         (~8.5 kbps)
    { 250.0,  8, 5 },  //11: Short+             (~12.4 kbps)
    { 250.0,  7, 5 },  //12: Fast mid           (~14.0 kbps)
    { 500.0,  9, 5 },  //13: High-speed         (~17.1 kbps)
    { 500.0,  8, 6 },  //14: Very fast +CR      (~21.0 kbps)
    { 500.0,  8, 5 },  //15: Very fast          (~24.0 kbps)
    { 500.0,  7, 6 },  //16: Max speed +CR      (~25.0 kbps)
    { 500.0,  7, 5 },  //17: ⚡ Max speed        (~29.3 kbps)
    { 250.0,  7, 5 },  //18: Fast+ mid          (~14.0 kbps)
    { 125.0,  8, 5 },  //19: Special case       (~9.0 kbps, more stable)
};



// -----------------------------------------------------------------------------
// --- СВОЙ ПРОТОКОЛ СООБЩЕНИЙ (буквенные коды для удобства) ---
// -----------------------------------------------------------------------------
enum CommandType : uint8_t
{
    CMD_NONE = 0, // нет команды

    CMD_COMMAND_STRING = 'C',     // C = Command строка
    CMD_TELEMETRY_FRAGMENT = 'T', // T = Telemetry фрагмент JSON

    CMD_INFO_ENGINE = 'I', // I = InfoEngine
    CMD_STATUS = 'S',      // S = Status
    CMD_CONFIG = 'F',      // F = ConFig
    CMD_NAV = 'G',         // G = NaVigation

    // CMD_REQUEST_ASA = 'R',          // R = Request ASA (запрос авто-адаптации)
    CMD_ACK_ASA = 'A', // A = Ack ASA (подтверждение адаптации)
    CMD_REQUEST_ASA = ')', // ) = Request ASA (запрос авто-адаптации)
    CMD_REPOSNCE_ASA = '(', 
    CMD_GET_BOAT_STATUS = 'Q',    // Q = Query boat Status (запрос статуса)
    CMD_BOAT_STATUS_REPORT = 'D', // D = Data report (отчёт статуса)

    CMD_ACK = 'K',          // K = aCK (универсальное «ок»)
    CMD_REQUEST_INFO = 'W', // W = request any Info
    CMD_PING = '-',
    CMD_PONG = 'O',

};


#define BOAT_DEVICE_ID 0x01
#define MISSION_CONTROL_ID 0x02

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
  #ifdef ROLE_BOAT
    const uint8_t MY_DEVICE_ID = BOAT_DEVICE_ID;
    const uint8_t TARGET_DEVICE_ID = MISSION_CONTROL_ID;
  #else // Для базовой станции
    const uint8_t MY_DEVICE_ID = MISSION_CONTROL_ID;
    const uint8_t TARGET_DEVICE_ID = BOAT_DEVICE_ID;
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

