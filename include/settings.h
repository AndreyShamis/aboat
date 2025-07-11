// File include/settings.h
#pragma once
#include <RadioLib.h>
#define LORA_PROFILE_COUNT 9

// Пресеты: от максимально надёжного до максимально быстрого
static constexpr struct {
    float bandwidth;       // kHz
    int spreadingFactor;   // 7–12 // SF увеличивает надёжность и дальность, но снижает скорость экспоненциально.
    int codingRate;        // 5 = 4/5, 6 = 4/6, 7 = 4/7, 8 = 4/8
} loraProfiles[LORA_PROFILE_COUNT] = {
    {125.0, 12, 7},   // 0: 🛡️ Максимальная надёжность (очень медленно, максимум дальности)
    {125.0, 11, 7},   // 1: Очень хорошая устойчивость
    {125.0, 10, 7},   // 2: Надёжный компромисс
    {250.0,  9, 6},   // 3: Средний режим (городская застройка)
    {250.0,  8, 6},   // 4: Средний, открытая местность
    {250.0,  7, 5},   // 5: Быстро, при хорошем сигнале
    {500.0,  9, 5},   // 6: Скорость + дальность
    {500.0,  8, 5},   // 7: Очень быстрый, LOS желательно
    {500.0,  7, 5}    // 8: 🚀 Максимальная скорость (минимум надёжности, максимум throughput)
};


// вручную мэппинг: RSSI >= X → профиль Y
static constexpr struct {
    float minRssi;
    int profileIndex;
} rssiToProfileTable[] = {
    { -95.0f, 8 },
    { -102.0f, 7 },
    { -105.0f, 6 },
    { -110.0f, 5 },
    { -114.0f, 4 },
    { -115.0f, 3 },
    { -118.0f, 2 },
    { -119.0f, 1 },
    { -120.0f, 0 }
};

// количество строк таблицы
constexpr size_t rssiProfileCount = sizeof(rssiToProfileTable) / sizeof(rssiToProfileTable[0]);


enum CommandType : uint8_t
{
    CMD_NONE = 0, // нет команды
    CMD_COMMAND_STRING = 'C',     // C = Command строка
    CMD_COMMAND_RESPONSE = 'Y',   // Y = Command response (ответ на команду)
    CMD_TELEMETRY_FRAGMENT = 'T', // T = Telemetry фрагмент JSON
    CMD_INFO_ENGINE = 'I', // I = InfoEngine
    CMD_STATUS = 'S',      // S = Status
    CMD_CONFIG = 'F',      // F = ConFig
    CMD_NAV = 'G',         // G = NaVigation
    CMD_ACK_ASA = 'A', // A = Ack ASA (подтверждение адаптации)
    CMD_REQUEST_ASA = ')', // ) = Request ASA (запрос авто-адаптации)
    CMD_REPOSNCE_ASA = '(', 
    CMD_GET_BOAT_STATUS = 'Q',    // Q = Query boat Status (запрос статуса)
    CMD_BOAT_STATUS_REPORT = 'D', // D = Data report (отчёт статуса)
    CMD_ACK = 'K',          // K = aCK (универсальное «ок»)
    CMD_REQUEST_INFO = 'W', // W = request any Info
    CMD_PING = '-',
    CMD_PONG = 'O',
    CMD_RSSI_REPORT = 'R', // R = Report RSSI
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
  #define LORA_FREQUENCY      863.21 //868.0
  #define LORA_BANDWIDTH      125.0 // кГц (можно попробовать 62.5 для большей дальности)
  #define LORA_SF             12    // Spreading Factor (от 6 до 12, 11/12 для дальности)
  #define LORA_CODING_RATE    7     // Coding Rate (от 5 до 8, 7/8 для надежности)
  #define LORA_SYNC_WORD      0x16 // Важно, чтобы совпадал на всех устройствах
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

// Enhanced safety and monitoring settings
#define SAFETY_TEMP_CRITICAL     85.0f  // °C - Critical temperature for emergency shutdown
#define SAFETY_TEMP_WARNING      75.0f  // °C - Warning temperature
#define SAFETY_VOLTAGE_LOW       11.0f  // V - Low voltage warning
#define SAFETY_VOLTAGE_CRITICAL  10.5f  // V - Critical voltage for shutdown
#define SAFETY_COMM_TIMEOUT      60000  // ms - Communication timeout
#define SAFETY_KEEP_CYCLE_WARN   50     // ms - Warning threshold for long keep() cycles

// Performance monitoring settings
#define PERFORMANCE_CACHE_TIME   100    // ms - Sensor cache validity time
#define PERFORMANCE_LOG_INTERVAL 300000 // ms - Performance stats logging interval

// Navigation settings
#define NAV_DEFAULT_CRUISE_SPEED 0.5f   // m/s - Default cruise speed
#define NAV_DEFAULT_MAX_SPEED    1.0f   // m/s - Maximum speed
#define NAV_WAYPOINT_TOLERANCE   10.0f  // m - Default waypoint reach tolerance
#define NAV_APPROACH_DISTANCE    50.0f  // m - Distance to start slowing down

// Enhanced LoRa settings
#define LORA_ADAPTIVE_INTERVAL   25011  // ms - Adaptive switching interval
#define LORA_ASA_TIMEOUT         15000  // ms - ASA acknowledgment timeout
#define LORA_PING_INTERVAL       20000  // ms - Ping interval
#define LORA_RSSI_REPORT_INTERVAL 35000 // ms - RSSI report interval
