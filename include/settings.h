// File include/settings.h
#pragma once
#include <RadioLib.h>
#define LORA_PROFILE_COUNT 13  // Расширено с 9 до 13 для поддержки GFSK

// Унифицированный тип для всех packetId в системе
typedef uint8_t PacketId_t;

// Режимы радио
enum class RadioProfileMode : uint8_t {
    LORA = 0,
    FSK = 1
};

// Универсальная структура профилей: LoRa + GFSK
static constexpr struct {
    RadioProfileMode mode;     // LORA или FSK (в данном случае GFSK)
    float bandwidth;           // kHz (для LoRa) или rxBandwidth (для GFSK)
    int spreadingFactor;       // 7–12 (только для LoRa, для GFSK = 0)
    int codingRate;            // 5–8 (только для LoRa, для GFSK = 0)
    uint32_t bitrate;          // bit/s (только для GFSK, для LoRa = 0)
    uint32_t deviation;        // Hz (только для GFSK, для LoRa = 0)
} loraProfiles[LORA_PROFILE_COUNT] = {
    // LoRa профили (0-8): от максимально надёжного до быстрого
    {RadioProfileMode::LORA, 125.0, 12, 7, 0, 0},      // 0: 🛡️ Максимальная надёжность (очень медленно, максимум дальности)
    {RadioProfileMode::LORA, 125.0, 11, 7, 0, 0},      // 1: Очень хорошая устойчивость
    {RadioProfileMode::LORA, 125.0, 10, 7, 0, 0},      // 2: Надёжный компромисс
    {RadioProfileMode::LORA, 250.0,  9, 6, 0, 0},      // 3: Средний режим (городская застройка)
    {RadioProfileMode::LORA, 250.0,  8, 6, 0, 0},      // 4: Средний, открытая местность
    {RadioProfileMode::LORA, 250.0,  7, 5, 0, 0},      // 5: Быстро, при хорошем сигнале
    {RadioProfileMode::LORA, 500.0,  9, 5, 0, 0},      // 6: Скорость + дальность
    {RadioProfileMode::LORA, 500.0,  8, 5, 0, 0},      // 7: Очень быстрый, LOS желательно
    {RadioProfileMode::LORA, 500.0,  7, 5, 0, 0},      // 8: 🚀 Максимальная скорость LoRa
    // GFSK профили (9-12): SX1262/RadioLib поддерживает GFSK, не классический FSK
    {RadioProfileMode::FSK, 117.3, 0, 0, 19200, 10000}, // 9: 📡 GFSK стандартный (увеличена скорость)
    {RadioProfileMode::FSK, 156.2, 0, 0, 38400, 20000}, // 10: GFSK средний  
    {RadioProfileMode::FSK, 187.2, 0, 0, 50000, 25000}, // 11: GFSK быстрый 
    {RadioProfileMode::FSK, 234.3, 0, 0, 100000, 50000} // 12: 🚀 GFSK максимальный (увеличена скорость)
};

// Расширенная таблица мэппинга: RSSI >= X → профиль Y (включая FSK)
static constexpr struct {
    float minRssi;
    float minSnr;       // Добавляем SNR для более точного определения
    int profileIndex;
} rssiToProfileTable[] = {
    // GFSK профили для отличного сигнала (RSSI > -85 и SNR > 10)
    { -75.0f, 15.0f, 12 }, // Максимальная скорость GFSK (отличный сигнал)
    { -80.0f, 12.0f, 11 }, // GFSK быстрый
    { -85.0f, 10.0f, 10 }, // GFSK средний  
    { -90.0f,  8.0f,  9 }, // GFSK консервативный
    
    // LoRa профили для стандартных условий
    { -95.0f,  5.0f,  8 }, // LoRa максимальная скорость
    { -102.0f, 2.0f,  7 }, // LoRa очень быстрый
    { -105.0f, 0.0f,  6 }, // LoRa скорость + дальность
    { -110.0f, -2.0f, 5 }, // LoRa быстро при хорошем сигнале
    { -114.0f, -5.0f, 4 }, // LoRa средний, открытая местность
    { -115.0f, -8.0f, 3 }, // LoRa средний режим
    { -118.0f, -10.0f, 2}, // LoRa надёжный компромисс
    { -119.0f, -12.0f, 1}, // LoRa очень хорошая устойчивость
    { -120.0f, -15.0f, 0}  // LoRa максимальная надёжность
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
    CMD_BULK_ACK = 'B',     // B = Bulk ACK (агрегированные подтверждения)
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

