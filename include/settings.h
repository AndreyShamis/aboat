#define I2C_SDA 21      // Compass I2C
#define I2C_SCL 20      // Compass I2C
#define GPS_RX 18       // GPS UART
#define GPS_TX 17       // GPS UART
#define MOTOR_LEFT 45   // ESC
#define MOTOR_RIGHT 42  // ESC
#define RUDDER_ON_OFF_PIN 47       // Servo

#define RUDDER 48       // Servo
#define RUDDER_PWM_CHANNEL 3
#define RUDDER_PWM_FREQ 50
#define RUDDER_PWM_RES 12

#define DALLAS_ONEWIRE_PIN 7

#define OIL_PUMP_PWM_PIN 4 // Oil pump pin
#define OILPUMP_PWM_CHANNEL 2
#define OILPUMP_PWM_FREQ 1000
#define OILPUMP_PWM_RES 8

#define SONAR_HORISONT_SERVO 39       // Servo
#define SONAR_VERTICAL_SERVO 40       // Servo
#define SERVO_HOOK 41       // Servo

// GPIO46 - broken
#define FLY_SKY_IBUS_RX_PIN  6 // FlySky IBUS RX pin 
#define IBUS_CONTROL_PIN 5 // FlySky IBUS control pin


//#define FLY_SKY_IBUS_RX_PIN  27 // FlySky IBUS RX pin 
// #define PPM_PIN 47      // FlySky

#define USE_IBUS

#define PWM_PIN       OIL_PUMP_PWM_PIN
#define TEMP_PIN      DALLAS_ONEWIRE_PIN


#define VextCtrl 36
