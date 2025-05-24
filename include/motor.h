#pragma once
enum MotorState {
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_REVERSE
};

// ===== Gesture detection thresholds =====
const int LOW_ZONE = 1090;
const int HIGH_ZONE = 1910;

// ===== Motor PWM range =====
const int PWM_MIN = 1000;
const int PWM_MAX = 1800;
const int PWM_BROKER = 1010;


extern int targetLeftPwm;
extern int targetRightPwm;
extern int currentLeftPwm;
extern int currentRightPwm;

extern MotorState motorState;
extern Servo escLeft;
extern Servo escRight;


void updatePwmLimit(uint16_t);
// ===== Основные функции =====
void setMotorPower(int percent);  // устанавливает целевое значение мощности
void applyMotorControl(int throttleInput, int steerInput);  // управляет моторами в зависимости от состояния
void checkMotorStateChange(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4);  // FSM переход

// ===== Вспомогательные функции =====
int mapWithDeadzone(int percent, int broker = 1100);  // мап с мёртвой зоной
void autoCalibrateESC(Servo& esc1, Servo& esc2);      // калибровка ESC (опционально)