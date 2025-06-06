#pragma once
#include <Arduino.h>
#include <esp_system.h>
#include <esp_sleep.h>

class SystemStatus {
public:
    static void printResetReason() {
        esp_reset_reason_t reason = esp_reset_reason();
        Serial.print("Reset reason: ");
        switch (reason) {
            case ESP_RST_POWERON:   Serial.println("Power-on reset"); break;
            case ESP_RST_EXT:       Serial.println("External reset"); break;
            case ESP_RST_SW:        Serial.println("Software reset"); break;
            case ESP_RST_PANIC:     Serial.println("Crash (panic)"); break;
            case ESP_RST_DEEPSLEEP: Serial.println("Wake from deep sleep"); break;
            default:                Serial.println("Other/unknown"); break;
        }
    }

    static void printWakeupReason() {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        Serial.print("Wakeup cause: ");
        switch (cause) {
            case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Timer"); break;
            case ESP_SLEEP_WAKEUP_EXT0:  Serial.println("EXT0 (pin)"); break;
            case ESP_SLEEP_WAKEUP_EXT1:  Serial.println("EXT1 (multiple pins)"); break;
            case ESP_SLEEP_WAKEUP_ULP:   Serial.println("ULP"); break;
            case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Touchpad"); break;
            case ESP_SLEEP_WAKEUP_GPIO:  Serial.println("GPIO"); break;
            default:                     Serial.println("Unknown / not from sleep"); break;
        }
    }

    static void printUptime() {
        Serial.printf("Uptime: %.2f sec\n", millis() / 1000.0);
    }
};
