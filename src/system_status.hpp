#pragma once

#include <Arduino.h>
#include <esp_system.h>
#include <esp_sleep.h>
#include <ArduinoJson.h>


class SystemStatus {
public:
    static const char* getResetReasonStr() {
        esp_reset_reason_t reason = esp_reset_reason();
        switch (reason) {
            case ESP_RST_POWERON:   return "Power-on reset";
            case ESP_RST_EXT:       return "External reset";
            case ESP_RST_SW:        return "Software reset";
            case ESP_RST_PANIC:     return "Crash (panic)";
            case ESP_RST_DEEPSLEEP: return "Wake from deep sleep";
            default:                return "Other/unknown";
        }
    }

    static const char* getWakeupReasonStr() {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        switch (cause) {
            case ESP_SLEEP_WAKEUP_TIMER:    return "Timer";
            case ESP_SLEEP_WAKEUP_EXT0:     return "EXT0 (pin)";
            case ESP_SLEEP_WAKEUP_EXT1:     return "EXT1 (multiple pins)";
            case ESP_SLEEP_WAKEUP_ULP:      return "ULP";
            case ESP_SLEEP_WAKEUP_TOUCHPAD: return "Touchpad";
            case ESP_SLEEP_WAKEUP_GPIO:     return "GPIO";
            default:                        return "Unknown / not from sleep";
        }
    }

    static float getUptimeSeconds() {
        return millis() / 1000.0f;
    }

    static void printResetReason() {
        Serial.print("Reset reason: ");
        Serial.println(getResetReasonStr());
    }

    static void printWakeupReason() {
        Serial.print("Wakeup cause: ");
        Serial.println(getWakeupReasonStr());
    }

    static void printUptime() {
        Serial.printf("Uptime: %.2f sec\n", getUptimeSeconds());
    }

    static void toJSON(JsonObject& json) {
        json["reset_reason"] = getResetReasonStr();
        json["wakeup_reason"] = getWakeupReasonStr();
        json["uptime_sec"] = getUptimeSeconds();
    }

    static void toExtendedJSON(JsonObject& json) {
        toJSON(json); // base info first

        esp_chip_info_t chip;
        esp_chip_info(&chip);

        JsonObject chipInfo = json["chip"].to<JsonObject>();
        chipInfo["cores"] = chip.cores;
        chipInfo["revision"] = chip.revision;
        chipInfo["features"]["wifi"] = (chip.features & CHIP_FEATURE_WIFI_BGN) != 0;
        chipInfo["features"]["ble"] = (chip.features & CHIP_FEATURE_BLE) != 0;
        chipInfo["features"]["bt"] = (chip.features & CHIP_FEATURE_BT) != 0;

        json["flash_size"] = spi_flash_get_chip_size();
        json["heap_free"] = esp_get_free_heap_size();
        json["heap_psram"] = ESP.getPsramSize() > 0 ? ESP.getFreePsram() : 0;
        json["heap_min"] = esp_get_minimum_free_heap_size();
        json["idf_version"] = esp_get_idf_version();
    }
};
