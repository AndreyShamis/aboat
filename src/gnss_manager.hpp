// gnss_manager.hpp
#pragma once

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <time.h>
#include <sys/time.h>
#include "settings.h"

class GNSSManager {
public:
    SFE_UBLOX_GNSS gnss;
    HardwareSerial& gnssSerial;
    bool enabled = false;

    GNSSManager(HardwareSerial& serial = Serial2) : gnssSerial(serial) {}

    void begin() {
        pinMode(GPS_PWR_PIN, OUTPUT);
        enable();

    }

    void enable() {
        digitalWrite(GPS_PWR_PIN, HIGH);
        enabled = true;
        gnssSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
        delay(20);

        if (!gnss.begin(gnssSerial)) {
            Serial.println("GNSS not detected. Check wiring.");
            return;
        }

        gnss.setUART1Output(COM_TYPE_UBX); // Only UBX
        gnss.setNavigationFrequency(1);    // 1Hz updates
        gnss.setAutoPVT(true);             // Auto NAV-PVT reports
    }

    void disable() {
        gnssSerial.end(); // переинициализируйте порт
        digitalWrite(GPS_PWR_PIN, LOW);
        
        enabled = false;
    }

    void update() {
        gnss.checkUblox(); // non-blocking, pulls latest data
        
    }

    void toJson(JsonObject obj) {
        obj["enabled"] = enabled;
        obj["lat"] = gnss.getLatitude() / 10000000.0;
        obj["lon"] = gnss.getLongitude() / 10000000.0;
        obj["alt"] = gnss.getAltitude() / 1000.0;
        obj["speed"] = gnss.getGroundSpeed() / 1000.0;
        obj["sats"] = gnss.getSIV();
        obj["fix"] = gnss.getFixType() >= 3;
        obj["date"] = String(gnss.getYear()) + "-" + String(gnss.getMonth()) + "-" + String(gnss.getDay());
        char timeBuffer[16];
        snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d.%03d", 
                 gnss.getHour(), gnss.getMinute(), gnss.getSecond(), gnss.getMillisecond());
        obj["time"] = String(timeBuffer);
    }

    String getStatusJson() {
        JsonDocument doc;
        toJson(doc.to<JsonObject>());
        String result;
        serializeJson(doc, result);
        return result;
    }

    void enablePSM() {
        // Enable Power Save Mode
        gnss.powerSaveMode(true);
    }

    // Navigation helper methods
    bool hasValidFix() {
        return gnss.getFixType() >= 3; // 3D fix or better
    }
    
    float getLatitude() {
        return gnss.getLatitude() / 10000000.0; // Convert to degrees
    }
    
    float getLongitude() {
        return gnss.getLongitude() / 10000000.0; // Convert to degrees
    }
    
    float getHeading() {
        return gnss.getHeading() / 100000.0; // Convert to degrees
    }
    
    float getSpeed() {
        return gnss.getGroundSpeed() / 1000.0; // Convert to m/s
    }
    
    int getSatelliteCount() {
        return gnss.getSIV();
    }
    
    // Time access methods
    uint8_t getHour() {
        return gnss.getHour();
    }
    
    uint8_t getMinute() {
        return gnss.getMinute();
    }
    
    uint8_t getSecond() {
        return gnss.getSecond();
    }

    // Time synchronization methods
    bool hasValidTime() {
        return gnss.getTimeValid() && gnss.getDateValid();
    }
    
    bool isTimeUpdated() {
        // Check if time data has been updated since last check
        static uint32_t lastTimeOfWeek = 0;
        uint32_t currentTimeOfWeek = gnss.getTimeOfWeek();
        bool updated = (currentTimeOfWeek != lastTimeOfWeek);
        lastTimeOfWeek = currentTimeOfWeek;
        return updated;
    }
    
    void syncSystemTimeFromGPS() {
        if (hasValidTime()) {
            struct tm t;
            t.tm_year = gnss.getYear() - 1900;
            t.tm_mon = gnss.getMonth() - 1;
            t.tm_mday = gnss.getDay();
            t.tm_hour = gnss.getHour() + 3; // GMT+3 (adjust for your timezone)
            t.tm_min = gnss.getMinute();
            t.tm_sec = gnss.getSecond();
            t.tm_isdst = 0;

            time_t timeSinceEpoch = mktime(&t);
            struct timeval now = {.tv_sec = timeSinceEpoch};
            settimeofday(&now, nullptr);

            Serial.println("System time synced from GPS!");
        }
    }
    
    // Get GPS time as string for logging
    String getTimeString() {
        if (hasValidTime()) {
            char timeStr[36];
            uint16_t milliseconds = gnss.getMillisecond();
            snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d.%03d UTC", 
                     gnss.getYear(), gnss.getMonth(), gnss.getDay(),
                     gnss.getHour(), gnss.getMinute(), gnss.getSecond(), milliseconds);
            return String(timeStr);
        }
        return "Invalid GPS time";
    }
};
