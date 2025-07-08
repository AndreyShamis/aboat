// gnss_manager.hpp
#pragma once

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
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
        delay(100);

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
        obj["time"] = String(gnss.getHour()) + ":" + String(gnss.getMinute()) + ":" + String(gnss.getSecond());
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
};
