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

    void enable() {
        digitalWrite(GPS_PWR_PIN, HIGH);
        enabled = true;
    }

    void disable() {
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
        StaticJsonDocument<256> doc;
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


// // gnss_manager.hpp
// #pragma once

// #include <TinyGPS++.h>
// #include <HardwareSerial.h>
// #include "settings.h"
// #include <ArduinoJson.h>

// class GNSSManager {
// public:
//     HardwareSerial& gnssSerial;
//     TinyGPSPlus gnss;
//     bool enabled = false;

//     GNSSManager(HardwareSerial& serial = Serial2)
//         : gnssSerial(serial) {}

//     void begin() {
//         pinMode(GPS_PWR_PIN, OUTPUT);
//         enable();
//         gnssSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
//     }

//     void enable() {
//         digitalWrite(GPS_PWR_PIN, HIGH);
//         enabled = true;
//     }

//     void disable() {
//         digitalWrite(GPS_PWR_PIN, LOW);
//         enabled = false;
//     }

//     void update() {
//         while (gnssSerial.available()) {
//             gnss.encode(gnssSerial.read());
//         }
//     }

//     void toJson(JsonObject obj) {
//         obj["enabled"] = enabled;
//         obj["lat"] = gnss.location.isValid() ? gnss.location.lat() : 0.0;
//         obj["lon"] = gnss.location.isValid() ? gnss.location.lng() : 0.0;
//         obj["alt"] = gnss.altitude.isValid() ? gnss.altitude.meters() : 0.0;
//         obj["speed"] = gnss.speed.isValid() ? gnss.speed.kmph() : -1;
//         obj["sats"] = gnss.satellites.isValid() ? gnss.satellites.value() : -1;
//         obj["hdop"] = gnss.hdop.isValid() ? gnss.hdop.hdop() : -1;
//         obj["fix"] = gnss.location.isValid() && gnss.satellites.value() > 3;

//         // Дополнительно для дебага:
//         obj["date"] = gnss.date.isValid() ? String(gnss.date.year()) + "-" + String(gnss.date.month()) + "-" + String(gnss.date.day()) : "n/a";
//         obj["time"] = gnss.time.isValid() ? String(gnss.time.hour()) + ":" + String(gnss.time.minute()) + ":" + String(gnss.time.second()) : "n/a";
//         obj["course"] = gnss.course.isValid() ? gnss.course.deg() : -1;
//         obj["age_ms"] = gnss.location.age();  // время, прошедшее с последнего обновления
//     }

//     String getStatusJson() {
//         StaticJsonDocument<256> doc;
//         toJson(doc.to<JsonObject>());
//         String result;
//         serializeJson(doc, result);
//         return result;
//     }

//     void sendUBX(const uint8_t* msg, uint8_t len) {
//         for (uint8_t i = 0; i < len; i++) {
//             gnssSerial.write(msg[i]);
//         }
//     }

//     void enablePSM() {
//         const uint8_t psmMsg[] = {
//             0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00,
//             0x00, 0x00, 0x00, 0x00,
//             0x00, 0x00, 0x00, 0x00,
//             0x00, 0x00,
//             0x01, 0x00,
//             0x00, 0x00,
//             0x00, 0x00,
//             0x00, 0x00,
//             0x00, 0x00, 0x00, 0x00,
//             0x00, 0x00, 0x00, 0x00,
//             0x00, 0x00, 0x00, 0x00,
//             0x00, 0x00
//         };
//         sendUBX(psmMsg, sizeof(psmMsg));
//     }
// };
