#pragma once

#include <Arduino.h>
#include <map>
#include <functional>

class CommandParser {
public:
    using Callback = std::function<void(const String& args)>;

    void registerCommand(const String& cmd, Callback cb) {
        commands[cmd] = cb;
    }

    void processLine(const String& line) {
        String payload = line;
        String crcStr;
        bool hasCRC = false;

        int asterisk = line.lastIndexOf('*');
        if (asterisk != -1 && asterisk < line.length() - 1) {
            payload = line.substring(0, asterisk);
            crcStr = line.substring(asterisk + 1);
            hasCRC = true;
        }

        if (hasCRC) {
            uint8_t expectedCrc = strtoul(crcStr.c_str(), nullptr, 16);
            uint8_t actualCrc = calcCRC(payload);
            if (expectedCrc != actualCrc) {
                Serial.println(F("! CRC ERROR"));
                return;
            }
        }

        int colon = payload.indexOf(':');
        String cmd = colon == -1 ? payload : payload.substring(0, colon);
        String args = colon == -1 ? "" : payload.substring(colon + 1);

        if (commands.count(cmd)) {
            commands[cmd](args);
        } else {
            Serial.print(F("! UNKNOWN CMD: "));
            Serial.println(cmd);
        }
    }

    static uint8_t calcCRC(const String& str) {
        uint8_t crc = 0;
        for (char c : str) crc ^= c;
        return crc;
    }

private:
    std::map<String, Callback> commands;
};
