// mission_control.hpp
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "LoRaCore.hpp"
#include "wifi_manager.hpp"
#include "PacketAsaExchange.hpp"
#include "DataPacket.hpp"  // 🚀 NEW: Support for structured data

// -----------------------------------------------------------------------------
// MissionControl
//
// Handles sending commands and telemetry requests over LoRa,
// processes incoming fragments, ASA speed‐adaptation handshake, and ACKs.
// -----------------------------------------------------------------------------
class MissionControl : public LogInterface
{
public:
    unsigned long lastPingHeartbeat = 0;    // время последней посылки heartbeat ping
    unsigned long nextPingInterval = 15000; // первый интервал (мс)
    MissionControl()
    {
        loraComm = new LoRaCore(MY_DEVICE_ID, this);
    }

    ~MissionControl()
    {
        delete loraComm;
    }

    // Initialize LoRaComm
    void begin()
    {
        randomSeed(micros());
        
        WiFiManager::begin("Radiation", "polkalol", 3 * 3600, 30 * 60 * 1000);
        vTaskDelay(pdMS_TO_TICKS(1000));
        Serial.println(F("[MC] Starting Mission Control..."));
        if (!loraComm->begin())
        {
            addLog(F("[MC] ERROR: Failed to initialize LoRaCore"));
        }
        else
        {
            addLog(F("[MC] LoRaCore initialized successfully"));
            addLog(F("[MC] LoRaComm initialized"));
            // sendInfoRequest(CMD_TELEMETRY_FRAGMENT); // запросить телеметрию:
            // sendInfoRequest(CMD_INFO_ENGINE);        // запросить “InfoEngine”:
            // sendInfoRequest(CMD_STATUS);             // запросить общий статус:
        }
    }

    // Main loop: check for and handle incoming LoRa frames
    void loop()
    {

        uint8_t senderId;
        PacketBase hdr;
        uint8_t payloadBuf[MAX_LORA_PAYLOAD];

        LoRaPacket pkt = {}; // Initialize to zero

        if (loraComm->receive(pkt))
        {
            PacketBase hdr;
            hdr.packetType = pkt.packetType;
            hdr.packetId = pkt.packetId;
            hdr.payloadLen = pkt.payloadLen;
            if (pkt.receiverId == MISSION_CONTROL_ID)
            {
                handlePacket(pkt.senderId, hdr, pkt.payload);
            }
        }

        // 🕒 Проверка потери связи с лодкой (по любой активности, не только пингам)
        if (millis() - lastBoatActivity > checkInterval && currentProfileIndex > 0)
        {
            unsigned long timeSinceActivity = (millis() - lastBoatActivity) / 1000;
            addLog("⚠️ No boat activity for " + String(timeSinceActivity) + " seconds (threshold: " + 
                   String(checkInterval / 1000) + "s). Degrading to profile 0 for maximum range.");
            
            currentProfileIndex = 0;
            const auto &profile = loraProfiles[currentProfileIndex];

            PacketAsaApprove resp;
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);
            resp.profileIndex = currentProfileIndex;
            loraComm->sendPacketBase(BOAT_DEVICE_ID, resp, (const uint8_t*)&resp.profileIndex, false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            applyProfile(currentProfileIndex);

            lastBoatActivity = millis(); // 🚀 FIX: Сбрасываем таймер активности лодки
            
            // 🚀 FIX: Сбрасываем флаги пингов
            firstPingSent = false;
            secondPingSent = false;
            waitingForPong = false;
        }
        else
        {
            // 🚀 NEW: Двойная стратегия пингов - 35 и 45 секунд
            unsigned long timeSinceLastActivity = millis() - lastBoatActivity;
            
            // Первый пинг на 35 секунде
            if (timeSinceLastActivity > FIRST_PING_INTERVAL && !firstPingSent) {
                sendPingToBoat();
                lastPingSent = millis();
                waitingForPong = true;
                firstPingSent = true;
                addLog("[MC] 🔍 No boat activity for " + String(timeSinceLastActivity/1000) + 
                       "s, sending FIRST PING to check connection");
            }
            
            // Второй пинг на 45 секунде (если первый не получил ответ)
            if (timeSinceLastActivity > SECOND_PING_INTERVAL && firstPingSent && !secondPingSent) {
                sendPingToBoat();
                lastPingSent = millis();
                waitingForPong = true;
                secondPingSent = true;
                addLog("[MC] 🔍 No boat activity for " + String(timeSinceLastActivity/1000) + 
                       "s, sending SECOND PING to check connection");
            }
            
            // Если ждем понг больше 10 секунд - считаем лодку недоступной
            if (waitingForPong && (millis() - lastPingSent > PONG_TIMEOUT)) {
                String pingType = secondPingSent ? "second" : "first";
                addLog("[MC] ⚠️ " + pingType + " PING timeout! No PONG response from boat.");
                waitingForPong = false;
                boatIsActive = false;
                
                // 🚀 FIX: Не сбрасываем флаги пингов, пусть система дойдет до 60 секунд
                // и сделает деградацию профиля если лодка действительно недоступна
            }
            
            // Обычные heartbeat пинги
            if (millis() - lastPingHeartbeat >= nextPingInterval)
            {
                sendHeartbeatPing();
            }
        }
        
        // Проверяем, не нужно ли отправить накопленные ACK
        checkBulkAckTimeout();
    }

    // Send an arbitrary command string (e.g. “M:120”)
    void sendCommandString(const String &cmdStr)
    {
        if (cmdStr.length() > MAX_LORA_PAYLOAD)
        {
            addLog("[MC] ⚠️ Command too long (" + String(cmdStr.length()) +
                   " bytes), truncating to " + String(MAX_LORA_PAYLOAD));
        }

        size_t len = std::min<size_t>(cmdStr.length(), MAX_LORA_PAYLOAD);

        // 👇 Локальный буфер, живёт до вызова sendPacketBase → безопасно
        uint8_t tempBuf[MAX_LORA_PAYLOAD];
        memcpy(tempBuf, cmdStr.c_str(), len);

        PacketCommand cmd{};
        cmd.packetType = CMD_COMMAND_STRING;
        cmd.packetId = nextPacketId++;
        cmd.payloadLen = len;

        // 👇 Передаём стабильный буфер
        loraComm->sendPacketBase(BOAT_DEVICE_ID, cmd, tempBuf);
    }
    void sendHeartbeatPing()
    {
        PacketCommand ping{};
        ping.packetType = CMD_PING;
        ping.packetId = nextPacketId++;
        ping.payloadLen = 0;

        loraComm->sendPacketBase(BOAT_DEVICE_ID, ping, nullptr, false);
        addLog(F("[MC] 💓 Heartbeat PING sent to boat (expecting PONG)"));

        // cформируем новый случайный интервал 10–15 с
        nextPingInterval = random(10000UL, 15001UL);
        lastPingHeartbeat = millis();
    }
    
    // 🚀 NEW: Send ping to boat for active connection checking
    void sendPingToBoat()
    {
        PacketCommand ping{};
        ping.packetType = CMD_PING;
        ping.packetId = nextPacketId++;
        ping.payloadLen = 0;

        loraComm->sendPacketBase(BOAT_DEVICE_ID, ping, nullptr, false);
        addLog("[MC] 🏓 Connection check PING sent to boat (expecting PONG)");
    }

    void addLog(const String &msg) override
    {
        Serial.println("[" + timeStr() + "] " + msg);
    }

    void applyProfile(uint8_t idx)
    {
        if (idx >= LORA_PROFILE_COUNT) {
            addLog("ERROR: Bad profile index: " + String(idx));
            return;
        }
        
        const auto& profile = loraProfiles[idx];
        
        // Используем новый метод LoRaCore для применения профилей из settings.h
        if (loraComm->applyProfileFromSettings(idx)) {
            currentProfileIndex = idx;
            
            String modeStr = (profile.mode == RadioProfileMode::FSK) ? "GFSK" : "LoRa";
            
            if (profile.mode == RadioProfileMode::LORA) {
                addLog("DEBUG: applied " + modeStr + " profile " + String(idx) + 
                       " (SF=" + String(profile.spreadingFactor) + 
                       ", CR=" + String(profile.codingRate) + 
                       ", BW=" + String(profile.bandwidth, 1) + "kHz)");
            } else {
                addLog("DEBUG: applied " + modeStr + " profile " + String(idx) + 
                       " (Bitrate=" + String(profile.bitrate) + 
                       ", Dev=" + String(profile.deviation) + 
                       ", RxBW=" + String(profile.bandwidth, 1) + "kHz)");
            }            
        } else {
            addLog("ERROR: Cannot apply profile " + String(idx));
        }
    }

    void scanSpectrumCSV(float startMHz = 863.0f,
                         float stopMHz = 928.0f,
                         float stepMHz = 1.0f,
                         uint8_t samples = 10,
                         uint16_t dwellMs = 25)
    {
        // 1. Save current settings
        auto &radio = loraComm->getRadio(); // RadioLib instance
        float origFreq = LORA_FREQUENCY;

        addLog(F("=== LoRa spectrum scan CSV (freq_MHz,rssi_dBm) ==="));

        // 2. Sweep the band
        for (float f = startMHz; f <= stopMHz + 1e-6; f += stepMHz)
        {
            radio.setFrequency(f);
            // Continuous RX lets RSSI settle
            radio.startReceive(); // switch to RX mode

            float sum = 0;
            for (uint8_t i = 0; i < samples; i++)
            {
                delay(dwellMs);
                sum += radio.getRSSI(); // instant RSSI in dBm
            }
            float avg = sum / samples;
            Serial.printf("%.1f,%.1f\n", f, avg);
        }

        addLog(F("=== scan done ==="));

        // 3. Restore original settings
        radio.setFrequency(origFreq);
        radio.standby(); // back to standby; LoRaComm keeps config
        // если нужно перезапустить приём → LoRaCore::resumeRX();
    }

    // ============================================================
    // COMMAND METHODS - Add all command methods here in public section
    // ============================================================
    
    // Send diagnostic command (D) to boat
    void sendDiagnosticCommand(const String &param = "")
    {
        String cmdStr = "D";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[sendDiagnosticCommand]" + cmdStr);
    }

    // Send LoRa command (L) to boat
    void sendLoRaCommand(const String &param = "")
    {
        String cmdStr = "L";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        // Check if this is a profile change command and synchronize
        if (param.length() > 0 && param[0] >= '0' && param[0] <= '9') {
            // Direct profile number input (e.g., "1", "12", etc.)
            int profileIndex = param.toInt();
            if (profileIndex >= 0 && profileIndex < LORA_PROFILE_COUNT) {
                addLog("[sendLoRaCommand]Synchronizing MissionControl to profile " + String(profileIndex) + " after sending command");
                vTaskDelay(pdMS_TO_TICKS(1000));
                applyProfile(profileIndex);
                addLog("[sendLoRaCommand]Profile " + String(profileIndex) + " applied");
            }
        } else if (param.startsWith("P") || param.startsWith("profile:")) {
            // Profile command with P prefix or profile: prefix
            String profileStr = param.startsWith("profile:") ? param.substring(8) : param.substring(1);
            int profileIndex = profileStr.toInt();
            if (profileIndex >= 0 && profileIndex < LORA_PROFILE_COUNT) {
                addLog("[sendLoRaCommand]Synchronizing MissionControl to profile " + String(profileIndex) + " after sending command");
                vTaskDelay(pdMS_TO_TICKS(1000));
                applyProfile(profileIndex);
                addLog("[sendLoRaCommand]Profile " + String(profileIndex) + " applied");
            }
        }
        addLog("[LORA:CMD->BOAT]" + cmdStr);
    }

    // Send navigation command (N) to boat
    void sendNavigationCommand(const String &param = "")
    {
        String cmdStr = "N";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[sendNavigationCommand]" + cmdStr);
    }

    // Send web interface command (W) to boat
    void sendWebCommand(const String &param = "")
    {
        String cmdStr = "W";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[sendWebCommand]" + cmdStr);
    }

    // Convenience methods for specific navigation operations
    void startNavigation(float targetLat, float targetLon)
    {
        String param = "start:" + String(targetLat, 6) + "," + String(targetLon, 6);
        sendNavigationCommand(param);
    }

    void stopNavigation()
    {
        sendNavigationCommand("stop");
    }

    void pauseNavigation()
    {
        sendNavigationCommand("pause");
    }

    void resumeNavigation()
    {
        sendNavigationCommand("resume");
    }

    void setNavigationMode(const String &mode)
    {
        sendNavigationCommand("mode:" + mode);
    }

    // Convenience methods for LoRa operations
    void requestLoRaStatus()
    {
        sendLoRaCommand("status");
    }

    void setLoRaProfile(uint8_t profileIndex)
    {
        sendLoRaCommand("profile:" + String(profileIndex));
    }

    void adaptLoRa()
    {
        sendLoRaCommand("adapt");
    }

    // Convenience methods for diagnostic operations
    void requestFullDiagnostics()
    {
        sendDiagnosticCommand("full");
    }

    void requestSensorStatus()
    {
        sendDiagnosticCommand("sensors");
    }

    void requestPerformanceMetrics()
    {
        sendDiagnosticCommand("performance");
    }

    void requestSafetyStatus()
    {
        sendDiagnosticCommand("safety");
    }

    // Convenience methods for web interface operations
    void requestWebStatus()
    {
        sendWebCommand("status");
    }

    void updateWebInterface()
    {
        sendWebCommand("update");
    }

    // Legacy command methods for convenience
    void sendEngineCommand(int power)
    {
        String cmdStr = "M:" + String(power);
        sendCommandString(cmdStr);
        addLog("[sendEngineCommand]" + cmdStr);
    }

    void emergencyStop()
    {
        sendCommandString("E");
        addLog("[MC] 🛑 Emergency stop sent!");
    }

    void requestTelemetry()
    {
        sendCommandString("R");
        addLog("[MC] 📊 Telemetry request sent");
    }

    void sendPingCommand()
    {
        sendCommandString("P");
        addLog("[MC] 🏓 Ping sent");
    }

    // Help system - display available commands
    void printHelp()
    {
        Serial.println("\n╔══════════════════════════════════════════════════════════════════╗");
        Serial.println("║                     MISSION CONTROL COMMANDS                      ║");
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        Serial.println("║ 🔧 Diagnostic Commands (D:)                                     ║");
        Serial.println("║   D:S    - Safety status                                        ║");
        Serial.println("║   D:P    - Performance metrics                                  ║");
        Serial.println("║   D:T    - Temperature readings                                 ║");
        Serial.println("║   D:R    - Reset performance metrics                           ║");
        Serial.println("║   D:full - Full diagnostic report                              ║");
        Serial.println("║   D:ext  - Extended diagnostic report (longer)                 ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 📡 LoRa/GFSK Commands (L:)                                     ║");
        Serial.println("║   L:S         - LoRa status                                     ║");
        Serial.println("║   L:P[0-12]   - Set profile (0-8=LoRa, 9-12=GFSK)              ║");
        Serial.println("║                 0=🛡️Max range, 8=🚀Fast LoRa, 12=🚀Max GFSK     ║");
        Serial.println("║   L:adapt     - Trigger adaptive LoRa                          ║");
        Serial.println("║   L:A:0       - Disable adaptive switching (manual mode)       ║");
        Serial.println("║   L:A:1       - Enable adaptive switching (auto mode)          ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 🔧 Adaptive Control Examples:                                  ║");
        Serial.println("║   L:A:0; L:6  - Lock to profile 6 (disable auto-switching)     ║");
        Serial.println("║   L:A:1       - Re-enable auto profile optimization            ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 📋 Profile Details:                                            ║");
        Serial.println("║   LoRa Profiles (0-8): SF12→SF7, bandwidth 125→500kHz         ║");
        Serial.println("║   GFSK Profiles (9-12): 9.6k→76.8k bitrate, excellent signal  ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 🧭 Navigation Commands (N:)                                    ║");
        Serial.println("║   N:M       - Manual mode                                       ║");
        Serial.println("║   N:R       - Return to home                                    ║");
        Serial.println("║   N:S       - Station keeping                                   ║");
        Serial.println("║   N:H       - Set current position as home                      ║");
        Serial.println("║   N:stop    - Stop navigation                                   ║");
        Serial.println("║   N:pause   - Pause navigation                                  ║");
        Serial.println("║   N:resume  - Resume navigation                                 ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 🌐 Waypoint Commands (W:)                                      ║");
        Serial.println("║   W:A55.123,37.456  - Add waypoint                             ║");
        Serial.println("║   W:S               - Start waypoint following                 ║");
        Serial.println("║   W:C               - Clear waypoints                          ║");
        Serial.println("║   W:status          - Web interface status                     ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 📦 Structured Data Commands (BS:) - NEW BoatSettings API       ║");
        Serial.println("║   BS:H      - Request structured heartbeat (basic status)      ║");
        Serial.println("║   BS:F      - Request full structured status                   ║");
        Serial.println("║   BS:G      - Request GPS data only                            ║");
        Serial.println("║   BS:M      - Request motor status only                        ║");
        Serial.println("║   BS:S      - Request sensor data only (temp, battery)         ║");
        Serial.println("║   BS:L      - Request LoRa status only                         ║");
        Serial.println("║   BS:N      - Request navigation status only                   ║");
        Serial.println("║   BS:SYS    - Request system info (uptime, memory, health)     ║");
        Serial.println("║   BS:ON     - Enable structured data mode                      ║");
        Serial.println("║   BS:OFF    - Disable structured data mode (use JSON)          ║");
        Serial.println("║   BS:PING   - Send manual ping to boat                         ║");
        Serial.println("║   BS:RSSI   - Request RSSI report from boat                    ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ ⚙️ Legacy Commands:                                            ║");
        Serial.println("║   M:120     - Set motor power                                   ║");
        Serial.println("║   E         - Emergency stop                                    ║");
        Serial.println("║   R         - Request telemetry (old JSON system)              ║");
        Serial.println("║   P         - Send ping (legacy command)                       ║");
        Serial.println("║   P:1-P:100 - Oil pump control (1-100% power)                  ║");
        Serial.println("║   SCAN      - Spectrum scan (CSV output)                       ║");
        Serial.println("║   profiles  - Show all available radio profiles                ║");
        Serial.println("║   status    - Show connection status and boat diagnostics      ║");
        Serial.println("║   ping      - Send manual ping to check boat connection        ║");
        Serial.println("║   demo      - Run demo commands sequence                       ║");
        Serial.println("║   help      - Show this help                                   ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 💡 Convenience methods (call directly in code):                ║");
        Serial.println("║   mc.printHelp()                  - Show this help              ║");
        Serial.println("║   mc.startNavigation(lat, lon)    - Start navigation          ║");
        Serial.println("║   mc.setLoRaProfile(profileIndex) - Set LoRa profile          ║");
        Serial.println("║   mc.requestSensorStatus()        - Request sensor status     ║");
        Serial.println("║   mc.setNavigationMode(\"manual\")  - Set navigation mode       ║");
        Serial.println("╚══════════════════════════════════════════════════════════════════╝");
        Serial.println();
    }

    // Показать информацию о всех доступных профилях
    void printProfileInfo()
    {
        Serial.println("\n╔══════════════════════════════════════════════════════════════════╗");
        Serial.println("║                      RADIO PROFILE INFORMATION                   ║");
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        
        for (uint8_t i = 0; i < LORA_PROFILE_COUNT; i++) {
            const auto& profile = loraProfiles[i];
            String line = "║ " + String(i, DEC);
            if (i < 10) line += " ";
            
            if (profile.mode == RadioProfileMode::LORA) {
                line += ": LoRa SF" + String(profile.spreadingFactor) + 
                       " CR4/" + String(profile.codingRate) + 
                       " BW" + String(profile.bandwidth, 0) + "k";
            } else {
                line += ": GFSK " + String(profile.bitrate/1000.0f, 1) + "kb/s" +
                       " dev" + String(profile.deviation/1000.0f, 1) + "k" +
                       " bw" + String(profile.bandwidth, 0) + "k";
            }
            
            // Дополняем пробелами до 66 символов
            while (line.length() < 67) line += " ";
            line += "║";
            Serial.println(line);
        }
        
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        String currentInfo = "║ Current: " + String(loraComm->getCurrentProfileIndex()) + " (" + 
                             loraComm->getCurrentProfileInfo() + ")";
        // Дополняем пробелами до 67 символов  
        while (currentInfo.length() < 67) currentInfo += " ";
        currentInfo += "║";
        Serial.println(currentInfo);
        Serial.println("╚══════════════════════════════════════════════════════════════════╝");
    }

    // Process commands from Serial Monitor
    void processSerialCommand(const String& cmd) 
    {
        if (cmd.length() == 0) return;
        
        addLog("[MC] 🚀 Processing command: " + cmd);
        
        if (cmd.startsWith("D:")) {
            String param = cmd.substring(2);
            sendDiagnosticCommand(param);
        }
        else if (cmd.startsWith("L:")) {
            String param = cmd.substring(2);
            sendLoRaCommand(param);
        }
        else if (cmd.startsWith("N:")) {
            String param = cmd.substring(2);
            sendNavigationCommand(param);
        }
        else if (cmd.startsWith("W:")) {
            String param = cmd.substring(2);
            sendWebCommand(param);
        }
        else if (cmd.startsWith("M:")) {
            // Legacy motor command
            int power = cmd.substring(2).toInt();
            sendEngineCommand(power);
        }
        else if (cmd == "E") {
            emergencyStop();
        }
        else if (cmd == "R") {
            requestTelemetry();
        }
        else if (cmd.startsWith("P:")) {
            // Oil pump command P:1-P:100
            String param = cmd.substring(2);
            int pumpPower = param.toInt();
            if (pumpPower >= 1 && pumpPower <= 100) {
                sendOilPumpCommand(pumpPower);
            } else {
                addLog("[MC] ❌ Invalid pump power: " + param + " (must be 1-100)");
            }
        }
        else if (cmd == "P") {
            sendPingCommand();
        }
        else if (cmd.startsWith("BS:")) {
            // 📦 NEW: Structured Data Commands for BoatSettings
            String param = cmd.substring(3);
            
            if (param == "H") {
                // Request structured heartbeat
                sendCommandString("DM:H");
                addLog("[MC] 📦 Requested structured heartbeat");
            }
            else if (param == "F") {
                // Request full structured status
                sendCommandString("DM:F");
                addLog("[MC] 📦 Requested full structured status");
            }
            else if (param == "G") {
                // Request GPS data only
                sendCommandString("DM:G");
                addLog("[MC] 📦 Requested GPS data");
            }
            else if (param == "M") {
                // Request motor status only
                sendCommandString("DM:M");
                addLog("[MC] 📦 Requested motor status");
            }
            else if (param == "S") {
                // Request sensor data only
                sendCommandString("DM:SENS");
                addLog("[MC] 📦 Requested sensor data");
            }
            else if (param == "L") {
                // Request LoRa status only
                sendCommandString("DM:L");
                addLog("[MC] 📦 Requested LoRa status");
            }
            else if (param == "N") {
                // Request navigation status only
                sendCommandString("DM:N");
                addLog("[MC] 📦 Requested navigation status");
            }
            else if (param == "SYS") {
                // Request system info
                sendCommandString("DM:SYS");
                addLog("[MC] 📦 Requested system info");
            }
            else if (param == "ON") {
                // Enable structured data mode
                sendCommandString("DM:S");
                addLog("[MC] 📦 Enabled structured data mode");
            }
            else if (param == "OFF") {
                // Disable structured data mode (use JSON)
                sendCommandString("DM:J");
                addLog("[MC] 📦 Switched to JSON mode");
            }
            else if (param == "PING") {
                // Send manual ping
                sendPingCommand();
                addLog("[MC] 🔄 Manual ping sent");
            }
            else if (param == "RSSI") {
                // Request RSSI report
                sendCommandString("RSSI");
                addLog("[MC] 📶 Requested RSSI report");
            }
            else {
                addLog("[MC] ❌ Unknown BS command: " + param + ". Available: H,F,G,M,S,L,N,SYS,ON,OFF,PING,RSSI");
            }
        }
        else if (cmd == "help" || cmd == "HELP" || cmd == "h" || cmd == "H") {
            printHelp();
        }
        else if (cmd == "profiles" || cmd == "PROFILES") {
            printProfileInfo();
        }
        else if (cmd == "status" || cmd == "STATUS") {
            printConnectionStatus();
        }
        else if (cmd == "ping" || cmd == "PING") {
            // Manual ping to boat
            sendPingToBoat();
            lastPingSent = millis();
            waitingForPong = true;
            addLog("[MC] 🏓 Manual PING sent to boat (expecting PONG)");
        }
        else if (cmd == "demo") {
            // Quick demo commands - оптимизированная версия
            addLog("[MC] Processing command: demo");
            addLog("[MC] Running demo commands...");
            requestFullDiagnostics();
            addLog("[MC] 🔧 Sent diagnostic command: D:full");
            vTaskDelay(pdMS_TO_TICKS(500)); // Уменьшено с 2000 до 500ms
            requestLoRaStatus();
            addLog("[MC] 📡 Sent LoRa command: L:status");
            vTaskDelay(pdMS_TO_TICKS(500)); // Уменьшено с 2000 до 500ms
            sendNavigationCommand("status");
            addLog("[MC] Sent navigation command: N:status");
            vTaskDelay(pdMS_TO_TICKS(800)); // Уменьшено с 2000 до 500ms
            sendPingCommand();
            addLog("[MC] Ping sent");
            addLog("[MC] ✅ Demo complete");
        }
        else if (cmd == "SCAN") {
            // Spectrum scanning command
            addLog("[MC] ⚡️ Remote scan requested");
            scanSpectrumCSV();
        }
        else {
            addLog("[MC] ❌ Unknown command: '" + cmd + "' Type 'HELP' or 'help' for available commands.");
        }
    }

    // Check and process Serial input (call this in your main loop)
    void handleSerialInput()
    {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            processSerialCommand(input);
        }
    }
    
    // 🚀 NEW: Connection status diagnostics
    void printConnectionStatus()
    {
        unsigned long timeSinceActivity = millis() - lastBoatActivity;
        Serial.println("\n╔══════════════════════════════════════════════════════════════════╗");
        Serial.println("║                       CONNECTION STATUS                          ║");
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        Serial.println("║ Boat Status: " + String(boatIsActive ? "🟢 ACTIVE" : "🔴 INACTIVE") + String(boatIsActive ? "                                      " : "                                    ") + "║");
        Serial.println("║ Last Activity: " + String(timeSinceActivity/1000) + " seconds ago" + String("                                ").substring(0, 34 - String(timeSinceActivity/1000).length()) + "║");
        Serial.println("║ Waiting for Pong: " + String(waitingForPong ? "YES" : "NO") + String(waitingForPong ? "                                  " : "                                   ") + "║");
        if (waitingForPong) {
            unsigned long waitTime = millis() - lastPingSent;
            Serial.println("║ Ping Wait Time: " + String(waitTime) + "ms" + String("                                    ").substring(0, 40 - String(waitTime).length()) + "║");
        }
        Serial.println("║ Current Profile: " + String(currentProfileIndex) + String("                                      ").substring(0, 38 - String(currentProfileIndex).length()) + "║");
        Serial.println("║ Last RSSI: " + String(lastRSSI) + "dBm" + String("                                        ").substring(0, 42 - String(lastRSSI).length()) + "║");
        Serial.println("║ Battery: " + String(lastBatteryPercent) + "%" + String("                                            ").substring(0, 46 - String(lastBatteryPercent).length()) + "║");
        Serial.println("║ System Health: " + String(lastSystemHealth) + "%" + String("                                      ").substring(0, 40 - String(lastSystemHealth).length()) + "║");
        Serial.println("╚══════════════════════════════════════════════════════════════════╝");
    }

private:
    LoRaCore *loraComm;
    PacketId_t nextPacketId = 0;
    unsigned long lastBoatActivity = 0;  // 🚀 FIX: Время последней активности лодки (любой пакет)
    int currentProfileIndex = 0;
    unsigned long checkInterval = 60 * 1000; // 🚀 FIX: Увеличено с 45 до 60 секунд

    // 🚀 NEW: Boat state tracking (для структурированных данных)
    unsigned long lastPacketReceived = 0;
    bool boatIsActive = false;
    GPSPoint lastKnownBoatPosition;
    int8_t lastRSSI = -120;
    uint8_t lastBatteryPercent = 0;
    uint8_t lastSystemHealth = 100;
    bool useStructuredData = true; // Использовать структурированные данные
    
    // 🚀 NEW: Active connection monitoring with double ping strategy
    unsigned long lastPingSent = 0;
    bool waitingForPong = false;
    bool firstPingSent = false;   // 🚀 NEW: Отслеживаем первый пинг на 35 сек
    bool secondPingSent = false;  // 🚀 NEW: Отслеживаем второй пинг на 45 сек
    static constexpr unsigned long FIRST_PING_INTERVAL = 35000;  // 35 секунд → первый пинг
    static constexpr unsigned long SECOND_PING_INTERVAL = 45000; // 45 секунд → второй пинг
    static constexpr unsigned long PONG_TIMEOUT = 10000; // 10 секунд ждем понг

    // Система агрегированных ACK
    PacketBulkAck pendingBulkAck;
    unsigned long lastBulkAckTime = 0;
    static constexpr unsigned long BULK_ACK_INTERVAL_MS = 1000; // 1 секунда
    static constexpr unsigned long BULK_ACK_MAX_WAIT_MS = 500;  // Максимум 0.5 сек ожидания

    String timeStr() const
    {
        time_t now = time(nullptr);
        if (now > 1600000000)
        {
            struct tm *t = localtime(&now);
            unsigned long currentMillis = millis();
            uint16_t milliseconds = currentMillis % 1000;
            char buf[9];
            strftime(buf, sizeof buf, "%H:%M:%S", t);
            char fullBuf[13];
            snprintf(fullBuf, sizeof(fullBuf), "%s.%03d", buf, milliseconds);
            return String(fullBuf);
        }
        
        // If system time is not synced, return default with millis
        unsigned long currentMillis = millis();
        uint16_t milliseconds = currentMillis % 1000;
        char defaultBuf[13];
        snprintf(defaultBuf, sizeof(defaultBuf), "00:00:00.%03d", milliseconds);
        return String(defaultBuf);
    }
    // Assemble telemetry JSON fragments
    void processTelemetryFragment(const String &frag)
    {
        // Format: “[i/N]…chunk…”
        int slashPos = frag.indexOf('/');
        int closePos = frag.indexOf(']');
        int idx = frag.substring(1, slashPos).toInt();
        int total = frag.substring(slashPos + 1, closePos).toInt();
        String chunk = frag.substring(closePos + 1);

        static String buffer;
        static int expected = 0;

        if (idx == 0)
        {
            buffer.clear();
            expected = total;
        }
        buffer += chunk;

        if (idx == expected - 1)
        {
            addLog(F("[MC] 🚀 Telemetry fully received:"));
            addLog(buffer);
            // TODO: deserializeJson(buffer) and handle data…
        }
    }
    // Assemble command response fragments
    void processCommandResponseFragment(const String &response)
    {
        // Check if this is a fragmented response
        if (response.startsWith("[") && response.indexOf("/") > 0 && response.indexOf("]") > 0) {
            // Format: "[i/N]...data..."
            int slashPos = response.indexOf('/');
            int closePos = response.indexOf(']');
            int idx = response.substring(1, slashPos).toInt();
            int total = response.substring(slashPos + 1, closePos).toInt();
            String chunk = response.substring(closePos + 1);

            static String responseBuffer;
            static int expectedResponseFragments = 0;

            if (idx == 0) {
                responseBuffer.clear();
                expectedResponseFragments = total;
                addLog("[MC] 📦 Starting to receive " + String(total) + " command response fragments");
            }
            
            responseBuffer += chunk;
            
            addLog("[MC] 📥 Fragment " + String(idx + 1) + "/" + String(total) + " received (" + String(chunk.length()) + " bytes)");

            if (idx == expectedResponseFragments - 1) {
                addLog("[MC] ✅ Complete command response received (" + String(responseBuffer.length()) + " bytes total)");
                processCompleteCommandResponse(responseBuffer);
                responseBuffer.clear();
                expectedResponseFragments = 0;
            }
        } else {
            // Single fragment response
            processCompleteCommandResponse(response);
        }
    }
    
    // Process complete command response (assembled from fragments or single message)
    void processCompleteCommandResponse(const String &response)
    {
        addLog("[MC] 📥 Command response: " + response);
        
        // Parse response to determine command type and result
        if (response.startsWith("D:")) {
            addLog("[MC] 🔧 Diagnostic response: " + response.substring(2));
        } else if (response.startsWith("L:")) {
            addLog("[MC] 📡 LoRa response: " + response.substring(2));
        } else if (response.startsWith("N:")) {
            addLog("[MC] 🧭 Navigation response: " + response.substring(2));
        } else if (response.startsWith("W:")) {
            addLog("[MC] 🌐 Web response: " + response.substring(2));
        } else if (response.startsWith("DM:")) {
            // 🚀 NEW: Handle structured data mode responses
            addLog("[MC] 📦 StructData response: " + response.substring(3));
        } else {
            addLog("[MC] ❓ Unknown response format: " + response);
        }
    }
    // Handle incoming PacketBase + payloadBuf
    void handlePacket(uint8_t sender,
                      const PacketBase &hdr,
                      const uint8_t *buf)
    {
        // 🚀 FIX: Обновляем время активности лодки при получении ЛЮБОГО пакета от неё
        if (sender == BOAT_DEVICE_ID) {
            lastBoatActivity = millis();
            boatIsActive = true;
            
            // 🚀 FIX: Сбрасываем флаги пингов при любой активности лодки
            if (firstPingSent || secondPingSent) {
                addLog("[MC] 🚤 Boat activity detected - resetting ping state");
                firstPingSent = false;
                secondPingSent = false;
                waitingForPong = false;
            }
            
            // Логируем каждый 10-й пакет от лодки для мониторинга активности
            static uint8_t packetCounter = 0;
            packetCounter++;
            if (packetCounter % 10 == 0) {
                addLog("[MC] 🚤 Boat activity: packet " + String(packetCounter) + 
                       ", type=" + String((char)hdr.packetType) + ", id=" + String(hdr.packetId));
            }
        }
        
        float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();
        addLog("profile:" + String(currentProfileIndex) + " .RSSI:" + String(rssi) + "dBm, SNR=" + String(snr, 1) + "dB");
        switch (hdr.packetType)
        {
            case CMD_BULK_ACK:
            {
                // 🚀 FIX: BULK_ACK тоже является активностью лодки (обрабатывается выше в общем блоке)
                
                // Обработка bulk ACK от лодки
                if (hdr.payloadLen < sizeof(uint8_t)) {
                    addLog("[MC] ⚠️ Invalid BULK ACK packet size");
                    break;
                }
                
                uint8_t count = buf[0];
                if (count > 10 || hdr.payloadLen != sizeof(uint8_t) + (count * sizeof(PacketId_t))) {
                    addLog("[MC] ⚠️ Invalid BULK ACK count: " + String(count));
                    break;
                }
                
                addLog("[MC] ✅ Received BULK ACK for " + String(count) + " packets");
                for (uint8_t i = 0; i < count; i++) {
                    PacketId_t ackedId;
                    memcpy(&ackedId, &buf[1 + i * sizeof(PacketId_t)], sizeof(PacketId_t));
                    addLog("[MC] 📨 Confirmed packet ID: " + String(ackedId));
                }
                break;
            }
            
            // case CMD_ACK:
            // {
            //     if (hdr.payloadLen != sizeof(uint16_t))
            //     { // защита от пустых ACK-ов
            //         addLog("[MC] ⚠️ ACK with invalid size" + String(hdr.payloadLen) +
            //                " from " + String(sender) + ", expected " + String(sizeof(uint16_t)));
            //         break;
            //     }
            //     uint16_t ackedId;
            //     memcpy(&ackedId, buf, sizeof(ackedId));
            //     addLog("[MC] ACK for packet ID: " + String(ackedId));
            //     break;
            // }

        case CMD_TELEMETRY_FRAGMENT:
        {
            String fragment(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            processTelemetryFragment(fragment);
            break;
        }
        case CMD_REQUEST_ASA:
        case CMD_REPOSNCE_ASA:
        {
            uint8_t profileIndex = 0;
            if (!parseAsaPacket(hdr, buf, profileIndex))
            {
                addLog("1 [MC] Invalid ASA packet received");
                break;
            }
            
            // 🚀 FIX: ASA пакеты тоже являются активностью лодки (обрабатывается выше в общем блоке)
            
            // Ответим только на CMD_REQUEST_ASA
            if (hdr.packetType == CMD_REQUEST_ASA)
            {
                vTaskDelay(pdMS_TO_TICKS(10)); 
                sendAsaResponse(loraComm, nextPacketId++, profileIndex, sender);
                addLog("3 [MC] ASA Response sent to LORA");
                vTaskDelay(pdMS_TO_TICKS(450)); 
            }
            else
            {
                addLog("4 [MC] ASA Response received, applying profile index " + String(profileIndex));
            }
            if(loraComm->getOutgoingQueueCount() > 0 || loraComm->getPendingCount() > 0) {
                addLog("[MC] Outgoing queue count or pending: " + String(loraComm->getOutgoingQueueCount()));
                vTaskDelay(pdMS_TO_TICKS(401)); // Ждем, чтобы пакет ушел
            }
            if(loraComm->getPendingCount() > 0) {
                addLog("[MC] Get Pending Count: " + String(loraComm->getPendingCount()));
                vTaskDelay(pdMS_TO_TICKS(409)); // Ждем, чтобы пакет ушел
            }
            if(loraComm->getOutgoingQueueCount() > 0 || loraComm->getPendingCount() > 0) {
                addLog("[MC] Outgoing queue count or pending:" + String(loraComm->getOutgoingQueueCount()));
                vTaskDelay(pdMS_TO_TICKS(607)); // Ждем, чтобы пакет ушел
            }
            applyProfile(profileIndex);
            break;
        }

        case CMD_PING:
        {
            PacketCommand pong;
            pong.packetType = CMD_PONG;
            pong.packetId = nextPacketId++;
            pong.payloadLen = 0;
            // loraComm->sendPacket(pong, sender);
            loraComm->sendPacketBase(sender, pong, nullptr, false);
            addLog("[MC] 📥 Received PING from boat, sent PONG response");
            break;
        }
        
        case CMD_PONG:
        {
            // 🚀 NEW: Handle pong responses to our pings
            if (waitingForPong) {
                unsigned long pingTime = millis() - lastPingSent;
                String pingType = secondPingSent ? "second" : "first";
                addLog("[MC] ✅ Boat responded to " + pingType + " PING with PONG in " + String(pingTime) + "ms - connection OK");
                waitingForPong = false;
                boatIsActive = true;
                // 🚀 FIX: Сбрасываем время последней активности при получении понга
                lastBoatActivity = millis();
                // 🚀 FIX: Сбрасываем флаги пингов при успешном ответе
                firstPingSent = false;
                secondPingSent = false;
            } else {
                addLog("[MC] 📥 Received heartbeat PONG from boat (not in response to our PING)");
            }
            break;
        }
        
        case CMD_RSSI_REPORT:
        {
            addLog("Got CMD_RSSI_REPORT");
            PacketRssiReport rpt{};
            rpt.packetType = hdr.packetType;
            rpt.packetId = hdr.packetId;
            rpt.payloadLen = hdr.payloadLen;

            memcpy(reinterpret_cast<uint8_t *>(&rpt) + sizeof(PacketBase), buf, hdr.payloadLen);

            addLog("📥RSSI:raw=" + String(rpt.rawRssi)+" ,smoothed=" + String(rpt.smoothedRssi));
            break;
        }
        case CMD_COMMAND_RESPONSE:
        {
            // Response to our D, L, N, W commands
            String response(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            processCommandResponseFragment(response);
            break;
        }

        case CMD_STATUS:
        {
            // Enhanced status information from boat
            String status(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            addLog("[MC] 📊 Enhanced status: " + status);
            break;
        }

        case CMD_INFO_ENGINE:
        {
            // Engine information response
            String engineInfo(reinterpret_cast<const char *>(buf), hdr.payloadLen);
            addLog("[MC] ⚙️ Engine info: " + engineInfo);
            break;
        }

        // 🚀 NEW: Structured data packet handling
        case CMD_STRUCTURED_HEARTBEAT:
        {
            UltraCompactHeartbeat hb;
            if (StructuredDataManager::parseHeartbeat(buf, hdr.payloadLen, hb)) {
                // Обновляем последнее время получения от лодки
                lastPacketReceived = millis();
                boatIsActive = true;
                
                // Отображаем компактную информацию
                addLog("[MC] 📡 Boat HB: " + String(hb.latitude, 6) + "," + String(hb.longitude, 6) + 
                       " P:" + String(hb.loraProfile) + 
                       " RSSI:" + String(hb.rssi) + 
                       " Bat:" + String(hb.batteryPercent) + "%");
                       
                // Сохраняем состояние лодки (если нужно)
                lastKnownBoatPosition.latitude = hb.latitude;
                lastKnownBoatPosition.longitude = hb.longitude;
                lastRSSI = hb.rssi;
                lastBatteryPercent = hb.batteryPercent;
                lastSystemHealth = hb.systemHealth;
                
            } else {
                addLog("[MC] ❌ Failed to parse ultra heartbeat");
            }
            break;
        }
        case CMD_STRUCTURED_GPS:
        {
            GPSStatus gps;
            if (StructuredDataManager::parseGPS(buf, hdr.payloadLen, gps)) {
                addLog("[MC] 📍 GPS: " + gps.toString());
            } else {
                addLog("[MC] ❌ Failed to parse GPS data");
            }
            break;
        }
        case CMD_STRUCTURED_MOTORS:
        case CMD_STRUCTURED_SENSORS:
        case CMD_STRUCTURED_NAVIGATION:
        case CMD_STRUCTURED_FRAGMENT:
        {
            addLog("[MC] 📦 Struct data type: " + String((char)hdr.packetType) + ", size: " + String(hdr.payloadLen));
            break;
        }

        default:
            // Unhandled packet types …
            addLog("[MC] Unhandled packet type: [" + String((char)hdr.packetType) +
                   "][" + String((int)hdr.packetType) + "], sender=" + String(sender) +
                   ", payloadLen=" + String(hdr.payloadLen));
            break;
        }

        if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_BULK_ACK && hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG && hdr.packetType != CMD_REQUEST_ASA && hdr.packetType != CMD_RSSI_REPORT)
        {
            // Используем новую систему bulk ACK вместо мгновенной отправки
            addToBulkAck(hdr.packetId);
        }
    }
    // MissionControl.hpp
    void sendInfoRequest(CommandType what)
    {
        // что именно спрашиваем: T/I/S/F/G/D/K…
        PacketCommand cmd{};
        cmd.packetType = CMD_REQUEST_INFO;
        cmd.packetId = nextPacketId++;
        cmd.payloadLen = 1;
        // первый байт payload — нужный код
        uint8_t code = what;
        loraComm->sendPacketBase(BOAT_DEVICE_ID, cmd, (const uint8_t*)&code);
        addLog("[MC] Sent P(request info=" + String((char)what) + ") to boat");
    }
    void sendOilPumpCommand(int power)
    {
        String cmdStr = "P:" + String(power);
        sendCommandString(cmdStr);
        addLog("[MC] 🛢️ Sent oil pump command: " + cmdStr + " (power: " + String(power) + "%)");
    }

    // Добавить ACK в bulk пакет
    void addToBulkAck(PacketId_t packetId)
    {
        if (pendingBulkAck.addAck(packetId)) {
            addLog("[MC] 📦 Added ACK for packet " + String(packetId) + " to bulk (" + String(pendingBulkAck.count) + "/10)");
            
            // Если пакет заполнен или прошло достаточно времени - отправляем
            if (pendingBulkAck.isFull() || 
                (millis() - lastBulkAckTime > BULK_ACK_MAX_WAIT_MS && !pendingBulkAck.isEmpty())) {
                sendBulkAck();
            }
        } else {
            // Пакет переполнен - отправляем текущий и начинаем новый
            sendBulkAck();
            if (pendingBulkAck.addAck(packetId)) {
                addLog("[MC] 📦 Started new bulk ACK with packet " + String(packetId));
            }
        }
    }
    
    // Отправить накопленные ACK
    void sendBulkAck()
    {
        if (pendingBulkAck.isEmpty()) return;
        
        // Диагностика перед отправкой
        if (pendingBulkAck.hasDuplicates()) {
            addLog("[MC] ⚠️ WARNING: BULK ACK contains duplicates: " + pendingBulkAck.getDebugInfo());
        }
        
        pendingBulkAck.packetId = nextPacketId++;
        
        // Формируем payload: count + массив ID
        uint8_t payload[1 + 10 * sizeof(PacketId_t)];
        payload[0] = pendingBulkAck.count;
        memcpy(&payload[1], pendingBulkAck.ackedIds, pendingBulkAck.count * sizeof(PacketId_t));
        
        size_t payloadSize = sizeof(uint8_t) + (pendingBulkAck.count * sizeof(PacketId_t));
        
        // Отправляем как высокоприоритетный пакет
        LoRaPacket bulkPacket = {}; // Initialize to zero
        packBaseIntoLoRa(bulkPacket, MY_DEVICE_ID, BOAT_DEVICE_ID, pendingBulkAck, payload);
        
        if (loraComm->sendHighPriority(bulkPacket)) {
            addLog("[MC] ✅ Sent BULK ACK for " + String(pendingBulkAck.count) + " packets (high priority): " + pendingBulkAck.getDebugInfo());
        } else {
            // Если высокоприоритетная отправка не удалась, используем обычную
            loraComm->sendPacketBase(BOAT_DEVICE_ID, pendingBulkAck, payload, false);
            addLog("[MC] ✅ Sent BULK ACK for " + String(pendingBulkAck.count) + " packets (standard): " + pendingBulkAck.getDebugInfo());
        }
        
        pendingBulkAck.clear();
        lastBulkAckTime = millis();
    }
    
    // Проверить, нужно ли отправить bulk ACK по таймауту
    void checkBulkAckTimeout()
    {
        if (!pendingBulkAck.isEmpty() && 
            (millis() - lastBulkAckTime > BULK_ACK_INTERVAL_MS)) {
            sendBulkAck();
        }
    }
};
