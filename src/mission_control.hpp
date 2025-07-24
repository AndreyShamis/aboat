// mission_control.hpp
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "LoRaCore.hpp"
#include "wifi_manager.hpp"
#include "DataPacket.hpp"  // NEW: Support for structured data
#include "BoatSettings.hpp" // NEW: Boat status structure

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
            addLog("[MC][LORA]No boat activity for " + String(timeSinceActivity) + " seconds (threshold:" + String(checkInterval / 1000) + "s). Degrading to profile 0");
            
            currentProfileIndex = 0;
            const auto &profile = loraProfiles[currentProfileIndex];

            PacketAsaExchange resp(CMD_REPOSNCE_ASA);
            resp.profileIndex = currentProfileIndex;
            loraComm->sendPacketBase(BOAT_DEVICE_ID, resp, (const uint8_t*)&resp.profileIndex, false);
            vTaskDelay(pdMS_TO_TICKS(500)); vTaskDelay(pdMS_TO_TICKS(500)); vTaskDelay(pdMS_TO_TICKS(500));
            applyProfile(currentProfileIndex);
            lastBoatActivity = millis();    // Сбрасываем таймер активности лодки
            firstPingSent = false;          //  Сбрасываем флаги пингов
            secondPingSent = false;
            waitingForPong = false;
        }
        else
        {
            // Двойная стратегия пингов - 35 и 45 секунд
            unsigned long timeSinceLastActivity = millis() - lastBoatActivity;
            
            // Первый пинг на 35 секунде
            if (timeSinceLastActivity > FIRST_PING_INTERVAL && !firstPingSent) {
                sendPingToBoat();
                lastPingSent = millis();
                waitingForPong = true;
                firstPingSent = true;
                addLog("[MC] 🔍 No boat activity for " + String(timeSinceLastActivity/1000) + "s, sending FIRST PING to check connection");
            }
            
            // Второй пинг на 45 секунде (если первый не получил ответ)
            if (timeSinceLastActivity > SECOND_PING_INTERVAL && firstPingSent && !secondPingSent) {
                sendPingToBoat();
                lastPingSent = millis();
                waitingForPong = true;
                secondPingSent = true;
                addLog("[MC] 🔍 No boat activity for " + String(timeSinceLastActivity/1000) + "s, sending SECOND PING to check connection");
            }
            
            // Если ждем понг больше 10 секунд - считаем лодку недоступной
            if (waitingForPong && (millis() - lastPingSent > PONG_TIMEOUT)) {
                String pingType = secondPingSent ? "second" : "first";
                addLog("[MC] ⚠️ " + pingType + " PING timeout! No PONG response from boat.");
                waitingForPong = false;
                boatIsActive = false;
                
                // Не сбрасываем флаги пингов, пусть система дойдет до 60 секунд
                // и сделает деградацию профиля если лодка действительно недоступна
            }
            
            // Обычные heartbeat пинги
            if (millis() - lastPingHeartbeat >= nextPingInterval)
            {
                sendHeartbeatPing();
            }
        }
        
        // Проверяем, не нужно ли отправить накопленные ACK
        loraComm->processBulkAckTimeout(BOAT_DEVICE_ID);
    }

    // Send an arbitrary command string (e.g. “M:120”)
    void sendCommandString(const String &cmdStr)
    {
        if (cmdStr.length() > MAX_LORA_PAYLOAD)
        {
            addLog("[MC] ⚠️ Command too long (" + String(cmdStr.length()) +" bytes), truncating to " + String(MAX_LORA_PAYLOAD));
        }

        size_t len = std::min<size_t>(cmdStr.length(), MAX_LORA_PAYLOAD);

        // Локальный буфер, живёт до вызова sendPacketBase → безопасно
        uint8_t tempBuf[MAX_LORA_PAYLOAD];
        memcpy(tempBuf, cmdStr.c_str(), len);

        PacketCommand cmd{}; // packetType = CMD_COMMAND_STRING
        cmd.payloadLen = len;

        // Передаём стабильный буфер
        loraComm->sendPacketBase(BOAT_DEVICE_ID, cmd, tempBuf);
        
        // Увеличиваем счётчик отправленных пакетов
        if (hasValidBoatStatus) {
            lastBoatStatus.lora.packetsSent++;
        }
    }
    void sendHeartbeatPing()
    {
        sendPingToBoat();
        nextPingInterval = random(10000UL, 15001UL);
        lastPingHeartbeat = millis();
    }
    
    void sendPingToBoat()
    {
        PacketPing ping{}; // packetType = CMD_PING
        loraComm->sendPacketBase(BOAT_DEVICE_ID, ping, nullptr, false);
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
        Serial.println("║ 📦 Structured Data Commands (BS:/DM:) - BoatSettings API      ║");
        Serial.println("║   BS:H / DM:H  - Request structured heartbeat (basic status)   ║");
        Serial.println("║   BS:F / DM:F  - Request full structured status                ║");
        Serial.println("║   DM:G / DM:1  - Request GPS data only                         ║");
        Serial.println("║   DM:M / DM:2  - Request motor status only                     ║");
        Serial.println("║   DM:SENS/DM:3 - Request sensor data only (temp, battery)      ║");
        Serial.println("║   DM:L / DM:5  - Request LoRa status only                      ║");
        Serial.println("║   DM:N / DM:4  - Request navigation status only                ║");
        Serial.println("║   DM:SYS/DM:6  - Request system info (uptime, memory, health)  ║");
        Serial.println("║   BS:ON/DM:ON  - Enable structured data mode                   ║");
        Serial.println("║   BS:OFF/DM:OFF- Disable structured data mode (use JSON)       ║");
        Serial.println("║   BS:PING      - Send manual ping to boat                      ║");
        Serial.println("║   BS:RSSI      - Request RSSI report from boat                 ║");
        Serial.println("║                                                                  ║");
        Serial.println("║ 💡 Note: Uses legacy letter commands (G,M,SENS,L,N,SYS) for   ║");
        Serial.println("║          compatibility with current boat firmware              ║");
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
        Serial.println("║   boatstatus- Show detailed boat status (from structured data) ║");
        Serial.println("║   requestboat- Request full boat status from boat              ║");
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
        
        addLog("\n[MC] 🚀 Processing command: " + cmd);
        
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
            // Oil pump command P:0-P:100
            String param = cmd.substring(2);
            int pumpPower = param.toInt();
            if (pumpPower >= 0 && pumpPower <= 100) {
                sendOilPumpCommand(pumpPower);
            } else {
                addLog("[MC] ❌ Invalid pump power: " + param + " (must be 0-100)");
            }
        }
        else if (cmd == "P") {
            sendPingCommand();
        }
        else if (cmd.startsWith("BS:") || cmd.startsWith("DM:")) {
            // 📦 NEW: Structured Data Commands for BoatSettings (BS:) and DataMode (DM:)
            // Both prefixes are supported for compatibility
            String param;
            if (cmd.startsWith("BS:")) {
                param = cmd.substring(3);
            } else {
                param = cmd.substring(3);
            }
            
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
            else if (param == "G" || param == "1") {
                // Request GPS data only (G for legacy, 1 for new format)
                sendCommandString("DM:G");
                addLog("[MC] 📦 Requested GPS data");
            }
            else if (param == "M" || param == "2") {
                // Request motor status only (M for legacy, 2 for new format)
                sendCommandString("DM:M");
                addLog("[MC] 📦 Requested motor status");
            }
            else if (param == "S" || param == "3") {
                // Request sensor data only (S for legacy, 3 for new format)
                sendCommandString("DM:SENS");
                addLog("[MC] 📦 Requested sensor data");
            }
            else if (param == "L" || param == "5") {
                // Request LoRa status only (L for legacy, 5 for new format)
                sendCommandString("DM:L");
                addLog("[MC] 📦 Requested LoRa status");
            }
            else if (param == "N" || param == "4") {
                // Request navigation status only (N for legacy, 4 for new format)
                sendCommandString("DM:N");
                addLog("[MC] 📦 Requested navigation status");
            }
            else if (param == "SYS" || param == "6") {
                // Request system info (SYS for legacy, 6 for new format)
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
                addLog("[MC] ❌ Unknown structured data command: " + param + ". Available: H,F,G/1,M/2,SENS/3,L/5,N/4,SYS/6,ON,OFF,PING,RSSI");
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
        else if (cmd == "boatstatus" || cmd == "BOATSTATUS" || cmd == "bs") {
            printBoatStatus();
        }
        else if (cmd == "requestboat" || cmd == "rb") {
            // Request full boat status
            sendCommandString("DM:F");
            vTaskDelay(pdMS_TO_TICKS(500)); // Уменьшено с 2000 до 500ms
            sendCommandString("DM:SYS");
            vTaskDelay(pdMS_TO_TICKS(500)); // Уменьшено с 2000 до 500ms
            sendCommandString("DM:G");
            addLog("[MC] 📦 Requested full boat status - wait a moment then use 'boatstatus'");
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
            addLog("\n\n[MC] Processing command: demo");
            addLog("[MC] Running demo commands...");
            requestFullDiagnostics();
            addLog("[MC] 🔧 Sent diagnostic command: D:full");
            vTaskDelay(pdMS_TO_TICKS(500));
            requestLoRaStatus();
            addLog("[MC] 📡 Sent LoRa command: L:status");
            sendNavigationCommand("status");
            addLog("[MC] Sent navigation command: N:status");
            sendPingCommand();
            addLog("[MC] Ping sent\n\n");
            vTaskDelay(pdMS_TO_TICKS(500));
            sendInfoRequest(CMD_TELEMETRY_FRAGMENT); // запросить телеметрию:
            addLog("[MC] 📊 Requested telemetry fragment");
            sendInfoRequest(CMD_INFO_ENGINE);        // запросить “InfoEngine”:
            addLog("[MC] 🔧 Requested engine info");
            sendInfoRequest(CMD_STATUS);             // запросить общий статус:
            addLog("[MC] ✅ Requested general status");
            vTaskDelay(pdMS_TO_TICKS(500));
            addLog("[MC] ✅ Demo complete");
            vTaskDelay(pdMS_TO_TICKS(500));
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
            if(input == "") {
                addLog("[MC] Ignoring empty input");
                return; // Ignore empty input
            }
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

    // 🚀 NEW: Full boat status display
    void printBoatStatus()
    {
        if (!hasValidBoatStatus) {
            Serial.println("\n╔══════════════════════════════════════════════════════════════════╗");
            Serial.println("║                         BOAT STATUS                              ║");
            Serial.println("╠══════════════════════════════════════════════════════════════════╣");
            Serial.println("║ No valid boat status data available                             ║");
            Serial.println("║ Try sending 'DM:F' or 'BS:F' to request full status             ║");
            Serial.println("╚══════════════════════════════════════════════════════════════════╝");
            return;
        }

        unsigned long timeSinceUpdate = millis() - lastBoatStatusUpdate;
        bool dataFresh = timeSinceUpdate < 30000; // 30 секунд

        Serial.println("\n╔══════════════════════════════════════════════════════════════════╗");
        Serial.println("║                         BOAT STATUS                              ║");
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        Serial.println("║ Data Age: " + String(timeSinceUpdate/1000) + "s " + String(dataFresh ? "(FRESH)" : "(OLD)") + String("                                    ").substring(0, 47 - String(timeSinceUpdate/1000).length()) + "║");
        Serial.println("║ Timestamp: " + String(lastBoatStatus.timestamp) + String("                                     ").substring(0, 45 - String(lastBoatStatus.timestamp).length()) + "║");
        Serial.println("║ Version: " + String(lastBoatStatus.version) + String("                                           ").substring(0, 51 - String(lastBoatStatus.version).length()) + "║");
        Serial.println("╠══════════════════════════════════════════════════════════════════╣");
        
        // GPS Status
        Serial.println("║ GPS:                                                             ║");
        Serial.println("║   Position: " + String(lastBoatStatus.gps.position.latitude, 6) + "," + String(lastBoatStatus.gps.position.longitude, 6) + String("                       ").substring(0, 37 - String(lastBoatStatus.gps.position.latitude, 6).length() - String(lastBoatStatus.gps.position.longitude, 6).length()) + "║");
        Serial.println("║   Satellites: " + String(lastBoatStatus.gps.satelliteCount) + ", Fix: " + String(lastBoatStatus.gps.hasFix ? "YES" : "NO") + String("                                ").substring(0, 41 - String(lastBoatStatus.gps.satelliteCount).length()) + "║");
        
        // Motor Status  
        Serial.println("║ Motors:                                                          ║");
        String motorStateStr[] = {"STOP", "FWD", "REV"};
        Serial.println("║   State: " + motorStateStr[lastBoatStatus.motors.state] + ", L:" + String(lastBoatStatus.motors.leftPower) + ", R:" + String(lastBoatStatus.motors.rightPower) + String("                            ").substring(0, 38 - motorStateStr[lastBoatStatus.motors.state].length() - String(lastBoatStatus.motors.leftPower).length() - String(lastBoatStatus.motors.rightPower).length()) + "║");
        Serial.println("║   Rudder: " + String(lastBoatStatus.motors.rudderAngle) + "°, Limit: " + String(lastBoatStatus.motors.throttleLimit) + "%" + String("                              ").substring(0, 36 - String(lastBoatStatus.motors.rudderAngle).length() - String(lastBoatStatus.motors.throttleLimit).length()) + "║");
        Serial.println("║   Emergency Stop: " + String(lastBoatStatus.motors.emergencyStop ? "YES" : "NO") + String("                                    ").substring(0, 47 - String(lastBoatStatus.motors.emergencyStop ? "YES" : "NO").length()) + "║");
        
        // LoRa Status
        Serial.println("║ LoRa:                                                            ║");
        Serial.println("║   Profile: " + String(lastBoatStatus.lora.currentProfile) + ", RSSI: " + String(lastBoatStatus.lora.rssi, 1) + "dBm" + String("                            ").substring(0, 33 - String(lastBoatStatus.lora.currentProfile).length() - String(lastBoatStatus.lora.rssi, 1).length()) + "║");
        Serial.println("║   SNR: " + String(lastBoatStatus.lora.snr, 1) + "dB, Adaptive: " + String(lastBoatStatus.lora.adaptiveMode ? "ON" : "OFF") + String("                             ").substring(0, 36 - String(lastBoatStatus.lora.snr, 1).length() - String(lastBoatStatus.lora.adaptiveMode ? "ON" : "OFF").length()) + "║");
        Serial.println("║   MC Connected: " + String(lastBoatStatus.lora.missionControlConnected ? "YES" : "NO") + String("                                  ").substring(0, 44 - String(lastBoatStatus.lora.missionControlConnected ? "YES" : "NO").length()) + "║");
        Serial.println("║   Packets RX/TX: " + String(lastBoatStatus.lora.packetsReceived) + "/" + String(lastBoatStatus.lora.packetsSent) + String("                                ").substring(0, 42 - String(lastBoatStatus.lora.packetsReceived).length() - String(lastBoatStatus.lora.packetsSent).length()) + "║");
        
        // Sensor Status
        Serial.println("║ Sensors:                                                         ║");
        Serial.println("║   Motors: " + String(lastBoatStatus.sensors.motor1Temp, 1) + "°C / " + String(lastBoatStatus.sensors.motor2Temp, 1) + "°C" + String("                              ").substring(0, 36 - String(lastBoatStatus.sensors.motor1Temp, 1).length() - String(lastBoatStatus.sensors.motor2Temp, 1).length()) + "║");
        Serial.println("║   Battery: " + String(lastBoatStatus.sensors.batteryVoltage, 1) + "V / " + String(lastBoatStatus.sensors.batteryPercent) + "%" + String("                             ").substring(0, 37 - String(lastBoatStatus.sensors.batteryVoltage, 1).length() - String(lastBoatStatus.sensors.batteryPercent).length()) + "║");
        Serial.println("║   Current: " + String(lastBoatStatus.sensors.batteryCurrent, 1) + "A" + String("                                      ").substring(0, 49 - String(lastBoatStatus.sensors.batteryCurrent, 1).length()) + "║");
        Serial.println("║   Warnings: Volt:" + String(lastBoatStatus.sensors.lowVoltageWarning ? "YES" : "NO") + ", Temp:" + String(lastBoatStatus.sensors.overtemperatureWarning ? "YES" : "NO") + String("                     ").substring(0, 31 - String(lastBoatStatus.sensors.lowVoltageWarning ? "YES" : "NO").length() - String(lastBoatStatus.sensors.overtemperatureWarning ? "YES" : "NO").length()) + "║");
        
        // Navigation Status
        Serial.println("║ Navigation:                                                      ║");
        String navModeStr[] = {"MANUAL", "WAYPOINT", "RTH", "STATION"};
        Serial.println("║   Mode: " + navModeStr[lastBoatStatus.navigation.mode] + ", Active: " + String(lastBoatStatus.navigation.navigationActive ? "YES" : "NO") + String("                        ").substring(0, 34 - navModeStr[lastBoatStatus.navigation.mode].length() - String(lastBoatStatus.navigation.navigationActive ? "YES" : "NO").length()) + "║");
        Serial.println("║   Waypoints: " + String(lastBoatStatus.navigation.currentWaypoint) + "/" + String(lastBoatStatus.navigation.totalWaypoints) + String("                                       ").substring(0, 46 - String(lastBoatStatus.navigation.currentWaypoint).length() - String(lastBoatStatus.navigation.totalWaypoints).length()) + "║");
        Serial.println("║   Distance to target: " + String(lastBoatStatus.navigation.distanceToTarget, 1) + "m" + String("                            ").substring(0, 38 - String(lastBoatStatus.navigation.distanceToTarget, 1).length()) + "║");
        
        // System Info
        Serial.println("║ System:                                                          ║");
        Serial.println("║   Uptime: " + String(lastBoatStatus.uptime) + "s, Health: " + String(lastBoatStatus.systemHealth) + "%" + String("                         ").substring(0, 35 - String(lastBoatStatus.uptime).length() - String(lastBoatStatus.systemHealth).length()) + "║");
        Serial.println("║   Free Heap: " + String(lastBoatStatus.freeHeap) + " bytes" + String("                                ").substring(0, 42 - String(lastBoatStatus.freeHeap).length()) + "║");
        Serial.println("║   Update Mode: " + String(lastBoatStatus.firmwareUpdateMode ? "YES" : "NO") + String("                                     ").substring(0, 46 - String(lastBoatStatus.firmwareUpdateMode ? "YES" : "NO").length()) + "║");
        
        Serial.println("╚══════════════════════════════════════════════════════════════════╝");
    }

private:
    LoRaCore *loraComm;
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
    
    // 🚀 NEW: Full boat status from structured data
    BoatSettings lastBoatStatus;    // Полное состояние лодки
    unsigned long lastBoatStatusUpdate = 0; // Время последнего обновления
    bool hasValidBoatStatus = false; // Есть ли валидные данные
    
    // 🚀 NEW: Active connection monitoring with double ping strategy
    unsigned long lastPingSent = 0;
    bool waitingForPong = false;
    bool firstPingSent = false;   // 🚀 NEW: Отслеживаем первый пинг на 35 сек
    bool secondPingSent = false;  // 🚀 NEW: Отслеживаем второй пинг на 45 сек
    static constexpr unsigned long FIRST_PING_INTERVAL = 35000;  // 35 секунд → первый пинг
    static constexpr unsigned long SECOND_PING_INTERVAL = 45000; // 45 секунд → второй пинг
    static constexpr unsigned long PONG_TIMEOUT = 10000; // 10 секунд ждем понг

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
                //addLog("[MC] 📦 Starting to receive " + String(total) + " command response fragments");
            }
            
            responseBuffer += chunk;
            
            //addLog("[MC] 📥 Fragment " + String(idx + 1) + "/" + String(total) + " received (" + String(chunk.length()) + " bytes)");

            if (idx == expectedResponseFragments - 1) {
                //addLog("[MC] ✅ Complete command response received (" + String(responseBuffer.length()) + " bytes total)");
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
                float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();
        if (sender == BOAT_DEVICE_ID) {
            lastBoatActivity = millis();        // Обновляем время активности лодки при получении ЛЮБОГО пакета от неё
            boatIsActive = true;
            
            
            if (firstPingSent || secondPingSent) { //Сбрасываем флаги пингов при любой активности лодки
                addLog("[MC] 🚤 Boat activity detected - resetting ping state");
                firstPingSent = false;
                secondPingSent = false;
                waitingForPong = false;
            }
            
            // 🚀 FIX: Обновляем только счетчик пакетов при получении от лодки
            if (hasValidBoatStatus) {
                lastBoatStatus.lora.packetsReceived++;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
            }
            
            // Сохраняем RSSI для legacy использования
            lastRSSI = rssi;
        }
    
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
                // 🚀 NEW: Проверяем RSSI пульта - игнорируем при плохом сигнале
                float currentRssi = loraComm->getRadio().getRSSI();
                if (currentRssi < -120.0f) {
                    addLog("2 [MC] 🚫 Ignoring CMD_REQUEST_ASA - RSSI too low: " + String(currentRssi, 1) + "dBm");
                    break; // Игнорируем запрос при плохом RSSI
                }
                
                vTaskDelay(pdMS_TO_TICKS(10)); 
                sendAsaResponse(loraComm, profileIndex, sender);
                addLog("3 [MC] ASA Response sent to LORA");
                vTaskDelay(pdMS_TO_TICKS(900)); 
            }
            else
            {
                addLog("4 [MC] ASA Response received, applying profile index " + String(profileIndex));
            }
            uint8_t loopCounter = 0;
            while ((loraComm->getOutgoingQueueCount() > 0 || loraComm->getPendingCount() > 0) && loopCounter++ < 5) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            applyProfile(profileIndex);
            break;
        }

        case CMD_PING:
        {
            PacketPong pong; // конструктор автоматически установит packetType = CMD_PONG
            loraComm->sendPacketBase(sender, pong, nullptr, false);
            addLog("[MC] Got PING, sent PONG response");
            break;
        }
        
        case CMD_PONG:
        {
            if (waitingForPong) {
                unsigned long pingTime = millis() - lastPingSent;
                String pingType = secondPingSent ? "second" : "first";
                waitingForPong = false;
                boatIsActive = true;
                lastBoatActivity = millis();
                firstPingSent = false;
                secondPingSent = false;
            }
            break;
        }
        case CMD_RSSI_REPORT:
        {
            PacketRssiReport rpt{}; // конструктор автоматически установит packetType = CMD_RSSI_REPORT
            rpt.packetId = hdr.packetId;
            rpt.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&rpt) + sizeof(PacketBase), buf, hdr.payloadLen);
            addLog("---- Boat RSSI:raw=" + String(rpt.rawRssi)+" ,smoothed=" + String(rpt.smoothedRssi));
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
        case CMD_STRUCTURED_HEARTBEAT:// 🚀 NEW: Structured data packet handling
        {
            UltraCompactHeartbeat hb;
            if (StructuredDataManager::parseHeartbeat(buf, hdr.payloadLen, hb)) {
                lastPacketReceived = millis();      // Обновляем последнее время получения от лодки
                boatIsActive = true;
                
                // Отображаем компактную информацию
                addLog("[MC][CMD_STRUCTURED_HEARTBEAT] Boat HB: " + String(hb.latitude, 6) + "," + String(hb.longitude, 6) + 
                       " P:" + String(hb.loraProfile) + 
                       " RSSI:" + String(hb.rssi) + 
                       " Bat:" + String(hb.batteryPercent) + "%");
                       
                // 🚀 NEW: Update full boat status from heartbeat
                lastBoatStatus.gps.position.latitude = hb.latitude;
                lastBoatStatus.gps.position.longitude = hb.longitude;
                lastBoatStatus.gps.position.timestamp = millis();
                lastBoatStatus.gps.hasFix = true; // Предполагаем, что если есть данные, то есть фикс
                lastBoatStatus.lora.currentProfile = hb.loraProfile;
                lastBoatStatus.lora.rssi = hb.rssi;
                lastBoatStatus.sensors.batteryPercent = hb.batteryPercent;
                lastBoatStatus.systemHealth = hb.systemHealth;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
                       
                lastKnownBoatPosition.latitude = hb.latitude;   // Сохраняем состояние лодки (если нужно)
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
                addLog("[MC][CMD_STRUCTURED_GPS] GPS: " + gps.toString());
                
                // 🚀 NEW: Update boat status GPS data
                lastBoatStatus.gps = gps;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
                
                // 🚀 FIX: Обновляем кэшированную позицию
                lastKnownBoatPosition = gps.position;
            } else {
                addLog("[MC] ❌ Failed to parse GPS data");
            }
            break;
        }
        case CMD_STRUCTURED_MOTORS:
        {
            addLog("[MC][CMD_STRUCTURED_MOTORS] Motor data received (" + String(hdr.payloadLen) + " bytes)");
            MotorStatus motors;
            if (StructuredDataManager::parseMotors(buf, hdr.payloadLen, motors)) {
                addLog("[MC] ⚙️ Motors: State=" + String(motors.state) + ", L=" + String(motors.leftPower) + 
                       ", R=" + String(motors.rightPower) + ", Rudder=" + String(motors.rudderAngle) + "°");
                lastBoatStatus.motors = motors;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
            } else {
                addLog("[MC] ❌ Failed to parse motor data");
            }
            break;
        }
        case CMD_STRUCTURED_SENSORS:
        {
            addLog("[MC][CMD_STRUCTURED_SENSORS] Sensor data received (" + String(hdr.payloadLen) + " bytes)");
            SensorStatus sensors;
            if (StructuredDataManager::parseSensors(buf, hdr.payloadLen, sensors)) {
                addLog("[MC] 🌡️ Sensors: Bat=" + String(sensors.batteryVoltage, 1) + "V/" + String(sensors.batteryPercent) + 
                       "%, Temp=" + String(sensors.motor1Temp, 1) + "°C/" + String(sensors.motor2Temp, 1) + "°C");
                lastBoatStatus.sensors = sensors;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
                
                // 🚀 FIX: Обновляем кэшированные значения
                lastBatteryPercent = sensors.batteryPercent;
            } else {
                addLog("[MC] ❌ Failed to parse sensor data");
            }
            break;
        }
        case CMD_STRUCTURED_NAVIGATION:
        {
            addLog("[MC][CMD_STRUCTURED_NAVIGATION] Navigation data received (" + String(hdr.payloadLen) + " bytes)");
            NavigationStatus navigation;
            if (StructuredDataManager::parseNavigation(buf, hdr.payloadLen, navigation)) {
                String modeStr[] = {"MANUAL", "WAYPOINT", "RTH", "STATION"};
                addLog("[MC] 🧭 Navigation: Mode=" + modeStr[navigation.mode] + 
                       ", Active=" + String(navigation.navigationActive ? "YES" : "NO") + 
                       ", Distance=" + String(navigation.distanceToTarget, 1) + "m");
                lastBoatStatus.navigation = navigation;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
            } else {
                addLog("[MC] ❌ Failed to parse navigation data");
            }
            break;
        }
        case CMD_STRUCTURED_LORA:
        {
            addLog("[MC][CMD_STRUCTURED_LORA] LoRa data received (" + String(hdr.payloadLen) + " bytes)");
            LoRaStatus lora;
            if (StructuredDataManager::parseLoRa(buf, hdr.payloadLen, lora)) {
                addLog("[MC] 📡 LoRa: Profile=" + String(lora.currentProfile) + 
                       ", RSSI=" + String(lora.rssi, 1) + "dBm, SNR=" + String(lora.snr, 1) + "dB");
                lastBoatStatus.lora = lora;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
            } else {
                addLog("[MC] ❌ Failed to parse LoRa data");
            }
            break;
        }
        case CMD_STRUCTURED_SYSTEM:
        {
            addLog("[MC][CMD_STRUCTURED_SYSTEM] System data received (" + String(hdr.payloadLen) + " bytes)");
            SystemInfo system;
            if (StructuredDataManager::parseSystem(buf, hdr.payloadLen, system)) {
                addLog("[MC] 🖥️ System: Uptime=" + String(system.uptime) + "s" + 
                       ", Free=" + String(system.freeHeap) + " bytes");
                lastBoatStatus.uptime = system.uptime;
                lastBoatStatus.freeHeap = system.freeHeap;
                lastBoatStatus.firmwareUpdateMode = system.firmwareUpdateMode;
                lastBoatStatus.updateTimestamp();
                lastBoatStatusUpdate = millis();
                hasValidBoatStatus = true;
            } else {
                addLog("[MC] ❌ Failed to parse system data");
            }
            break;
        }
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
            loraComm->addAckToBulk(hdr.packetId, BOAT_DEVICE_ID); // Используем новую централизованную систему bulk ACK
        }
    }
    // MissionControl.hpp
    void sendInfoRequest(CommandType what)
    {
        // что именно спрашиваем: T/I/S/F/G/D/K…
        PacketRequestInfo cmd{}; // конструктор автоматически установит packetType = CMD_REQUEST_INFO
        cmd.requestType = what;
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

};
