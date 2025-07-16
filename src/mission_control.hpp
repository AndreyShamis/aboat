// mission_control.hpp
#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "settings.h"
#include "LoRaCore.hpp"
#include "wifi_manager.hpp"

#include "PacketAsaExchange.hpp"

// -----------------------------------------------------------------------------
// MissionControl
//
// Handles sending commands and telemetry requests over LoRa,
// processes incoming fragments, ASA speed‐adaptation handshake, and ACKs.
// -----------------------------------------------------------------------------
class MissionControl : public LogInterface
{
public:
    unsigned long lastPongHeartbeat = 0;    // время последней посылки
    unsigned long nextPongInterval = 15000; // первый интервал (мс)
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

        LoRaPacket pkt;

        if (loraComm->receive(pkt))
        {
            PacketBase hdr;
            hdr.packetType = pkt.packetType;
            hdr.packetId = pkt.packetId;
            hdr.payloadLen = pkt.payloadLen;
            if (pkt.receiverId == MISSION_CONTROL_ID)
            {
                // addLog("[" + String(currentProfileIndex) + "]  Received LoRaPacket: sender=" + String(pkt.senderId) +
                //        ", type=" + String((char)pkt.packetType) +
                //        ", id=" + String(pkt.packetId) +
                //        ", len=" + String(pkt.payloadLen));
                handlePacket(pkt.senderId, hdr, pkt.payload);
            }
        }

        // 🕒 Проверка потери пинга
        if (millis() - lastPingTime > checkInterval && currentProfileIndex > 0)
        {
            currentProfileIndex = 0;
            const auto &profile = loraProfiles[currentProfileIndex];

            addLog("⚠️ No ping in " + String(checkInterval / 1000) + " sec. Degrading LoRa profile to index " + String(currentProfileIndex) +
                   " (SF=" + profile.spreadingFactor + ", BW=" + profile.bandwidth + ", CR=" + profile.codingRate + ")");
            PacketAsaApprove resp;
            resp.packetType = CMD_REPOSNCE_ASA;
            resp.packetId = nextPacketId++;
            resp.payloadLen = sizeof(uint8_t);
            resp.profileIndex = currentProfileIndex;
            loraComm->sendPacketBase(BOAT_DEVICE_ID, resp, (const uint8_t*)&resp.profileIndex, false);
            vTaskDelay(pdMS_TO_TICKS(500));
            applyProfile(currentProfileIndex);

            lastPingTime = millis(); // Сбросим таймер
        }
        else
        {
            if (millis() - lastPongHeartbeat >= nextPongInterval)
            {
                sendHeartbeatPong();
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
    void sendHeartbeatPong()
    {
        PacketCommand pong{};
        pong.packetType = CMD_PONG;
        pong.packetId = nextPacketId++;
        pong.payloadLen = 0;

        loraComm->sendPacketBase(BOAT_DEVICE_ID, pong, nullptr, false);
        addLog(F("[MC] 🔄 Heartbeat PONG sent"));

        // cформируем новый случайный интервал 10–20 с
        nextPongInterval = random(10000UL, 20001UL);
        lastPongHeartbeat = millis();
    }

    void addLog(const String &msg) override
    {
        Serial.print('[');
        Serial.print(timeStr());
        Serial.print("] ");
        Serial.println(msg);
    }

    void applyProfile(uint8_t idx)
    {
        if (idx >= LORA_PROFILE_COUNT) {
            addLog("❌ Недопустимый индекс профиля: " + String(idx));
            return;
        }
        
        const auto& profile = loraProfiles[idx];
        
        // Используем новый метод LoRaCore для применения профилей из settings.h
        if (loraComm->applyProfileFromSettings(idx)) {
            currentProfileIndex = idx;
            
            String modeStr = (profile.mode == RadioProfileMode::FSK) ? "GFSK" : "LoRa";
            
            if (profile.mode == RadioProfileMode::LORA) {
                addLog("⚙️ MC: Применён " + modeStr + " профиль " + String(idx) + 
                       " (SF=" + String(profile.spreadingFactor) + 
                       ", CR=" + String(profile.codingRate) + 
                       ", BW=" + String(profile.bandwidth, 1) + "kHz)");
            } else {
                addLog("⚙️ MC: Применён " + modeStr + " профиль " + String(idx) + 
                       " (Bitrate=" + String(profile.bitrate) + 
                       ", Dev=" + String(profile.deviation) + 
                       ", RxBW=" + String(profile.bandwidth, 1) + "kHz)");
            }
        } else {
            addLog("❌ MC: Ошибка применения профиля " + String(idx));
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
        addLog("[MC] 🔧 Sent diagnostic command: " + cmdStr);
    }

    // Send LoRa command (L) to boat
    void sendLoRaCommand(const String &param = "")
    {
        String cmdStr = "L";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[MC] 📡 Sent LoRa command: " + cmdStr);
    }

    // Send navigation command (N) to boat
    void sendNavigationCommand(const String &param = "")
    {
        String cmdStr = "N";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[MC] 🧭 Sent navigation command: " + cmdStr);
    }

    // Send web interface command (W) to boat
    void sendWebCommand(const String &param = "")
    {
        String cmdStr = "W";
        if (param.length() > 0) {
            cmdStr += ":" + param;
        }
        sendCommandString(cmdStr);
        addLog("[MC] 🌐 Sent web command: " + cmdStr);
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
        addLog("[MC] ⚙️ Sent engine command: " + cmdStr);
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
        Serial.println("║ ⚙️ Legacy Commands:                                            ║");
        Serial.println("║   M:120     - Set motor power                                   ║");
        Serial.println("║   E         - Emergency stop                                    ║");
        Serial.println("║   R         - Request telemetry                                ║");
        Serial.println("║   P         - Send ping                                        ║");
        Serial.println("║   P:1-P:100 - Oil pump control (1-100% power)                  ║");
        Serial.println("║   SCAN      - Spectrum scan (CSV output)                       ║");
        Serial.println("║   profiles  - Show all available radio profiles                ║");
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
        else if (cmd == "help" || cmd == "HELP" || cmd == "h" || cmd == "H") {
            printHelp();
        }
        else if (cmd == "profiles" || cmd == "PROFILES") {
            printProfileInfo();
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
            vTaskDelay(pdMS_TO_TICKS(500)); // Уменьшено с 2000 до 500ms
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

private:
    LoRaCore *loraComm;
    PacketId_t nextPacketId = 0;
    unsigned long lastPingTime = 0;
    int currentProfileIndex = 0;
    unsigned long checkInterval = 35 * 1000;

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
        } else {
            addLog("[MC] ❓ Unknown response format: " + response);
        }
    }
    // Handle incoming PacketBase + payloadBuf
    void handlePacket(uint8_t sender,
                      const PacketBase &hdr,
                      const uint8_t *buf)
    {
        lastPingTime = millis(); // Обновляем время последнего пинга
        float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();
        addLog("profile:" + String(currentProfileIndex) + " .RSSI:" + String(rssi) + "dBm, SNR=" + String(snr, 1) + "dB");
        switch (hdr.packetType)
        {
            case CMD_BULK_ACK:
            {
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
            // Ответим только на CMD_REQUEST_ASA
            if (hdr.packetType == CMD_REQUEST_ASA)
            {
                sendAsaResponse(loraComm, nextPacketId++, profileIndex, sender);
                addLog("3 [MC] ASA Response sent");
                vTaskDelay(pdMS_TO_TICKS(50)); 
            }
            else
            {
                addLog("4 [MC] ASA Response received, applying profile index " + String(profileIndex));
            }
            if(loraComm->getOutgoingQueueCount() > 0) {
                addLog("[MC] Outgoing queue count: " + String(loraComm->getOutgoingQueueCount()));
                vTaskDelay(pdMS_TO_TICKS(409)); // Ждем, чтобы пакет ушел
            }
            if(loraComm->getOutgoingQueueCount() > 0) {
                addLog("[MC] Outgoing queue count: " + String(loraComm->getOutgoingQueueCount()));
                vTaskDelay(pdMS_TO_TICKS(209)); // Ждем, чтобы пакет ушел
            }
            if(loraComm->getOutgoingQueueCount() > 0) {
                addLog("[MC] Outgoing queue count: " + String(loraComm->getOutgoingQueueCount()));
                vTaskDelay(pdMS_TO_TICKS(207)); // Ждем, чтобы пакет ушел
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
            addLog("Pong->BOAT");
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

        default:
            // Unhandled packet types …
            addLog("[MC] Unhandled packet type: [" + String((char)hdr.packetType) +
                   "][" + String((int)hdr.packetType) + "], sender=" + String(sender) +
                   ", payloadLen=" + String(hdr.payloadLen));
            break;
        }

        if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_BULK_ACK && hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG && hdr.packetType != CMD_REQUEST_ASA)
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
            pendingBulkAck.addAck(packetId);
            addLog("[MC] 📦 Started new bulk ACK with packet " + String(packetId));
        }
    }
    
    // Отправить накопленные ACK
    void sendBulkAck()
    {
        if (pendingBulkAck.isEmpty()) return;
        
        pendingBulkAck.packetId = nextPacketId++;
        
        // Формируем payload: count + массив ID
        uint8_t payload[1 + 10 * sizeof(PacketId_t)];
        payload[0] = pendingBulkAck.count;
        memcpy(&payload[1], pendingBulkAck.ackedIds, pendingBulkAck.count * sizeof(PacketId_t));
        
        size_t payloadSize = sizeof(uint8_t) + (pendingBulkAck.count * sizeof(PacketId_t));
        
        // Отправляем как высокоприоритетный пакет
        LoRaPacket bulkPacket;
        packBaseIntoLoRa(bulkPacket, MY_DEVICE_ID, BOAT_DEVICE_ID, pendingBulkAck, payload);
        
        if (loraComm->sendHighPriority(bulkPacket)) {
            addLog("[MC] ✅ Sent BULK ACK for " + String(pendingBulkAck.count) + " packets (high priority)");
        } else {
            // Если высокоприоритетная отправка не удалась, используем обычную
            loraComm->sendPacketBase(BOAT_DEVICE_ID, pendingBulkAck, payload, false);
            addLog("[MC] ✅ Sent BULK ACK for " + String(pendingBulkAck.count) + " packets (standard)");
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
