// BoatCommandHandler.cpp
#include "BoatCommandHandler.hpp"
#include "Boat.hpp"  // Include full Boat definition
#include "settings.h"
#include "temp_sensors.hpp"

// Helper method implementations
void BoatCommandHandler::addLog(const String& message)
{
    if (boat) {
        boat->addLog(message);
    }
}

String BoatCommandHandler::timeStr()
{
    if (boat) {
        return boat->timeStr();
    }
    return "";
}

void BoatCommandHandler::sendStructuredHeartbeat()
{
    if (boat) {
        boat->sendStructuredHeartbeat();
    }
}

void BoatCommandHandler::sendStructuredFullStatus()
{
    if (boat) {
        boat->sendStructuredFullStatus();
    }
}

void BoatCommandHandler::sendStructuredGPS()
{
    if (boat) {
        boat->sendStructuredGPS();
    }
}

void BoatCommandHandler::sendStructuredMotors()
{
    if (boat) {
        boat->sendStructuredMotors();
    }
}

void BoatCommandHandler::sendStructuredSensors()
{
    if (boat) {
        boat->sendStructuredSensors();
    }
}

void BoatCommandHandler::sendStructuredLoRaStatus()
{
    if (boat) {
        boat->sendStructuredLoRaStatus();
    }
}

void BoatCommandHandler::sendStructuredNavigation()
{
    if (boat) {
        boat->sendStructuredNavigation();
    }
}

void BoatCommandHandler::sendStructuredSystemInfo()
{
    if (boat) {
        boat->sendStructuredSystemInfo();
    }
}

void BoatCommandHandler::adaptiveLoraUpdate()
{
    if (boat) {
        boat->adaptiveLoraUpdate();
    }
}

void BoatCommandHandler::applyProfile(uint8_t idx)
{
    if (boat) {
        boat->applyProfile(idx);
    }
}

// LoRa Profile Command Handler
String BoatCommandHandler::handleLoRaProfileCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "L:";
    if (arg == "S" || arg == "status") {
        String status = "Profile:" + String(boat->loraComm->getCurrentProfileIndex()) + 
               ",RSSI:" + String(boat->smoothedRssi) + "dBm" +
               ",Adaptive:" + String(boat->allowAdaptiveLoraSwitch ? "ON" : "OFF");
        
        // Add manual block status if active
        unsigned long timeSinceManual = millis() - boat->lastManualProfileChange;
        if (timeSinceManual < MANUAL_PROFILE_BLOCK_TIME) {
            unsigned long remainingBlockTime = (MANUAL_PROFILE_BLOCK_TIME - timeSinceManual) / 1000;
            status += ",ManualBlock:" + String(remainingBlockTime) + "s";
        }
        
        addLog("[LORA] " + status);
        response += status;
    } else if (arg.startsWith("P") || arg.startsWith("profile:")) {
        String profileStr = arg.startsWith("profile:") ? arg.substring(8) : arg.substring(1);
        int profileIndex = profileStr.toInt();
        response += applyLoRaProfile(profileIndex);
    } else if (arg == "adapt") {
        adaptiveLoraUpdate();
        response += "Adaptive LoRa triggered";
    } else if (arg.startsWith("A:")) {
        response += handleAdaptiveControl(arg.substring(2));
    } else if (arg == "unblock" || arg == "U") {
        boat->lastManualProfileChange = 0; // Reset manual block timer
        response += "Manual profile block reset, adaptive switching re-enabled";
    } else if (arg.length() > 0 && arg[0] >= '0' && arg[0] <= '9') {
        int profileIndex = arg.toInt();
        response += applyLoRaProfile(profileIndex);
    } else {
        response += "Usage: L:[S|P<n>|<n>|adapt|A:0/1|U] - Current: Profile:" + String(boat->loraComm->getCurrentProfileIndex()) + 
                   ",RSSI:" + String(boat->smoothedRssi) + "dBm";
    }
    return response;
}

String BoatCommandHandler::applyLoRaProfile(int profileIndex)
{
    if (!boat) return "Error: No boat reference";
    
    if (profileIndex >= 0 && profileIndex < LORA_PROFILE_COUNT) {
        boat->applyProfile(profileIndex);
        boat->lastManualProfileChange = millis(); // Block adaptive switching for some time
        addLog("[LORA] Manually switched to profile " + String(profileIndex) + " - adaptive switching blocked for " + String(MANUAL_PROFILE_BLOCK_TIME/1000) + "s");
        return "Profile changed to " + String(profileIndex);
    } else {
        return "Invalid profile index:" + String(profileIndex);
    }
}

String BoatCommandHandler::handleAdaptiveControl(const String& adaptiveStr)
{
    if (!boat) return "Error: No boat reference";
    
    if (adaptiveStr == "0") {
        boat->allowAdaptiveLoraSwitch = false;
        addLog("[LORA] Adaptive switching DISABLED by command");
        return "Adaptive switching disabled";
    } else if (adaptiveStr == "1") {
        boat->allowAdaptiveLoraSwitch = true;
        addLog("[LORA] Adaptive switching ENABLED by command");
        return "Adaptive switching enabled";
    } else {
        return "Invalid adaptive command (use A:0 or A:1)";
    }
}

// Structured Data Command Handler
String BoatCommandHandler::handleStructuredDataCommand(const String& command)
{
    if (!boat) return "Error: No boat reference";
    
    if (!boat->useStructuredData) {
        return "structured mode disabled";
    }
    
    if (command == "H" || command == "heartbeat") {
        sendStructuredHeartbeat();
        return "structured heartbeat sent";
    } else if (command == "F" || command == "full") {
        sendStructuredFullStatus();
        return "full structured status sent";
    } else if (command == "G" || command == "gps") {
        sendStructuredGPS();
        return "GPS data sent";
    } else if (command == "M" || command == "motors") {
        sendStructuredMotors();
        return "motor data sent";
    } else if (command == "SENS" || command == "sensors") {
        sendStructuredSensors();
        return "sensor data sent";
    } else if (command == "L" || command == "lora") {
        sendStructuredLoRaStatus();
        return "LoRa status sent";
    } else if (command == "N" || command == "navigation") {
        sendStructuredNavigation();
        return "navigation data sent";
    } else if (command == "SYS" || command == "system") {
        sendStructuredSystemInfo();
        return "system info sent";
    }
    
    return "unknown structured command";
}

// Diagnostic Command Handler
String BoatCommandHandler::handleDiagnosticCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "D:";
    if (arg == "S") {
        response += "Safety:" + boat->safetyMonitor.getStatusString();
    } else if (arg == "P") {
        response += "Performance:avg=" + String(boat->performanceMetrics.getAverageKeepTime()) + 
                   "ms,max=" + String(boat->performanceMetrics.maxKeepDuration) + "ms";
    } else if (arg == "T") {
        response += "Temps:M1=" + String(boat->temps.get(MOTOR1)) + 
                   "C,M2=" + String(boat->temps.get(MOTOR2)) + "C";
    } else if (arg == "R") {
        boat->performanceMetrics.reset();
        response += "Performance metrics reset";
    } else if (arg == "full" || arg.isEmpty()) {
        response += buildFullDiagnostic();
    } else if (arg == "extended" || arg == "ext") {
        response += buildExtendedDiagnostic();
    } else if (arg == "stack" || arg == "st") {
        response += "Stack Report:\n" + boat->stackMonitor.getReport();
    } else {
        response += "Unknown diagnostic command:" + arg;
    }
    return response;
}

String BoatCommandHandler::buildFullDiagnostic()
{
    if (!boat) return "Error: No boat reference";
    
    String safetyInfo = boat->safetyMonitor.getStatusString();
    if (safetyInfo.length() > 30) safetyInfo = safetyInfo.substring(0, 30) + "...";
    
    return "Safety:" + safetyInfo + 
           ";Perf:avg=" + String(boat->performanceMetrics.getAverageKeepTime()) + "ms" +
           ";Temps:M1=" + String(boat->temps.get(MOTOR1), 1) + "C,M2=" + String(boat->temps.get(MOTOR2), 1) + "C" +
           ";GPS:" + (boat->gnss.hasValidFix() ? "OK" : "NO_FIX") +
           ";Heap:" + String(ESP.getFreeHeap());
}

String BoatCommandHandler::buildExtendedDiagnostic()
{
    if (!boat) return "Error: No boat reference";
    
    String extDiag = "System:OK;";
    extDiag += "Uptime:" + String(millis()/1000) + "s;";
    extDiag += "FreeHeap:" + String(ESP.getFreeHeap()) + ";";
    extDiag += "MinFreeHeap:" + String(ESP.getFlashChipSize()) + ";";
    extDiag += "FlashSize:" + String(ESP.getFlashChipSize()) + ";";
    extDiag += "CPUFreq:" + String(ESP.getCpuFreqMHz()) + "MHz;";
    extDiag += "ChipModel:" + String(ESP.getChipModel()) + ";";
    extDiag += "TempSensors:" + String(boat->temps.getSensorCount()) + ";";
    extDiag += "WiFiStatus:" + String(WiFi.status()) + ";";
    extDiag += "LoRaProfile:" + String(boat->loraComm->getCurrentProfileIndex()) + ";";
    extDiag += "Safety:" + boat->safetyMonitor.getStatusString() + ";";
    extDiag += "Performance:avg=" + String(boat->performanceMetrics.getAverageKeepTime()) + "ms;";
    extDiag += "Stack:" + boat->stackMonitor.getCompactStatus();
    
    addLog("[DIAG] Extended diagnostic (" + String(extDiag.length()) + " bytes): " + extDiag);
    return extDiag;
}

// Navigation Command Handler
String BoatCommandHandler::handleNavigationCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "N:";
    if (arg == "M" || arg == "manual") {
        boat->autoNav.setMode(AutoNavigation::MANUAL);
        addLog("[NAV] Switched to manual mode");
        response += "Manual mode activated";
    } else if (arg == "R" || arg == "home") {
        boat->autoNav.setMode(AutoNavigation::RETURN_TO_HOME);
        addLog("[NAV] Return to home initiated");
        response += "Return to home mode activated";
    } else if (arg == "S" || arg == "station") {
        boat->autoNav.setMode(AutoNavigation::STATION_KEEPING);
        addLog("[NAV] Station keeping mode");
        response += "Station keeping mode activated";
    } else if (arg == "H" || arg == "sethome") {
        response += handleSetHome();
    } else if (arg.startsWith("start:")) {
        response += handleNavigationStart(arg.substring(6));
    } else if (arg == "stop") {
        boat->autoNav.setMode(AutoNavigation::MANUAL);
        response += "Navigation stopped";
    } else if (arg == "pause") {
        boat->autoNav.pause();
        response += "Navigation paused";
    } else if (arg == "resume") {
        boat->autoNav.resume();
        response += "Navigation resumed";
    } else if (arg.startsWith("mode:")) {
        response += handleNavigationMode(arg.substring(5));
    } else {
        String status = boat->autoNav.getStatusString();
        addLog("[NAV] Status: " + status);
        response += "Status:" + status;
    }
    return response;
}

String BoatCommandHandler::handleSetHome()
{
    if (!boat) return "Error: No boat reference";
    
    if (boat->gnss.hasValidFix()) {
        boat->autoNav.setHome(boat->gnss.getLatitude(), boat->gnss.getLongitude());
        String homePos = String(boat->gnss.getLatitude(), 6) + "," + String(boat->gnss.getLongitude(), 6);
        addLog("[NAV] Home position set: " + homePos);
        return "Home set at " + homePos;
    } else {
        addLog("[NAV] Cannot set home - no GPS fix");
        return "Error: No GPS fix available";
    }
}

String BoatCommandHandler::handleNavigationStart(const String& coords)
{
    if (!boat) return "Error: No boat reference";
    
    int commaIndex = coords.indexOf(',');
    if (commaIndex > 0) {
        float lat = coords.substring(0, commaIndex).toFloat();
        float lon = coords.substring(commaIndex + 1).toFloat();
        boat->autoNav.setTarget(lat, lon);
        boat->autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
        addLog("[NAV] Navigation started to: " + String(lat, 6) + "," + String(lon, 6));
        return "Navigation started to " + String(lat, 6) + "," + String(lon, 6);
    } else {
        return "Error: Invalid coordinates format";
    }
}

String BoatCommandHandler::handleNavigationMode(const String& mode)
{
    if (!boat) return "Error: No boat reference";
    
    if (mode == "manual") boat->autoNav.setMode(AutoNavigation::MANUAL);
    else if (mode == "waypoint") boat->autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
    else if (mode == "home") boat->autoNav.setMode(AutoNavigation::RETURN_TO_HOME);
    else if (mode == "station") boat->autoNav.setMode(AutoNavigation::STATION_KEEPING);
    return "Mode set to " + mode;
}

// Waypoint Command Handler
String BoatCommandHandler::handleWaypointCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "W:";
    if (arg.startsWith("A") || arg.startsWith("add:")) {
        String coords = arg.startsWith("add:") ? arg.substring(4) : arg.substring(1);
        int commaIndex = coords.indexOf(',');
        if (commaIndex > 0) {
            float lat = coords.substring(0, commaIndex).toFloat();
            float lon = coords.substring(commaIndex + 1).toFloat();
            boat->autoNav.addWaypoint(lat, lon);
            addLog("[NAV] Waypoint added: " + String(lat, 6) + "," + String(lon, 6));
            response += "Waypoint added:" + String(lat, 6) + "," + String(lon, 6);
        } else {
            response += "Error: Invalid waypoint format";
        }
    } else if (arg == "S" || arg == "start") {
        boat->autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
        addLog("[NAV] Waypoint following started");
        response += "Waypoint following started";
    } else if (arg == "C" || arg == "clear") {
        boat->autoNav.clearWaypoints();
        addLog("[NAV] Waypoints cleared");
        response += "Waypoints cleared";
    } else if (arg == "status") {
        response += "Web interface active,clients:" + String(WiFi.softAPgetStationNum());
    } else if (arg == "update") {
        response += "Web interface updated";
    } else {
        response += "Waypoint count:" + String(boat->autoNav.getWaypointCount());
    }
    return response;
}

// Time Command Handler
String BoatCommandHandler::handleTimeCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "T:";
    if (arg == "S" || arg == "sync") {
        if (boat->gnss.isTimeUpdated()) {
            boat->gnss.syncSystemTimeFromGPS();
            time_t now = time(nullptr);
            addLog("🕒 GPS time sync forced: " + String(now));
            response += "GPS time synced:" + String(now);
        } else {
            addLog("🕒 GPS time not available for sync");
            response += "GPS time not available";
        }
    } else if (arg == "status" || arg.isEmpty()) {
        time_t now = time(nullptr);
        bool synced = (now > 1600000000);
        String timeString = timeStr();
        String status = "Time:" + timeString + 
               ",Synced:" + String(synced ? "YES" : "NO") + 
               ",GPS_Updated:" + String(boat->gnss.isTimeUpdated() ? "YES" : "NO");
        addLog("🕒 " + status);
        response += status;
    } else if (arg == "reset") {
        struct timeval tv = {0, 0};
        settimeofday(&tv, NULL);
        addLog("🕒 System time reset");
        response += "System time reset";
    } else {
        response += "Unknown time command:" + arg;
    }
    return response;
}

// Data Mode Command Handler
String BoatCommandHandler::handleDataModeCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "DM:";
    if (arg == "S" || arg == "structured") {
        boat->useStructuredData = true;
        addLog("📦 Switched to structured data transmission");
        response += "structured data enabled";
    } else if (arg == "J" || arg == "json") {
        boat->useStructuredData = false;
        addLog("📄 Switched to JSON data transmission");
        response += "JSON data enabled";
    } else if (arg == "?") {
        response += String(boat->useStructuredData ? "structured" : "json") + 
                   ", HB:" + String((millis() - boat->lastHeartbeatSent)/1000) + "s ago" +
                   ", Full:" + String((millis() - boat->lastFullStatusSent)/1000) + "s ago";
    } else {
        // Handle structured data sub-commands
        String subResponse = handleStructuredDataCommand(arg);
        if (subResponse == "unknown structured command") {
            response += "Usage: DM:[S|J|H|F|G|M|SENS|L|N|SYS|?] (Structured/Json/Heartbeat/Full/Gps/Motors/Sensors/LoRa/Nav/System/status)";
        } else {
            response += subResponse;
        }
    }
    return response;
}

// IMU Command Handler
String BoatCommandHandler::handleIMUCommand(const String& arg)
{
    if (!boat) return "Error: No boat reference";
    
    String response = "I:";
    if (arg == "S" || arg == "status" || arg.isEmpty()) {
        response += boat->imu.getOrientationString();
        addLog("[IMU] " + boat->imu.getOrientationString());
    } else if (arg == "C" || arg == "calibrate") {
        if (boat->imu.isCalibrating()) {
            response += "calibration already in progress";
        } else {
            boat->imu.startGyroCalibration(1000);
            response += "gyro calibration started (1000 samples)";
            addLog("[IMU] 🔧 Gyro calibration started by command");
        }
    } else if (arg == "R" || arg == "reset") {
        boat->imu.reset();
        response += "AHRS filter reset";
        addLog("[IMU] 🔄 AHRS filter reset by command");
    } else if (arg == "D" || arg == "diag") {
        String diagInfo = boat->imu.getDiagnosticInfo();
        addLog("[IMU] " + diagInfo);
        response += diagInfo.substring(6); // Remove "[IMU] " prefix
    } else if (arg == "F" || arg == "force") {
        if (boat->imu.forceUpdate()) {
            response += "forced update completed: " + boat->imu.getOrientationString();
        } else {
            response += "forced update failed";
        }
    } else if (arg.startsWith("freq:") || arg.startsWith("F:")) {
        String freqStr = arg.startsWith("freq:") ? arg.substring(5) : arg.substring(2);
        int frequency = freqStr.toInt();
        if (frequency >= 10 && frequency <= 200) {
            unsigned long interval = 1000 / frequency;
            boat->imu.setUpdateInterval(interval);
            response += "update frequency set to " + String(frequency) + "Hz";
            addLog("[IMU] Update frequency changed to " + String(frequency) + "Hz");
        } else {
            response += "invalid frequency (valid range: 10-200 Hz)";
        }
    } else if (arg == "raw") {
        const IMUModule::RawData& raw = boat->imu.getRawData();
        response += "Accel[" + String(raw.accelX, 2) + "," + String(raw.accelY, 2) + "," + String(raw.accelZ, 2) + "]";
        response += " Gyro[" + String(raw.gyroX, 1) + "," + String(raw.gyroY, 1) + "," + String(raw.gyroZ, 1) + "]";
        response += " Mag[" + String(raw.magX, 1) + "," + String(raw.magY, 1) + "," + String(raw.magZ, 1) + "]";
    } else {
        response += "Usage: I:[S|C|R|D|F|freq:N|raw] (Status/Calibrate/Reset/Diagnostic/Force_update/Frequency/Raw_data)";
    }
    return response;
}
