// Boat.hpp
#pragma once
#include <Arduino.h>
#include <vector>
#include <algorithm>
#include <ArduinoJson.h>
#include <MPU9250_asukiaaa.h>
#include <WiFi.h>
// #include <TinyGPSPlus.h>
#include "settings.h"
#include "gnss_manager.hpp"
#include "battery_monitor.hpp"
#include "voltage_current_sensor.hpp"
#include "fly_sky.hpp"
#include "system_status.hpp"
#include "temp_sensors.hpp"
#include "oil_pump.hpp"
#include "rudder.hpp"
#include "motor_control.hpp"
#include "command_parser.hpp"
#include "Adafruit_INA3221.h"
#include <Preferences.h>
#include <time.h>
#include "FS.h"
#include "SPIFFS.h"
#include "LoRaCore.hpp"
#include <Adafruit_PWMServoDriver.h>
#include "PacketAsaExchange.hpp"
#include "boat_utils.hpp"
#include "auto_navigation.hpp"
#include "stack_monitor.hpp"
#include "GPSPoint.hpp"
#include "BoatSettings.hpp"
#include "DataPacket.hpp"

using namespace ArduinoJson;

// ============================================================================
// PERFORMANCE OPTIMIZATION CONSTANTS
// ============================================================================
static constexpr unsigned long CHANNEL_PRINT_INTERVAL = 2000;
static constexpr unsigned long CONTROL_INTERVAL = 100; // 10 Hz
static constexpr unsigned long RSSI_REPORT_INTERVAL = 27123;
static constexpr unsigned long PING_INTERVAL = 40013;
static constexpr unsigned long ASA_TIMEOUT = 15007;
static constexpr unsigned long ACTIVITY_TIMEOUT = 42777;
static constexpr unsigned long ADAPTIVE_SWITCH_INTERVAL = 8000; // 🚀 Reduced from 25s to 8s for faster adaptation

// ============================================================================
// MAIN BOAT CLASS WITH OPTIMIZATIONS
// ============================================================================
static unsigned long lastChannelPrint = 0;
static unsigned long lastRandomRudderTime = 0;
static unsigned long nextRandomRudderDelay = 0;
static unsigned long lastControlUpdate = 0;

// ============================================================================
// HELPER CLASSES AND STRUCTURES
// ============================================================================

// Simple performance monitoring

class Boat : LogInterface
{
public:
    MPU9250_asukiaaa mpu;
    FusionAhrs ahrs;
    BatteryMonitor battery;
    VoltageCurrentSensor sensor;
    Adafruit_INA3221 ina3221;
    Adafruit_INA3221 ina3221_low;
    FlySkyReceiver flysky;
    OilPumpController oilPump;
    TempSensorManager temps;
    RudderController rudder{RUDDER};
    MotorEngineControl engine;
    CommandParser parser;
    GNSSManager gnss;
    AutoNavigation autoNav;
    bool updateStarted = false;
    LoRaCore *loraComm; // Unified LoRa communication object
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

    TaskHandle_t sendStatusTaskHandle = nullptr;
    bool statusTaskRunning = false;

    // 🚀 NEW: Structured data transmission system
    BoatSettings bs;      // Текущее состояние лодки
    unsigned long lastHeartbeatSent = 0;   // Время последнего heartbeat
    unsigned long lastFullStatusSent = 0;  // Время последнего полного статуса
    unsigned long lastDataUpdate = 0;      // Время последнего обновления данных
    bool useStructuredData = true;         // Флаг использования новой системы
    static constexpr unsigned long DATA_UPDATE_INTERVAL = 1000;   // 1 секунда

    bool waitingForASAAck = false;
    bool asaActive = false;
    unsigned long asaProposalTime = 0;
    unsigned long lastPacketReceived = 0; // обновляется при каждом принятом пакете
    unsigned long asaLastSwitchTime = 0;
    PacketId_t lastAsaRequestId = 0; // ID последнего отправленного ASA запроса
    int currentSF = LORA_SF;
    int currentCR = LORA_CODING_RATE;
    float currentBW = LORA_BANDWIDTH;

    void onTelemetry(const PacketTelemetry &tel)
    {
        addLog("Received telemetry fragment: len=" + String(tel.payloadLen));
        // собрать, parse и т.д.
    }

    void onInfoEngine(const PacketInfoEngine &info)
    {
        addLog("InfoEngine packet received");
    }

    void onStatus(const PacketStatus &st)
    {
        addLog("Status packet received");
    }

    void onAck(const PacketAck &ack)
    {
        addLog("ACK for ID=" + String(ack.ackedId));
    }

    void onConfig(const PacketConfig &cfg)
    {
        addLog("Config packet received");
    }

    void onNav(const PacketNav &nav)
    {
        addLog("Nav packet received");
    }

    void onHeartbeat(const PacketHeartbeat &hb)
    {
        addLog("Heartbeat packet received");
    }

    Boat() : sensor(0x48), gnss(Serial2)
    {
        loraComm = new LoRaCore(BOAT_DEVICE_ID, this);
        // Создаем мьютекс для потокобезопасного логирования
        logMutex = xSemaphoreCreateMutex();
        if (logMutex == nullptr)
        {
            Serial.println("ERROR: Failed to create log mutex!");
        }
    }
    ~Boat()
    {
        delete loraComm;
        // Освобождаем мьютекс
        if (logMutex != nullptr)
        {
            vSemaphoreDelete(logMutex);
        }
    }

    // Helper function to register command with response
    void registerCommandWithResponse(const String& cmd, std::function<String(const String&)> handler)
    {
        parser.registerCommand(cmd, [this, handler](const String &arg) {
            String response = handler(arg);
            sendResponseToMissionControl(response);
        });
    }
    // Helper function for LoRa profile management
    String handleLoRaProfileCommand(const String& arg)
    {
        String response = "L:";
        if (arg == "S" || arg == "status") {
            String status = "Profile:" + String(loraComm->getCurrentProfileIndex()) + 
                   ",RSSI:" + String(smoothedRssi) + "dBm" +
                   ",Adaptive:" + String(allowAdaptiveLoraSwitch ? "ON" : "OFF");
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
        } else if (arg.length() > 0 && arg[0] >= '0' && arg[0] <= '9') {
            int profileIndex = arg.toInt();
            response += applyLoRaProfile(profileIndex);
        } else {
            response += "Current profile:" + String(loraComm->getCurrentProfileIndex()) + 
                       ",RSSI:" + String(smoothedRssi) + "dBm";
        }
        return response;
    }

    String applyLoRaProfile(int profileIndex)
    {
        if (profileIndex >= 0 && profileIndex < LORA_PROFILE_COUNT) {
            applyProfile(profileIndex);
            addLog("[LORA] Manually switched to profile " + String(profileIndex));
            return "Profile changed to " + String(profileIndex);
        } else {
            return "Invalid profile index:" + String(profileIndex);
        }
    }

    String handleAdaptiveControl(const String& adaptiveStr)
    {
        if (adaptiveStr == "0") {
            allowAdaptiveLoraSwitch = false;
            addLog("[LORA] Adaptive switching DISABLED by command");
            return "Adaptive switching disabled";
        } else if (adaptiveStr == "1") {
            allowAdaptiveLoraSwitch = true;
            addLog("[LORA] Adaptive switching ENABLED by command");
            return "Adaptive switching enabled";
        } else {
            return "Invalid adaptive command (use A:0 or A:1)";
        }
    }

    void setup()
    {
        delay(3); // Задержка для отладки
        addLog("RAM: Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        addLog("Flash: Sketch size:" + String(ESP.getSketchSize()) + " bytes / Free: " + String(ESP.getFreeSketchSpace()) + " bytes");

        // addLog("Enabling servo power ,,,,,,,,,,,,,,,");
        // pinMode(SERVO_PWR_PIN, OUTPUT);
        // digitalWrite(SERVO_PWR_PIN, HIGH); // Включаем питание сервоприводов
        battery.setup();
        gnss.begin();

        addLog("Starting I2C bus scan...");
        Wire.begin(I2C_SDA, I2C_SCL);
        addLog("Scanning I2C bus...");

        for (uint8_t address = 1; address < 127; ++address)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                addLog("Found I2C device at 0x" + String(address, HEX));
            }
        }

        mpu.setWire(&Wire);
        mpu.beginAccel();
        mpu.beginGyro();
        mpu.beginMag();
        FusionAhrsInitialise(&ahrs);

        addLog("MPU9250 + Madgwick initialized");

        // Инициализация INA3221 датчиков
        initINA3221(ina3221, 0x41, "INA3221", 4.0f, 18.0f);
        initINA3221(ina3221_low, 0x43, "INA3221 LOW", 3.2f, 10.0f);
        SystemStatus::printResetReason();
        SystemStatus::printWakeupReason();
        SystemStatus::printUptime();
        sensor.begin();
        temps.begin();
        addLog("Dallas sensors initialized");
        oilPump.begin();
        addLog("Oil pump initialized");

        rudder.begin();
        rudder.setTrim(0);
        engine.begin(MOTOR_LEFT, MOTOR_RIGHT);
        addLog("Boat finish setup");

        parser.registerCommand("M", [this](const String &arg)
                               {
                                    addLog("[CMD M] Motor power set to: " + arg);
                                    int power = arg.toInt();
                                    engine.apply(power, 1500); });

        parser.registerCommand("E", [this](const String &arg)
                               {
                                    addLog("[CMD E] Engine power set to: " + arg);
                                        if (arg=="S") {
                                            addLog("[CMD E] Engine stopped");
                                            engine.setState(MotorEngineControl::MOTOR_STOP);
                                        } else if(arg=="F") {
                                            addLog("[CMD E] Engine forward");
                                            engine.setState(MotorEngineControl::MOTOR_FORWARD);
                                        } else if(arg=="R") {
                                            addLog("[CMD E] Engine reverse");
                                            engine.setState(MotorEngineControl::MOTOR_REVERSE);
                                        } else {
                                            addLog("[CMD E] Unknown engine command: " + arg);
                                        } });

        parser.registerCommand("R", [this](const String &arg)
                               {
                                    addLog("[CMD R] Rudder angle set to: " + arg);
                                    int angle = arg.toInt();
                                    rudder.setAngle(angle); });

        parser.registerCommand("P", [this](const String &arg)
                               {
                                    addLog("[CMD P] Oil pump set to: " + arg);
                                    int speed = arg.toInt();
                                    oilPump.setSpeed(speed); });

        // LoRa commands with simplified registration
        registerCommandWithResponse("L", [this](const String &arg) {
            return handleLoRaProfileCommand(arg);
        });

        // Diagnostic commands with simplified registration
        registerCommandWithResponse("D", [this](const String &arg) {
            return handleDiagnosticCommand(arg);
        });

        // Navigation commands with simplified registration
        registerCommandWithResponse("N", [this](const String &arg) {
            return handleNavigationCommand(arg);
        });

        // Waypoint commands with simplified registration
        registerCommandWithResponse("W", [this](const String &arg) {
            return handleWaypointCommand(arg);
        });

        // Time commands with simplified registration
        registerCommandWithResponse("T", [this](const String &arg) {
            return handleTimeCommand(arg);
        });

        // Data mode commands with simplified registration
        registerCommandWithResponse("DM", [this](const String &arg) {
            return handleDataModeCommand(arg);
        });

        // 📶 RSSI Report command
        parser.registerCommand("RSSI", [this](const String &arg)
                               {
                                   sendRssiReport(MISSION_CONTROL_ID);
                                   addLog("📶 RSSI report sent on request"); });

        addLog("Commands registered: M (motor), E (engine), R (rudder), P (oil pump), D (diagnostics), L (LoRa - supports direct profile numbers + A:0/A:1 adaptive control), N (navigation), W (waypoints), T (time), DM (data mode - structured data commands), RSSI (signal report)");

        addLog("Boat setup completed.");
        addLog("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        addLog("Free sketch space: " + String(ESP.getFreeSketchSpace()) + " bytes");
        addLog("Sketch size: " + String(ESP.getSketchSize()) + " bytes");
        addLog("Flash chip size: " + String(ESP.getFlashChipSize()) + " bytes");
        addLog("Flash chip speed: " + String(ESP.getFlashChipSpeed()) + " Hz");
        addLog("Chip model: " + String(ESP.getChipModel()));
        addLog("Chip revision: " + String(ESP.getChipRevision()));
        addLog("Chip cores: " + String(ESP.getChipCores()));
        addLog("CPU frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
        addLog("SDK version: " + String(ESP.getSdkVersion()));
        addLog("MAC address: " + String(ESP.getEfuseMac(), HEX));
        printSPIFFSInfo();
        // listSPIFFSFiles();
        addLog("Starting PWM for servos...");

        bool servoStarted = pwm.begin();
        if (!servoStarted)
        {
            addLog(" - ERROR! failed to start Adafruit_PWMServoDriver.");
        }
        else
        {
            addLog(" + Adafruit_PWMServoDriver succesfully connected to module.");
            pwm.setPWMFreq(50);
        }

        // Инициализация нового класса LoRaComm
        if (!loraComm->begin())
        {
            addLog(" - ERROR! LoRaComm не инициализирован.");
        }
        else
        {
            addLog(" + LoRaCore успешно инициализирован.");
            
            // Устанавливаем callback для обработки ACK
            loraComm->setAckCallback([this](PacketId_t packetId, uint8_t senderId, uint8_t originalPacketType) {
                this->handleAckReceived(packetId, senderId, originalPacketType);
            });
        }

        addLog("Strarting setup iBUS FlySky...");
        flysky.begin();
        addLog("Boat initialized successfully.");

        // Инициализация мониторинга стека
        stackMonitor.autoRegisterTasks();
        addLog("Stack monitor initialized with " + String(stackMonitor.getTaskCount()) + " tasks");

        // Initial GPS time check and setup
        addLog("🕒 Checking GPS time sync capability...");
        if (gnss.isTimeUpdated())
        {
            gnss.syncSystemTimeFromGPS();
            addLog("🕒 GPS time sync performed during setup");
        }
        else
        {
            addLog("🕒 GPS time not yet available, will sync when ready");
        }
    }

    unsigned long lastSync = 0;

    // Helper for structured data commands
    String handleStructuredDataCommand(const String& command)
    {
        if (!useStructuredData) {
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

    // Helper function for diagnostic commands
    String handleDiagnosticCommand(const String& arg)
    {
        String response = "D:";
        if (arg == "S") {
            String status = safetyMonitor.getStatusString();
            addLog("[DIAG] " + status);
            response += "Safety:" + status;
        } else if (arg == "P") {
            String perfInfo = "avg=" + String(performanceMetrics.getAverageKeepTime()) + 
                   "ms,max=" + String(performanceMetrics.maxKeepDuration) + "ms";
            addLog("[DIAG] Performance: " + perfInfo);
            response += "Performance:" + perfInfo;
        } else if (arg == "T") {
            String tempInfo = "M1=" + String(temps.get(MOTOR1)) + 
                   "C,M2=" + String(temps.get(MOTOR2)) + "C";
            addLog("[DIAG] Temps: " + tempInfo);
            response += "Temps:" + tempInfo;
        } else if (arg == "R") {
            performanceMetrics.reset();
            addLog("[DIAG] Performance metrics reset");
            response += "Performance metrics reset";
        } else if (arg == "full" || arg.isEmpty()) {
            response += buildFullDiagnostic();
        } else if (arg == "extended" || arg == "ext") {
            response += buildExtendedDiagnostic();
        } else if (arg == "stack" || arg == "st") {
            String stackReport = stackMonitor.getReport();
            addLog("[DIAG] Stack report:");
            addLog(stackReport);
            response += "Stack Report:\n" + stackReport;
        } else {
            response += "Unknown diagnostic command:" + arg;
        }
        return response;
    }

    String buildFullDiagnostic()
    {
        String safetyInfo = safetyMonitor.getStatusString();
        if (safetyInfo.length() > 30) safetyInfo = safetyInfo.substring(0, 30) + "...";
        
        String perfInfo = "avg=" + String(performanceMetrics.getAverageKeepTime()) + "ms";
        String tempInfo = "M1=" + String(temps.get(MOTOR1), 1) + "C,M2=" + String(temps.get(MOTOR2), 1) + "C";
        String gpsInfo = (gnss.hasValidFix() ? "OK" : "NO_FIX");
        String heapInfo = String(ESP.getFreeHeap());
        
        String fullDiag = "Safety:" + safetyInfo + 
                         ";Perf:" + perfInfo +
                         ";Temps:" + tempInfo +
                         ";GPS:" + gpsInfo +
                         ";Heap:" + heapInfo;
        
        addLog("[DIAG] Full diagnostic (" + String(fullDiag.length()) + " bytes): " + fullDiag);
        return fullDiag;
    }

    String buildExtendedDiagnostic()
    {
        String extDiag = "System:OK;";
        extDiag += "Uptime:" + String(millis()/1000) + "s;";
        extDiag += "FreeHeap:" + String(ESP.getFreeHeap()) + ";";
        extDiag += "MinFreeHeap:" + String(ESP.getFlashChipSize()) + ";";
        extDiag += "FlashSize:" + String(ESP.getFlashChipSize()) + ";";
        extDiag += "CPUFreq:" + String(ESP.getCpuFreqMHz()) + "MHz;";
        extDiag += "ChipModel:" + String(ESP.getChipModel()) + ";";
        extDiag += "TempSensors:" + String(temps.getSensorCount()) + ";";
        extDiag += "WiFiStatus:" + String(WiFi.status()) + ";";
        extDiag += "LoRaProfile:" + String(loraComm->getCurrentProfileIndex()) + ";";
        extDiag += "Safety:" + safetyMonitor.getStatusString() + ";";
        extDiag += "Performance:avg=" + String(performanceMetrics.getAverageKeepTime()) + "ms;";
        extDiag += "Stack:" + stackMonitor.getCompactStatus();
        
        addLog("[DIAG] Extended diagnostic (" + String(extDiag.length()) + " bytes): " + extDiag);
        return extDiag;
    }

    // Helper function for navigation commands
    String handleNavigationCommand(const String& arg)
    {
        String response = "N:";
        if (arg == "M" || arg == "manual") {
            autoNav.setMode(AutoNavigation::MANUAL);
            addLog("[NAV] Switched to manual mode");
            response += "Manual mode activated";
        } else if (arg == "R" || arg == "home") {
            autoNav.setMode(AutoNavigation::RETURN_TO_HOME);
            addLog("[NAV] Return to home initiated");
            response += "Return to home mode activated";
        } else if (arg == "S" || arg == "station") {
            autoNav.setMode(AutoNavigation::STATION_KEEPING);
            addLog("[NAV] Station keeping mode");
            response += "Station keeping mode activated";
        } else if (arg == "H" || arg == "sethome") {
            response += handleSetHome();
        } else if (arg.startsWith("start:")) {
            response += handleNavigationStart(arg.substring(6));
        } else if (arg == "stop") {
            autoNav.setMode(AutoNavigation::MANUAL);
            response += "Navigation stopped";
        } else if (arg == "pause") {
            autoNav.pause();
            response += "Navigation paused";
        } else if (arg == "resume") {
            autoNav.resume();
            response += "Navigation resumed";
        } else if (arg.startsWith("mode:")) {
            response += handleNavigationMode(arg.substring(5));
        } else {
            String status = autoNav.getStatusString();
            addLog("[NAV] Status: " + status);
            response += "Status:" + status;
        }
        return response;
    }

    String handleSetHome()
    {
        if (gnss.hasValidFix()) {
            autoNav.setHome(gnss.getLatitude(), gnss.getLongitude());
            String homePos = String(gnss.getLatitude(), 6) + "," + String(gnss.getLongitude(), 6);
            addLog("[NAV] Home position set: " + homePos);
            return "Home set at " + homePos;
        } else {
            addLog("[NAV] Cannot set home - no GPS fix");
            return "Error: No GPS fix available";
        }
    }

    String handleNavigationStart(const String& coords)
    {
        int commaIndex = coords.indexOf(',');
        if (commaIndex > 0) {
            float lat = coords.substring(0, commaIndex).toFloat();
            float lon = coords.substring(commaIndex + 1).toFloat();
            autoNav.setTarget(lat, lon);
            autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
            addLog("[NAV] Navigation started to: " + String(lat, 6) + "," + String(lon, 6));
            return "Navigation started to " + String(lat, 6) + "," + String(lon, 6);
        } else {
            return "Error: Invalid coordinates format";
        }
    }

    String handleNavigationMode(const String& mode)
    {
        if (mode == "manual") autoNav.setMode(AutoNavigation::MANUAL);
        else if (mode == "waypoint") autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
        else if (mode == "home") autoNav.setMode(AutoNavigation::RETURN_TO_HOME);
        else if (mode == "station") autoNav.setMode(AutoNavigation::STATION_KEEPING);
        return "Mode set to " + mode;
    }

    // Helper function for waypoint commands
    String handleWaypointCommand(const String& arg)
    {
        String response = "W:";
        if (arg.startsWith("A") || arg.startsWith("add:")) {
            String coords = arg.startsWith("add:") ? arg.substring(4) : arg.substring(1);
            int commaIndex = coords.indexOf(',');
            if (commaIndex > 0) {
                float lat = coords.substring(0, commaIndex).toFloat();
                float lon = coords.substring(commaIndex + 1).toFloat();
                autoNav.addWaypoint(lat, lon);
                addLog("[NAV] Waypoint added: " + String(lat, 6) + "," + String(lon, 6));
                response += "Waypoint added:" + String(lat, 6) + "," + String(lon, 6);
            } else {
                response += "Error: Invalid waypoint format";
            }
        } else if (arg == "S" || arg == "start") {
            autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
            addLog("[NAV] Waypoint following started");
            response += "Waypoint following started";
        } else if (arg == "C" || arg == "clear") {
            autoNav.clearWaypoints();
            addLog("[NAV] Waypoints cleared");
            response += "Waypoints cleared";
        } else if (arg == "status") {
            response += "Web interface active,clients:" + String(WiFi.softAPgetStationNum());
        } else if (arg == "update") {
            response += "Web interface updated";
        } else {
            response += "Waypoint count:" + String(autoNav.getWaypointCount());
        }
        return response;
    }

    // Helper function for time commands
    String handleTimeCommand(const String& arg)
    {
        String response = "T:";
        if (arg == "S" || arg == "sync") {
            if (gnss.isTimeUpdated()) {
                gnss.syncSystemTimeFromGPS();
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
            String timeStr = this->timeStr();
            String status = "Time:" + timeStr + 
                   ",Synced:" + String(synced ? "YES" : "NO") + 
                   ",GPS_Updated:" + String(gnss.isTimeUpdated() ? "YES" : "NO");
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

    // Helper function for data mode commands
    String handleDataModeCommand(const String& arg)
    {
        String response = "DM:";
        if (arg == "S" || arg == "structured") {
            useStructuredData = true;
            addLog("📦 Switched to structured data transmission");
            response += "structured data enabled";
        } else if (arg == "J" || arg == "json") {
            useStructuredData = false;
            addLog("📄 Switched to JSON data transmission");
            response += "JSON data enabled";
        } else if (arg == "?") {
            response += String(useStructuredData ? "structured" : "json") + 
                       ", HB:" + String((millis() - lastHeartbeatSent)/1000) + "s ago" +
                       ", Full:" + String((millis() - lastFullStatusSent)/1000) + "s ago";
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

    // Helper function for mission control deactivation logic
    void deactivateMissionControl(const String& reason)
    {
        addLog("💔 " + reason);
        missionCOntrolIsActivae = false;
        PongRssi = 0;
        
        // 🔄 Отключаем адаптивное переключение при потере пульта
        allowAdaptiveLoraSwitch = false;
        addLog("❌ Adaptive LoRa switching disabled (MissionControl disconnected)");
        
        // Сбрасываем кэш для немедленного применения профиля 0
        sensorCache.valid = false;
        
        // Если активен ASA режим - тоже выключаем
        if (asaActive) {
            addLog("⏳ ASA: Деактивация из-за потери MissionControl. Возврат к дефолтным LoRa-настройкам.");
            restoreDefaultLoRaSettings();
            asaActive = false;
        }
    }

    void keep()
    {
        // Performance monitoring
        unsigned long keepStartTime = millis();

        if (updateStarted)
        {
            addLog("Update in progress, skipping keep() cycle.");
            return;
        }

        // Quick safety check first - most critical
        static unsigned long lastSafetyCheck = 0;
        static unsigned long lastStackCheck = 0;

        if (millis() - lastSafetyCheck >= 6111)
        { // 10Hz для безопасности
            safetyMonitor.checkSystemHealth(temps, battery, lastPacketReceived);

            // Emergency shutdown if critical conditions are met
            if (safetyMonitor.overtemperatureShutdown || safetyMonitor.lowVoltageShutdown)
            {
                engine.setState(engine.MOTOR_STOP);
                addLog("⚠️ EMERGENCY SHUTDOWN: " + safetyMonitor.getStatusString());
            }
            lastSafetyCheck = millis();
        }


        if (millis() - lastStackCheck >= 1800000)
        {
            stackMonitor.update();
            if (stackMonitor.hasCriticalStackUsage())
            {
                addLog("🚨 STACK WARNING: " + stackMonitor.getCriticalTasks());
            }
            lastStackCheck = millis();
        }

        updateSensorsOptimized();       // Optimize sensor updates - разделяем по времени
        processLoRaPackets();           // Process LoRa packets - вынесено в отдельную функцию
        updateControlAndNavigation();   // Control and navigation updates

        static unsigned long lastOilPumpUpdate = 0;
        static unsigned long VuPDATE = 0;
        if (waitingForASAAck && millis() - asaProposalTime > ASA_TIMEOUT)
        {
            addLog("❌ ASA: Нет ACK от управления. Отклоняем переход.");
            if (lastAsaRequestId != 0 && loraComm) {// Удаляем ASA запрос из pending списка при таймауте
                if (loraComm->removePendingPacket(lastAsaRequestId)) {
                    addLog("🗑️ Removed ASA request id=" + String(lastAsaRequestId) + " from pending (timeout)");
                }
                lastAsaRequestId = 0;
            }
            
            waitingForASAAck = false;
        }
        // 🚨 ОБЩАЯ проверка heartbeat от MissionControl (независимо от ASA)
        if (missionCOntrolIsActivae && millis() - lastPacketReceived > ACTIVITY_TIMEOUT)
        {
            unsigned long timeoutSeconds = (millis() - lastPacketReceived) / 1000;
            deactivateMissionControl("MissionControl heartbeat timeout (" + String(timeoutSeconds) + 
                                   "s > " + String(ACTIVITY_TIMEOUT/1000) + "s) - deactivating");
        }

        if (asaActive && millis() - lastPacketReceived > ACTIVITY_TIMEOUT)
        {
            addLog("⏳ ASA: Таймаут активности. Возврат к дефолтным LoRa-настройкам.");
            restoreDefaultLoRaSettings();
            asaActive = false;
            PongRssi = 0;
            missionCOntrolIsActivae = false;
            
            // 🔄 Отключаем адаптивное переключение при потере пульта (дублирование для надежности)
            allowAdaptiveLoraSwitch = false;
            
            // Сбрасываем кэш для немедленного применения профиля 0
            sensorCache.valid = false;
        }

        // 🚨 ПРИНУДИТЕЛЬНЫЙ переход на профиль 0 при потере связи с пультом
        if (!missionCOntrolIsActivae && loraComm->getCurrentProfileIndex() != 0)
        {
            addLog("🚨 Mission Control неактивен - принудительный переход на профиль 0");
            applyProfile(0);
            // Сбрасываем кэш, чтобы избежать повторного переключения
            sensorCache.valid = false;
            // Сбрасываем время последнего адаптивного переключения
            lastAdaptiveSwitchTime = 0;
        }

        // Адаптивная LoRa работает ТОЛЬКО когда MissionControl активен И разрешено переключение
        if (missionCOntrolIsActivae && allowAdaptiveLoraSwitch)
        {
            adaptiveLoraUpdate();
        }


        if (millis() - lastOilPumpUpdate >= BOAT_TMR_OIL_PUMP_UPDATE_TIME)
        {
            int percent = 0;
            lastOilPumpUpdate = millis();
            if (temps.get(MOTOR1) > 35 || temps.get(MOTOR2) > 35)
            {
                percent = 90;
            }
            else
            {
                percent = 0;
            }

            oilPump.setSpeed(percent);
        }

        static unsigned long lastRssiSent = 0;
        if (millis() - lastRssiSent > RSSI_REPORT_INTERVAL)
        {
            sendRssiReport(MISSION_CONTROL_ID); // или другой ID
            lastRssiSent = millis();
        }
        // Показываем системное время раз в 5 секунд
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > BOAT_TMR_STSTEM_PRINT_TIME)
        {
            lastPrint = millis();
            time_t now = time(nullptr);
            addLog("System time:" + String(now));
        }

        // 🔄 Ping'и отправляются только если Mission Control активен и разрешены автоматические пинги
        // Отключено по умолчанию - отправка только по запросу
        // if (missionCOntrolIsActivae && millis() - lastPingSent >= PING_INTERVAL && !waitingForASAAck)
        // {
        //     PacketBase ping{};
        //     ping.packetType = CMD_PING;
        //     ping.packetId = nextPacketId++;
        //     ping.payloadLen = 0;
        //     loraComm->sendPacketBase(MISSION_CONTROL_ID, ping, nullptr, false);
        //     addLog("[PROF:" + String(loraComm->getCurrentProfileIndex()) + "] 🔄 Ping → MC");
        //     lastPingSent = millis();
        // }


        // Синхронизация при первом валидном значении GPS времени
        static unsigned long lastSync = 0;
        static bool timeEverSynced = false;

        // Первая синхронизация - сразу при получении валидного GPS времени
        if (gnss.isTimeUpdated() && !timeEverSynced)
        {
            gnss.syncSystemTimeFromGPS();
            lastSync = millis();
            timeEverSynced = true;
            addLog("🕒 Initial GPS time sync completed");
        }
        // Последующие синхронизации - каждые 160 секунд
        else if (gnss.isTimeUpdated() && timeEverSynced && millis() - lastSync > 160000)
        {
            gnss.syncSystemTimeFromGPS();
            lastSync = millis();
            addLog("🕒 GPS time re-sync completed");
        }
        // static unsigned long wakeStart = 0;
        // static bool firstRun = true;

        // if (firstRun)
        // {
        //     wakeStart = millis();
        //     firstRun = false;
        //     Serial.println("⏰ Wake cycle started.");
        // }

        // const unsigned long wakeDuration = 3600000;                 // 1 минута
        // const uint64_t sleepDurationUs = 1ULL * 60ULL * 1000000ULL; // 5 минут

        // if (millis() - wakeStart >= wakeDuration)
        // {
        //     if (!flysky.transmitter_on)
        //     {
        //         flysky.powerOff();
        //         sensorsPowerOff();
        //         delay(100); // Даем время на отключение
        //         Serial.println("😴 No transmitter detected. Going to deep sleep for 5 minutes...");
        //         ESP.deepSleep(sleepDurationUs);
        //     }
        //     else
        //     {
        //         Serial.println("📡 Transmitter is active. Stay awake.");
        //         wakeStart = millis(); // сбрасываем цикл, остаёмся бодрствовать
        //     }
        // }
        while (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n')
            {
                Serial.println("Command processed: " + inputBuffer);
                parser.processLine(inputBuffer);
                inputBuffer = "";
            }
            else if (c >= 32 && c <= 126)
            {
                inputBuffer += c;
            }
        }
        uint16_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;

        if (!failsafeTriggered)
        {
            ch1 = flysky.getChannel(0); // Правый stick X
            ch2 = flysky.getChannel(1); // Правый stick Y
            ch3 = flysky.getChannel(2); // Левый stick Y
            ch4 = flysky.getChannel(3); // Левый stick X
            ch5 = flysky.getChannel(4);
            ch6 = flysky.getChannel(5);
            ch7 = flysky.getChannel(6);
            // ch8 = flysky.getChannel(7);
            // ch9 = flysky.getChannel(8);
            // ch10 = flysky.getChannel(9);
            //}
            if (ch5 < 1900)
            {
                engine.setState(engine.MOTOR_STOP);
            }
            else
            {
                if (ch6 < 1500 && ch6 >= 1000)
                {
                    engine.setState(engine.MOTOR_FORWARD);
                }
                else if (ch6 > 1500 && ch6 <= 2000)
                {
                    engine.setState(engine.MOTOR_REVERSE);
                }
                else
                {
                    engine.setState(engine.MOTOR_STOP);
                }
            }
            if (millis() - lastChannelPrint > CHANNEL_PRINT_INTERVAL)
            {
                lastChannelPrint = millis();
                Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u\n", ch1, ch2, ch3, ch4);
                // Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u -- CH5: %u | CH6: %u | CH7: %u | CH8: %u :::: CH9: %u - CH10: %u\n", ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,ch9, ch10);
            }

            lastControlUpdate = millis();
            setThrottleLimit(ch7);
            engine.apply(ch3, ch4);

            if (ch1 >= 1000 && ch1 <= 2000 && flysky.transmitter_on)
            {
                rudder.setAngle(map(ch1, 1000, 2000, -90, 90)); // Обновляем руль только если есть валидный сигнал
            }
        }
        else
        {
            engine.update(); // Обновляем моторы даже в фейлсейфе, чтобы они остановились
        }
        checkFailsafeTransition();

        rudder.update();

        // Проверяем, не нужно ли отправить накопленные ACK
        checkBulkAckTimeout();

        // 🚀 NEW: Structured data transmission
        // processStructuredDataTransmission();

        // Performance tracking
        unsigned long keepDuration = millis() - keepStartTime;

        // Warn if keep() cycle is taking too long
        if (keepDuration > 50)
        { // 50ms warning threshold
            addLog("⚠️ Long keep() cycle: " + String(keepDuration) + "ms");
        }
    }

    void sendRssiReport(uint8_t receiverId)
    {
        PacketRssiReport pkt{};
        pkt.packetType = CMD_RSSI_REPORT;
        pkt.packetId = nextPacketId++;
        pkt.payloadLen = sizeof(pkt.rawRssi) + sizeof(pkt.smoothedRssi);

        pkt.rawRssi = loraComm->getRadio().getRSSI();
        pkt.smoothedRssi = smoothedRssi;

        // Create a proper payload buffer instead of using reinterpret_cast
        uint8_t payload[sizeof(float) * 2];
        memcpy(payload, &pkt.rawRssi, sizeof(float));
        memcpy(payload + sizeof(float), &pkt.smoothedRssi, sizeof(float));

        loraComm->sendPacketBase(receiverId, pkt, payload, false);
        addLog("📡 Sent RSSI Report → ID " + String(receiverId) +
               " raw=" + String(pkt.rawRssi) + " smoothed=" + String(pkt.smoothedRssi));
    }

    int rssiToIndex(float rssi)
    {
        for (size_t i = 0; i < rssiProfileCount; ++i)
        {
            if (rssi >= rssiToProfileTable[i].minRssi)
            {
                int index = rssiToProfileTable[i].profileIndex;
                addLog(String("[LoRa] RSSI = ") + String(rssi, 1) + " dBm → профиль " + String(index));
                return (index < LORA_PROFILE_COUNT) ? index : LORA_PROFILE_COUNT - 1;
            }
        }
        addLog(String("[LoRa] RSSI = ") + String(rssi, 1) + " dBm → профиль 0 (fallback)");
        return 0;
    }

    // Helper function to validate RSSI
    float validateRSSI(float rssi, float fallbackRssi = -90.0f)
    {
        if (rssi < -130.0f || rssi > 0.0f) {
            addLog("⚠️ Invalid RSSI value detected: " + String(rssi) + "dBm, using fallback");
            return sensorCache.isValid() ? sensorCache.lastRSSI : fallbackRssi;
        }
        return rssi;
    }

    void adaptiveLoraUpdate()
    {
        // Проверяем разрешение на адаптивное переключение
        if (!allowAdaptiveLoraSwitch) {
            return; // Адаптивное переключение запрещено
        }
        
        if (waitingForASAAck || millis() - lastAdaptiveSwitchTime < ADAPTIVE_SWITCH_INTERVAL)
            return;

        lastAdaptiveSwitchTime = millis();

        // Use cached values if available, otherwise get fresh data
        float rssi, snr;
        bool verboseDebug = false; // Flag to control verbose output
        
        if (sensorCache.isValid())
        {
            rssi = sensorCache.lastRSSI;
            snr = sensorCache.lastSNR;
        }
        else
        {
            rssi = loraComm->getRadio().getRSSI();
            snr = loraComm->getRadio().getSNR();
            verboseDebug = true; // Show debug for fresh data

            // Debug: показываем режим и исходные значения только для fresh data
            if (verboseDebug) {
                RadioMode currentMode = loraComm->mode();
                String modeStr = (currentMode == RadioMode::FSK) ? "GFSK" : "LoRa";
                addLog("📊 Fresh data: Mode=" + modeStr + ", RSSI=" + String(rssi, 2) + ", Raw SNR=" + String(snr, 2));
            }

            // Validate RSSI values - reject obviously incorrect readings
            rssi = validateRSSI(rssi, sensorCache.isValid() ? sensorCache.lastRSSI : -90.0f);

            sensorCache.update(rssi, snr);
        }

        // 🔧 GFSK Fix: В GFSK режиме SNR может быть неточным
        // Применяем коррекцию SNR для GFSK ВСЕГДА (как для свежих, так и для кэшированных данных)
        RadioMode currentMode = loraComm->mode();
        if (currentMode == RadioMode::FSK)
        {
            float originalSnr = snr;
            // В GFSK режиме используем фиксированное SNR на основе RSSI
            if (rssi > -50.0f)
            {
                snr = 15.0f; // Отличный сигнал
            }
            else if (rssi > -70.0f)
            {
                snr = 10.0f; // Хороший сигнал
            }
            else if (rssi > -85.0f)
            {
                snr = 8.0f; // Приемлемый сигнал
            }
            else
            {
                snr = 5.0f; // Слабый сигнал - стоит вернуться к LoRa
            }
            
            // // Show GFSK fix only for fresh data or when there's a significant difference
            // if (verboseDebug || abs(originalSnr - snr) > 2.0f) {
            //     addLog("🔧 GFSK SNR Fix: " + String(originalSnr, 1) + "dB → " + String(snr, 1) + "dB (RSSI-based estimate)");
            // }
        }

        int bestIndex = loraComm->getCurrentProfileIndex();
        // Always update PongRssi with current RSSI for profile selection
        PongRssi = rssi;

        // Enhanced profile selection with SNR consideration
        bestIndex = selectOptimalProfile(PongRssi, snr, verboseDebug);

        uint8_t currentProfile = loraComm->getCurrentProfileIndex();
        if (bestIndex != currentProfile)
        {
            String direction = bestIndex > currentProfile ? "upgrade" : "downgrade";
            addLog("[adaptiveLoraUpdate] rssi:" + String(PongRssi) + " snr:" + String(snr) +
                   " → Proposing " + direction + " to profile " + String(bestIndex));

            sendAsaRequest(loraComm, nextPacketId, bestIndex, MISSION_CONTROL_ID);
            lastAsaRequestId = nextPacketId; // Сохраняем ID для последующего удаления из pending
            nextPacketId++;
            waitingForASAAck = true;

            // 🚀 AGGRESSIVE UPGRADES: Apply upgrades immediately for faster adaptation
            if (direction == "upgrade" && rssi > -80.0f) {
                applyProfile(bestIndex);
                addLog("⬆️ ASA: Immediate upgrade to profile " + String(bestIndex) + " (good signal)");
            }
            // For downgrades, apply immediately for safety
            else if (direction == "downgrade")
            {
                applyProfile(bestIndex);
                addLog("⬇️ ASA: Emergency downgrade to profile " + String(bestIndex));
            }
            asaProposalTime = millis();
        }
        // No profile change needed - stay quiet unless it's fresh data
        else if (verboseDebug) {
            addLog("🔄 Profile " + String(currentProfile) + " optimal, no change needed (RSSI=" + String(rssi, 1) + ", SNR=" + String(snr, 1) + ")");
        }
    }

    // Enhanced profile selection with SNR consideration
    int selectOptimalProfile(float rssi, float snr, bool verbose = false)
    {
        if (verbose) {
            addLog("🔍 Profile Selection: RSSI=" + String(rssi, 1) + "dBm, SNR=" + String(snr, 1) + "dB");
        }

        // 🔒 ЗАЩИТА от экстремальных значений RSSI
        rssi = validateRSSI(rssi, PongRssi);

        int bestProfileByRssi = 0;      // Лучший профиль по RSSI
        int bestProfileByRssiSnr = 0;   // Лучший профиль по RSSI+SNR

        // Используем новую таблицу с поддержкой FSK и SNR
        for (size_t i = 0; i < rssiProfileCount; ++i)
        {
            if (verbose) {
                addLog("  🔸 Check [" + String(i) + "]: minRSSI=" + String(rssiToProfileTable[i].minRssi, 1) +
                       ", minSNR=" + String(rssiToProfileTable[i].minSnr, 1) +
                       " → profile " + String(rssiToProfileTable[i].profileIndex));
            }

            // Проверяем только по RSSI (для fallback)
            if (rssi >= rssiToProfileTable[i].minRssi) {
                bestProfileByRssi = rssiToProfileTable[i].profileIndex;
            }

            // Проверяем по RSSI+SNR (идеальный случай)
            if (rssi >= rssiToProfileTable[i].minRssi &&
                snr >= rssiToProfileTable[i].minSnr)
            {
                bestProfileByRssiSnr = rssiToProfileTable[i].profileIndex;
                
                if (verbose) {
                    String modeStr = (loraProfiles[bestProfileByRssiSnr].mode == RadioProfileMode::FSK) ? "GFSK" : "LoRa";
                    addLog("✅ PERFECT MATCH! RSSI=" + String(rssi, 1) + "dBm, SNR=" + String(snr, 1) +
                           "dB → профиль " + String(bestProfileByRssiSnr) + " (" + modeStr + ")");
                }

                return bestProfileByRssiSnr;
            }
        }

        // 🛡️ УМНЫЙ FALLBACK: Используем профиль на основе RSSI, но не хуже текущего на 2 позиции
        uint8_t currentProfile = loraComm ? loraComm->getCurrentProfileIndex() : 0;
        
        // Если SNR плохой, но RSSI хороший - делаем консервативный выбор
        if (bestProfileByRssi > 0 && bestProfileByRssi < currentProfile) {
            // Не деградируем больше чем на 2 профиля за раз при плохом SNR
            int conservativeProfile = max(currentProfile - 2, bestProfileByRssi);
            
            if (verbose) {
                addLog("🛡️ CONSERVATIVE: Bad SNR but good RSSI → profile " + String(conservativeProfile) + 
                       " (was " + String(bestProfileByRssi) + " by RSSI, current " + String(currentProfile) + ")");
            }
            
            return conservativeProfile;
        }

        // Fallback: используем лучший по RSSI, но не хуже профиля 2
        int fallbackProfile = max(bestProfileByRssi, 2);
        
        if (verbose) {
            addLog("⚠️ SNR-based fallback to profile " + String(fallbackProfile) + 
                   " (RSSI-based: " + String(bestProfileByRssi) + ")");
        }
        
        return fallbackProfile;
    }

    void applyProfile(uint8_t idx)
    {
        if (idx >= LORA_PROFILE_COUNT)
        {
            addLog("❌ Недопустимый индекс профиля: " + String(idx));
            return;
        }

        const auto &profile = loraProfiles[idx];

        // Используем новый метод LoRaCore для применения профилей из settings.h
        if (loraComm->applyProfileFromSettings(idx))
        {
            // Force cache invalidation after profile change to get fresh RSSI/SNR
            sensorCache.valid = false;
            
            String modeStr = (profile.mode == RadioProfileMode::FSK) ? "GFSK" : "LoRa";

            if (profile.mode == RadioProfileMode::LORA)
            {
                addLog("⚙️ Применён " + modeStr + " профиль " + String(idx) +
                       " (SF=" + String(profile.spreadingFactor) +
                       ", CR=" + String(profile.codingRate) +
                       ", BW=" + String(profile.bandwidth, 1) + "kHz)");
            }
            else
            {
                addLog("⚙️ Применён " + modeStr + " профиль " + String(idx) +
                       " (Bitrate=" + String(profile.bitrate) +
                       ", Dev=" + String(profile.deviation) +
                       ", RxBW=" + String(profile.bandwidth, 1) + "kHz)");
            }
        }
        else
        {
            addLog("❌ Ошибка применения профиля " + String(idx));
        }
    }

    void restoreDefaultLoRaSettings()
    {
        addLog("🔁 Восстановление стандартных LoRa параметров...");
        
        // Удаляем любой pending ASA запрос
        if (lastAsaRequestId != 0 && loraComm) {
            if (loraComm->removePendingPacket(lastAsaRequestId)) {
                addLog("🗑️ Removed ASA request id=" + String(lastAsaRequestId) + " from pending (restore defaults)");
            }
            lastAsaRequestId = 0;
        }
        
        applyProfile(0);          // Применяем профиль 0 (стандартный)
        asaActive = false;        // Деактивируем ASA
        waitingForASAAck = false; // Сбрасываем ожидание ACK
    }

    void printSPIFFSInfo()
    {
        addLog("=== SPIFFS Info ===");
        size_t totalBytes = SPIFFS.totalBytes();
        size_t usedBytes = SPIFFS.usedBytes();
        addLog("Total size: " + String(totalBytes / 1024) + " Kbytes");
        addLog("Used size:  " + String(usedBytes / 1024) + " Kbytes");
        addLog("Free size:  " + String(totalBytes - usedBytes / 1024) + " Kbytes");
    }

    void listSPIFFSFiles()
    {
        addLog("=== Files in SPIFFS ===");
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        if (!file)
        {
            addLog("No files found");
            return;
        }
        while (file)
        {
            addLog(String(file.name()) + " (" + String(file.size()) + " bytes)");
            file = root.openNextFile();
        }
    }

    const size_t maxPayload = sizeof(((LoRaPacket *)nullptr)->payload); // обычно 40

    // Примерная длина заголовка "[XX/YY]" — максимум 9 символов
    const size_t headerReserve = 10;                             // с запасом
    const size_t jsonChunkSize = maxPayload - headerReserve - 1; // -1 на '\0'

    static void sendStatusJsonTaskWrapper(void *param)
    {
        Boat *boat = static_cast<Boat *>(param);
        boat->sendStatusJsonFragmentsTask();
        vTaskDelete(nullptr); // убиваем задачу после выполнения
    }

    void sendStatusJsonFragmentsTask()
    {
        if (statusTaskRunning)
        {
            return;
        }
        statusTaskRunning = true;

        // Собираем полный JSON
        String json = getStatusJson();
        size_t totalLen = json.length();
        size_t chunks = (totalLen + jsonChunkSize - 1) / jsonChunkSize;

        // Для каждого фрагмента формируем команду
        for (size_t i = 0; i < chunks; i++)
        {
            // Заголовок вида “[i/N]”
            String header = "[" + String(i) + "/" + String(chunks) + "]";

            // Собственно фрагмент JSON
            String fragment = json.substring(i * jsonChunkSize, (i + 1) * jsonChunkSize);
            String payloadStr = header + fragment;

            // Упаковываем в PacketCommand
            PacketCommand cmd{};
            cmd.packetType = CMD_TELEMETRY_FRAGMENT;
            cmd.packetId = nextPacketId++;
            cmd.payloadLen = std::min<size_t>(payloadStr.length(), MAX_LORA_PAYLOAD);

            // Создаём буфер для данных
            uint8_t tempBuf[MAX_LORA_PAYLOAD];
            memcpy(tempBuf, payloadStr.c_str(), cmd.payloadLen);

            // Отправляем на MissionControl
            loraComm->sendPacketBase(MISSION_CONTROL_ID, cmd, tempBuf);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        statusTaskRunning = false;
    }

    String getStatusJson()
    {
        JsonDocument doc; // Подбери под размер данных

        JsonObject temp = doc[F("temperature")].to<JsonObject>();
        temp[F("motor1")] = temps.get(MOTOR1);
        temp[F("motor2")] = temps.get(MOTOR2);
        temp[F("radiator")] = temps.get(MOTOR_RAD);
        temp[F("oil")] = temps.get(OIL);
        temp[F("ambient")] = temps.get(ENV);

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(quat);

        JsonObject imu = doc[F("imu")].to<JsonObject>();
        imu[F("roll")] = euler.angle.roll;
        imu[F("pitch")] = euler.angle.pitch;
        imu[F("yaw")] = euler.angle.yaw;

        JsonObject pumpObj = doc[F("oil_pump")].to<JsonObject>();
        oilPump.toJSON(pumpObj);
        JsonObject sysObj = doc[F("system")].to<JsonObject>();
        SystemStatus::toExtendedJSON(sysObj);

        JsonObject rudderObj = doc[F("rudder")].to<JsonObject>();
        rudder.toJSON(rudderObj);

        JsonObject gnssObj = doc[F("gnss")].to<JsonObject>();
        gnss.toJson(gnssObj);

        // Add GPS time sync status
        time_t currentTime = time(nullptr);
        gnssObj[F("time_synced")] = (currentTime > 1600000000);
        gnssObj[F("system_time")] = currentTime;
        gnssObj[F("time_updated")] = gnss.isTimeUpdated();

        JsonObject motorObj = doc[F("motor")].to<JsonObject>();
        engine.toJSON(motorObj);

        JsonObject batteryObj = doc[F("b")].to<JsonObject>();
        battery.toJson(batteryObj);

        JsonObject receiverObj = doc["receiver"].to<JsonObject>();
        flysky.toJson(receiverObj);
        // Helper to add INA3221 array to JSON
        auto addINA3221ToJson = [&](JsonArray& array, Adafruit_INA3221& sensor) {
            for (uint8_t ch = 0; ch < 3; ch++)
            {
                JsonObject chObj = array.add<JsonObject>();
                chObj["channel"] = ch + 1;
                chObj["bus"] = sensor.getBusVoltage(ch);
                chObj["shunt"] = sensor.getShuntVoltage(ch);
                chObj["current"] = sensor.getCurrentAmps(ch);
            }
        };

        JsonArray inaArray = doc["ina3221"].to<JsonArray>();
        addINA3221ToJson(inaArray, ina3221);

        JsonArray inaLowArray = doc["ina3221_low"].to<JsonArray>();
        addINA3221ToJson(inaLowArray, ina3221_low);

        // Safety and performance monitoring
        JsonObject safetyObj = doc[F("safety")].to<JsonObject>();
        safetyObj[F("healthy")] = safetyMonitor.systemHealthy;
        safetyObj[F("overtemp")] = safetyMonitor.overtemperatureShutdown;
        safetyObj[F("low_voltage")] = safetyMonitor.lowVoltageShutdown;
        safetyObj[F("comm_loss")] = safetyMonitor.communicationLoss;

        JsonObject perfObj = doc[F("performance")].to<JsonObject>();
        perfObj[F("avg_keep_time")] = performanceMetrics.getAverageKeepTime();
        perfObj[F("max_keep_time")] = performanceMetrics.maxKeepDuration;
        perfObj[F("cycle_count")] = performanceMetrics.keepCycleCount;

        // Stack monitoring information
        JsonObject stackObj = doc[F("stack")].to<JsonObject>();
        stackObj[F("critical")] = stackMonitor.hasCriticalStackUsage();
        stackObj[F("status")] = stackMonitor.getCompactStatus();
        if (stackMonitor.hasCriticalStackUsage())
        {
            stackObj[F("critical_tasks")] = stackMonitor.getCriticalTasks();
        }

        // Memory information
        size_t totalHeap, freeHeap, minFreeHeap;
        stackMonitor.getMemoryInfo(totalHeap, freeHeap, minFreeHeap);
        JsonObject memObj = doc[F("memory")].to<JsonObject>();
        memObj[F("total_heap")] = totalHeap;
        memObj[F("free_heap")] = freeHeap;
        memObj[F("min_free_heap")] = minFreeHeap;
        memObj[F("heap_usage_pct")] = (float)(totalHeap - freeHeap) / totalHeap * 100.0f;

        // Enhanced LoRa statistics
        JsonObject loraObj = doc[F("lora")].to<JsonObject>();
        loraObj[F("profile_index")] = loraComm->getCurrentProfileIndex();
        loraObj[F("smoothed_rssi")] = smoothedRssi;
        if (sensorCache.isValid())
        {
            loraObj[F("rssi")] = sensorCache.lastRSSI;
            loraObj[F("snr")] = sensorCache.lastSNR;
        }
        loraObj[F("asa_active")] = asaActive;
        loraObj[F("waiting_asa_ack")] = waitingForASAAck;

        // Navigation information
        JsonObject navObj = doc[F("navigation")].to<JsonObject>();
        navObj[F("mode")] = autoNav.getStatusString();
        navObj[F("waypoint_count")] = autoNav.getWaypointCount();
        navObj[F("current_waypoint")] = autoNav.getCurrentWaypointIndex();
        if (gnss.hasValidFix())
        {
            navObj[F("lat")] = gnss.getLatitude();
            navObj[F("lon")] = gnss.getLongitude();
            navObj[F("heading")] = gnss.getHeading();
            navObj[F("speed")] = gnss.getSpeed();
            navObj[F("satellites")] = gnss.getSatelliteCount();
        }

        JsonArray logs = doc[F("logs")].to<JsonArray>();

        // Потокобезопасное чтение логов
        if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            for (const auto &logEntry : logBuffer)
            {
                logs.add(sanitizeLog(logEntry));
            }
            xSemaphoreGive(logMutex);
        }
        else
        {
            // Если не удалось получить мьютекс, добавляем сообщение об ошибке
            logs.add("ERROR: Could not access log buffer (mutex timeout)");
        }

        String result;
        serializeJson(doc, result);
        return result;
    }

    String sanitizeLog(const String &str)
    {
        String cleaned = str;
        cleaned.replace("\r", "");
        cleaned.replace("\n", "");
        cleaned.replace("\x1B", "");
        return cleaned;
    }
    void printStatus()
    {
        // float v = 0, a = 0;
        // sensor.read(v, a);
        // addLog(String("Motor1: ") + temps.get(MOTOR1) + " \t-  Motor 2 " + temps.get(MOTOR2) + " \u00B0C" + String("Radiator: ") + temps.get(MOTOR_RAD) + " \tOil " + temps.get(OIL) + " \u00B0C \tAmbient: " + temps.get(ENV) + " \u00B0C");
        // addLog(String("Battery voltage: ") + battery.getVoltage() + " V \tRaw ADC value: " + battery.getRaw() + " \t" + battery.getMillivolts() + " mV - BEC in V: " + v + " V\tI: " + a + " A");
    }

    void checkFailsafeTransition()
    {
        bool currentState = flysky.transmitter_on;

        if (lastTransmitterState && !currentState && !failsafeTriggered)
        {
            addLog(" ! FAILSAFE detected: switching to fallback mode");
            failsafeTriggered = true;
            enterFailsafeMode();
        }

        if (currentState && failsafeTriggered)
        {
            addLog(" ! Transmitter reconnected");
            failsafeTriggered = false;
            exitFailsafeMode();
        }
        lastTransmitterState = currentState;
    }

    void enterFailsafeMode()
    {
        addLog(" ! Entering autonomous mode...Stopping motors");
        engine.setState(engine.MOTOR_STOP);
    }

    void exitFailsafeMode()
    {
        addLog(" ! Returning to manual control");
    }

    void addLog(const String &msg) override
    {
        // Безопасное логирование для вывода в Serial (не требует мьютекса)
        Serial.print('[');
        Serial.print(timeStr());
        Serial.print("] ");
        Serial.println(msg);

        // Потокобезопасное управление логами с мьютексом
        if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Создаем копию сообщения для безопасности с временной меткой
            String safeCopy = "[" + timeStr() + "] " + msg;

            // Проверяем размер буфера и удаляем старые записи
            if (logBuffer.size() >= logCapacity)
            {
                // Безопасное удаление первого элемента
                if (!logBuffer.empty())
                {
                    logBuffer.erase(logBuffer.begin());
                }
            }

            logBuffer.push_back(safeCopy);
            xSemaphoreGive(logMutex);
        }
        // Если не удалось получить мьютекс за 10ms, пропускаем запись в буфер
        // но Serial-лог всё равно выводится
    }
    String getMotorConfigJson()
    {
        Preferences prefs;
        prefs.begin("boatcfg", true);
        JsonDocument doc;
        doc["left"]["esc"] = prefs.getString("l_esc", "uni");
        doc["left"]["sensor"] = prefs.getString("l_addr", "");
        doc["right"]["esc"] = prefs.getString("r_esc", "uni");
        doc["right"]["sensor"] = prefs.getString("r_addr", "");
        prefs.end();
        String result;
        serializeJson(doc, result);
        return result;
    }

    void setMotorConfigJson(const String &json)
    {
        flysky.stopIBus(); // Останавливаем iBUS, чтобы избежать конфликтов при записи в память
        Preferences prefs;
        Serial.println("Setting motor config from JSON: " + json);
        prefs.begin("boatcfg", false);
        JsonDocument doc;
        deserializeJson(doc, json);
        Serial.println("Disabled putting motor config to preferences");
        prefs.putString("l_esc", doc["left"]["esc"].as<String>());
        prefs.putString("l_addr", doc["left"]["sensor"].as<String>());
        prefs.putString("r_esc", doc["right"]["esc"].as<String>());
        prefs.putString("r_addr", doc["right"]["sensor"].as<String>());
        prefs.end();
        flysky.startIBus(); // Запускаем iBUS снова
        Serial.println("Finish");
    }

    String getMotorTempsJson()
    {
        Preferences prefs;
        prefs.begin("boatcfg", true);
        String leftAddr = prefs.getString("l_addr", "");
        String rightAddr = prefs.getString("r_addr", "");
        prefs.end();

        float leftTemp = temps.getByAddressString(leftAddr);
        float rightTemp = temps.getByAddressString(rightAddr);

        JsonDocument doc;
        doc["left"] = isnan(leftTemp) ? -127.0 : leftTemp;
        doc["right"] = isnan(rightTemp) ? -127.0 : rightTemp;

        String result;
        serializeJson(doc, result);
        return result;
    }

    String getAllSensorAddressesJson()
    {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (const auto &addr : temps.getAllAddresses())
        {
            arr.add(addr);
        }
        String result;
        serializeJson(doc, result);
        return result;
    }

    void sensorsPowerOn()
    {
        addLog("Start Sensors powered on");
        pinMode(VextCtrl, OUTPUT);
        digitalWrite(VextCtrl, LOW);
        addLog("Sensors powered on");
    }

    void sensorsPowerOff()
    {
        addLog("Start Sensors powered off");
        pinMode(VextCtrl, OUTPUT);
        digitalWrite(VextCtrl, HIGH);
        addLog("Sensors powered off");
    }

    void setThrottleLimit(int limit)
    {
        int pwmLimit = PWM_MIN;
        if (limit < 1250)
        {
            pwmLimit = PWM_MIN + 0.20 * (PWM_MAX - PWM_MIN); // 20%
        }
        else if (limit < 1750)
        {
            pwmLimit = PWM_MIN + 0.50 * (PWM_MAX - PWM_MIN); // 50%
        }
        else
        {
            pwmLimit = PWM_MIN + 0.80 * (PWM_MAX - PWM_MIN); // 80%
        }

        engine.setLimit(pwmLimit);
    }

    // Send response back to MissionControl
    void sendResponseToMissionControl(const String &response)
    {
        const size_t maxPayloadSize = MAX_LORA_PAYLOAD - 15; // Резерв для заголовка фрагмента

        if (response.length() <= maxPayloadSize)
        {
            // Короткое сообщение - отправляем как есть
            sendResponseFragment(response, 0, 1);
        }
        else
        {
            // Длинное сообщение - фрагментируем
            size_t totalLen = response.length();
            size_t chunks = (totalLen + maxPayloadSize - 1) / maxPayloadSize;

            // Безопасное логирование фрагментации
            char logBuffer[128];
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Response too long (%zu bytes), fragmenting into %zu parts",
                     totalLen, chunks);
            addLog(String(logBuffer));

            for (size_t i = 0; i < chunks; i++)
            {
                String fragment = response.substring(i * maxPayloadSize, (i + 1) * maxPayloadSize);

                // Безопасное формирование заголовка
                char headerBuffer[32];
                snprintf(headerBuffer, sizeof(headerBuffer), "[%zu/%zu]", i, chunks);
                String payload = String(headerBuffer) + fragment;

                sendResponseFragment(payload, i, chunks);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }

    void sendResponseFragment(const String &payload, size_t fragmentIndex, size_t totalFragments)
    {
        if (payload.length() > MAX_LORA_PAYLOAD)
        {
            // Безопасное логирование
            char logBuffer[128];
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Fragment %zu too large (%zu bytes), truncating to %d",
                     fragmentIndex, payload.length(), MAX_LORA_PAYLOAD);
            addLog(String(logBuffer));
        }

        size_t len = std::min<size_t>(payload.length(), MAX_LORA_PAYLOAD);

        // Create temporary buffer for the response
        uint8_t tempBuf[MAX_LORA_PAYLOAD];
        memcpy(tempBuf, payload.c_str(), len);

        PacketCommand responsePacket{};
        responsePacket.packetType = CMD_COMMAND_RESPONSE;
        responsePacket.packetId = nextPacketId++;
        responsePacket.payloadLen = len;

        // Send response to MissionControl
        loraComm->sendPacketBase(MISSION_CONTROL_ID, responsePacket, tempBuf);

        // Безопасное логирование результата
        char logBuffer[128];
        if (totalFragments > 1)
        {
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Fragment %zu/%zu sent (%zu bytes)",
                     fragmentIndex + 1, totalFragments, len);
        }
        else
        {
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Response sent (%zu bytes)", len);
        }
        addLog(String(logBuffer));
    }

    // Добавить ACK в bulk пакет (для лодки)
    void addToBulkAck(uint16_t packetId)
    {
        if (pendingBulkAck.addAck(packetId))
        {
            addLog("[BOAT] 📦 Added ACK for packet " + String(packetId) + " to bulk (" + String(pendingBulkAck.count) + "/10)");

            // Если пакет заполнен или прошло достаточно времени - отправляем
            if (pendingBulkAck.isFull() ||
                (millis() - lastBulkAckTime > BULK_ACK_MAX_WAIT_MS && !pendingBulkAck.isEmpty()))
            {
                sendBulkAck();
            }
        }
        else
        {
            // Пакет переполнен - отправляем текущий и начинаем новый
            sendBulkAck();
            if (pendingBulkAck.addAck(packetId)) {
                addLog("[BOAT] 📦 Started new bulk ACK with packet " + String(packetId));
            }
        }
    }

    // Отправить накопленные ACK (лодка -> MissionControl)
    void sendBulkAck()
    {
        if (pendingBulkAck.isEmpty())
            return;

        // Диагностика перед отправкой
        if (pendingBulkAck.hasDuplicates()) {
            addLog("[BOAT] ⚠️ WARNING: BULK ACK contains duplicates: " + pendingBulkAck.getDebugInfo());
        }

        pendingBulkAck.packetId = nextPacketId++;

        // Формируем payload: count + массив ID
        uint8_t payload[1 + 10 * sizeof(uint16_t)];
        payload[0] = pendingBulkAck.count;
        memcpy(&payload[1], pendingBulkAck.ackedIds, pendingBulkAck.count * sizeof(uint16_t));

        size_t payloadSize = sizeof(uint8_t) + (pendingBulkAck.count * sizeof(uint16_t));

        // Отправляем bulk ACK в MissionControl
        loraComm->sendPacketBase(MISSION_CONTROL_ID, pendingBulkAck, payload, false);
        addLog("[BOAT] ✅ Sent BULK ACK for " + String(pendingBulkAck.count) + " packets to MC: " + pendingBulkAck.getDebugInfo());

        pendingBulkAck.clear();
        lastBulkAckTime = millis();
    }

    // Проверить, нужно ли отправить bulk ACK по таймауту
    void checkBulkAckTimeout()
    {
        if (!pendingBulkAck.isEmpty() &&
            (millis() - lastBulkAckTime > BULK_ACK_INTERVAL_MS))
        {
            sendBulkAck();
        }
    }

private:
    // Helper function for INA3221 initialization
    void initINA3221(Adafruit_INA3221& sensor, uint8_t address, const String& name, float lowerLimit, float upperLimit)
    {
        if (!sensor.begin(address, &Wire))
        {
            addLog("-" + name + " not found at address 0x" + String(address, HEX));
        }
        else
        {
            addLog("+" + name + " found at address 0x" + String(address, HEX));
            sensor.setAveragingMode(INA3221_AVG_16_SAMPLES);
            for (uint8_t i = 0; i < 3; i++)
            {
                sensor.setShuntResistance(i, 0.1);
            }
            sensor.setPowerValidLimits(lowerLimit, upperLimit);
        }
    }

    float smoothedRssi = -120.0f;  // начальное значение
    const float rssiAlpha = 0.15f; // коэффициент сглаживания (0.1–0.3)

    // Система агрегированных ACK для лодки
    PacketBulkAck pendingBulkAck;
    unsigned long lastBulkAckTime = 0;
    static constexpr unsigned long BULK_ACK_INTERVAL_MS = 1000; // 1 секунда

    static constexpr unsigned long BULK_ACK_MAX_WAIT_MS = 500; // Максимум 0.5 сек ожидания

    float updateSmoothedRssi(float newRssi)
    {

        smoothedRssi = rssiAlpha * newRssi + (1.0f - rssiAlpha) * smoothedRssi;
        return smoothedRssi;
    }

    String inputBuffer;
    PacketId_t nextPacketId = 0;
    bool lastTransmitterState = true; // было ли соединение ранее
    bool failsafeTriggered = false;   // уже обработали событие потери
    static constexpr size_t logCapacity = 90;
    std::vector<String> logBuffer;
    SemaphoreHandle_t logMutex = nullptr; // Мьютекс для потокобезопасного логирования
    unsigned long lastPingSent = 0;
    unsigned long lastAdaptiveSwitchTime = 0;
    float PongRssi = 0; // RSSI при получении PONG
    bool missionCOntrolIsActivae = false;
    bool allowAdaptiveLoraSwitch = true; // Разрешение на автоматическое переключение профилей

    // 🚀 Command deduplication system
    struct LastCommand {
        PacketId_t packetId = 0;
        unsigned long timestamp = 0;
        String command = "";
    } lastProcessedCommand;
    static constexpr unsigned long COMMAND_DEDUP_TIMEOUT_MS = 2000; // 2 секунды для дедупликации

    // 🚀 NEW: Structured data management functions
    void updateBoatSettings() {
        unsigned long now = millis();
        if (now - lastDataUpdate < DATA_UPDATE_INTERVAL) {
            return;
        }
        
        bs.updateTimestamp();
        
        // GPS данные
        if (gnss.hasValidFix()) {
            bs.gps.position.latitude = gnss.getLatitude();
            bs.gps.position.longitude = gnss.getLongitude();
            bs.gps.position.altitude = 0.0f; // TODO: добавить altitude в GNSS
            bs.gps.position.timestamp = now;
            bs.gps.satelliteCount = gnss.getSatelliteCount();
            bs.gps.setFixQuality(GPSStatus::GPS_FIX); // Используем setter
            bs.gps.hasFix = gnss.hasValidFix();
        }
        
        // Моторы - используем текущие значения или заглушки
        bs.motors.leftPower = 0;  // TODO: добавить API в MotorEngineControl
        bs.motors.rightPower = 0; // TODO: добавить API в MotorEngineControl
        bs.motors.rudderAngle = rudder.getCurrentAngle();
        bs.motors.emergencyStop = false; // TODO: реализовать emergency stop
        
        // LoRa статус
        if (loraComm) {
            bs.lora.currentProfile = loraComm->getCurrentProfileIndex();
            bs.lora.rssi = loraComm->getRadio().getRSSI();
            bs.lora.snr = loraComm->getRadio().getSNR();
            bs.lora.adaptiveMode = allowAdaptiveLoraSwitch;
            bs.lora.missionControlConnected = missionCOntrolIsActivae;
            bs.lora.lastPacketTime = lastPacketReceived;
        }
        
        // Датчики и температуры
        bs.sensors.motor1Temp = temps.get(MOTOR1);
        bs.sensors.motor2Temp = temps.get(MOTOR2);
        bs.sensors.radiatorTemp = temps.get(MOTOR_RAD);
        bs.sensors.oilTemp = temps.get(OIL);
        bs.sensors.ambientTemp = temps.get(ENV);
        bs.sensors.batteryVoltage = battery.getVoltage();
        bs.sensors.batteryCurrent = 0.0f; // TODO: реализовать ток батареи
        bs.sensors.batteryPercent = (uint8_t)((battery.getVoltage() - 11.0f) / (12.6f - 11.0f) * 100.0f); // Примерная формула
        
        // Навигация
        bs.navigation.navigationActive = false; // TODO: проверить AutoNavigation API
        bs.navigation.mode = NavigationStatus::MANUAL;
        
        // Системная информация
        bs.uptime = millis() / 1000;
        bs.freeHeap = ESP.getFreeHeap();
        bs.systemHealth = calculateSystemHealth();
        
        lastDataUpdate = now;
    }
    
    uint8_t calculateSystemHealth() {
        uint8_t health = 100;
        
        // Снижаем здоровье системы при различных проблемах
        if (bs.sensors.batteryVoltage < 11.0f) health -= 30;
        if (bs.sensors.motor1Temp > 80.0f) health -= 20;
        if (bs.sensors.motor2Temp > 80.0f) health -= 20;
        if (!bs.gps.hasFix) health -= 15;
        if (!bs.lora.missionControlConnected) health -= 10;
        if (bs.freeHeap < 50000) health -= 15;
        
        return max(0, (int)health);
    }
    
    void sendStructuredHeartbeat() {
        if (!useStructuredData || !loraComm) return;
        
        updateBoatSettings();
        
        // Используем новую систему структурированных данных
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        
        uint8_t payload[StructuredDataManager::MAX_STRUCTURED_DATA_SIZE];
        size_t payloadSize;
        
        if (StructuredDataManager::createHeartbeatPacket(bs, packetBase, payload, payloadSize)) {
            loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
            addLog("📡 Structured Heartbeat sent: " + String(payloadSize) + " bytes, Health:" + String(bs.systemHealth) + "%");
            lastHeartbeatSent = millis(); // Обновляем время для статистики
        }
    }
    
    void sendStructuredFullStatus() {
        if (!useStructuredData || !loraComm) return;
        
        updateBoatSettings();
        
        // Отправляем отдельные компоненты системы
        sendStructuredGPS();
        sendStructuredMotors();
        sendStructuredSensors();
        
        lastFullStatusSent = millis(); // Обновляем время для статистики
    }
    
    void sendStructuredGPS() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        
        uint8_t payload[StructuredDataManager::MAX_STRUCTURED_DATA_SIZE];
        size_t payloadSize;
        
        if (StructuredDataManager::createGPSPacket(bs.gps, packetBase, payload, payloadSize)) {
            loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
            addLog("� GPS data sent: " + String(payloadSize) + " bytes");
        }
    }
    
    void sendStructuredMotors() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        
        uint8_t payload[StructuredDataManager::MAX_STRUCTURED_DATA_SIZE];
        size_t payloadSize;
        
        if (StructuredDataManager::createMotorPacket(bs.motors, packetBase, payload, payloadSize)) {
            loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
            addLog("⚙️ Motor data sent: " + String(payloadSize) + " bytes");
        }
    }
    
    void sendStructuredSensors() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        
        uint8_t payload[StructuredDataManager::MAX_STRUCTURED_DATA_SIZE];
        size_t payloadSize;
        
        if (StructuredDataManager::createSensorPacket(bs.sensors, packetBase, payload, payloadSize)) {
            loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
            addLog("🌡️ Sensor data sent: " + String(payloadSize) + " bytes");
        }
    }
    
    void sendStructuredLoRaStatus() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        packetBase.packetType = CMD_STATUS; // Используем существующую команду
        
        // Простая сериализация LoRa статуса
        uint8_t payload[LoRaStatus::serializedSize()];
        bs.lora.serialize(payload);
        
        packetBase.payloadLen = LoRaStatus::serializedSize();
        loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
        addLog("📡 LoRa status sent: " + String(packetBase.payloadLen) + " bytes");
    }
    
    void sendStructuredNavigation() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        packetBase.packetType = CMD_NAV; // Используем существующую команду
        
        // Простая сериализация навигационных данных
        uint8_t payload[NavigationStatus::serializedSize()];
        bs.navigation.serialize(payload);
        
        packetBase.payloadLen = NavigationStatus::serializedSize();
        loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
        addLog("🧭 Navigation data sent: " + String(packetBase.payloadLen) + " bytes");
    }
    
    void sendStructuredSystemInfo() {
        if (!loraComm) return;
        
        PacketBase packetBase;
        packetBase.packetId = nextPacketId++;
        packetBase.packetType = CMD_STATUS; // Используем существующую команду
        
        // Создаем временную структуру с системной информацией
        struct SystemInfo {
            uint32_t uptime;
            uint32_t freeHeap;
            uint8_t systemHealth;
            bool firmwareUpdateMode;
        } sysInfo;
        
        sysInfo.uptime = bs.uptime;
        sysInfo.freeHeap = bs.freeHeap;
        sysInfo.systemHealth = bs.systemHealth;
        sysInfo.firmwareUpdateMode = bs.firmwareUpdateMode;
        
        // Используем простую сериализацию для системной информации
        uint8_t payload[sizeof(sysInfo)];
        memcpy(payload, &sysInfo, sizeof(sysInfo));
        
        packetBase.payloadLen = sizeof(sysInfo);
        loraComm->sendPacketBase(MISSION_CONTROL_ID, packetBase, payload);
        addLog("💻 System info sent: " + String(packetBase.payloadLen) + " bytes, Health:" + String(sysInfo.systemHealth) + "%");
    }
    
    void processStructuredDataTransmission() {
        if (!useStructuredData) {
            return; // Используем старую JSON систему
        }
        
        sendStructuredHeartbeat();
        sendStructuredFullStatus();
    }

    SensorCache sensorCache;
    PerformanceMetrics performanceMetrics;
    StackMonitor stackMonitor;

    SafetyMonitor safetyMonitor;

    // Optimized sensor update function
    void updateSensorsOptimized()
    {
        static unsigned long lastSensorUpdate = 0;
        static unsigned long lastGnssUpdate = 0;
        static unsigned long lastCacheUpdate = 0;

        unsigned long now = millis();

        // Battery and basic sensors - каждые 50ms (20Hz)
        if (now - lastSensorUpdate >= 50)
        {
            battery.prepareForRead();
            battery.readVoltage();
            lastSensorUpdate = now;
        }

        // GNSS - каждые 200ms (5Hz) для экономии ресурсов
        if (now - lastGnssUpdate >= 200)
        {
            gnss.update();
            lastGnssUpdate = now;
        }

        // LoRa cache - каждые 100ms (10Hz)
        if (now - lastCacheUpdate >= 100 && !sensorCache.isValid() && loraComm)
        {
            float rssi = loraComm->getRadio().getRSSI();
            float snr = loraComm->getRadio().getSNR();
            sensorCache.update(rssi, snr);
            lastCacheUpdate = now;
        }
    }

    // Separate LoRa packet processing with stack protection
    void processLoRaPackets()
    {
        static unsigned long lastPacketCheck = 0;
        if (millis() - lastPacketCheck >= 10)
        { // 100Hz для LoRa
            // Use LoRaCore API instead of direct LoRaComm calls
            LoRaPacket pkt = {}; // Explicitly initialize to zero
            if (loraComm->receive(pkt) && pkt.senderId != BOAT_DEVICE_ID)
            {
                // 💗 ЛЮБОЙ пакет от MissionControl сбрасывает таймер heartbeat
                if (pkt.senderId == MISSION_CONTROL_ID) {
                    lastPacketReceived = millis();
                }
                // Process packet with optimized stack usage
                handleLoRaPacketOptimized(pkt);
            }
            lastPacketCheck = millis();
        }
    }

    // Lightweight packet handler with reduced stack usage
    void handleLoRaPacketOptimized(const LoRaPacket &pkt)
    {
        // Use minimal local variables to save stack
        uint8_t senderId = pkt.senderId;
        
        // Validate payload length first to prevent memory corruption
        if (pkt.payloadLen > MAX_LORA_PAYLOAD) {
            addLog("❌ Invalid packet: payload length " + String(pkt.payloadLen) + 
                   " exceeds maximum " + String(MAX_LORA_PAYLOAD));
            return;
        }
        
        PacketBase hdr;
        hdr.packetType = pkt.packetType;
        hdr.packetId = pkt.packetId;
        hdr.payloadLen = pkt.payloadLen;
        const uint8_t *buf = pkt.payload;

        // Get RSSI and SNR for all packets (like in oldHandlePacket)
        float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();
        
        // Log packet info like in oldHandlePacket with safe formatting
        char sd[256];
        snprintf(sd, sizeof(sd),
                 "[P:%d][RSSI:%.2f/%.2fdBm,SNR=%.1fdB] T=%c from=%d id=%d len=%d",
                 loraComm->getCurrentProfileIndex(), rssi, smoothedRssi, snr,
                 (char)hdr.packetType, senderId, hdr.packetId, hdr.payloadLen);
        addLog(String(sd));

        // Handle MissionControl specific processing (like in oldHandlePacket)
        if (pkt.senderId == MISSION_CONTROL_ID)
        {
            // lastPacketReceived уже обновлен выше для ЛЮБОГО пакета от MissionControl
            PongRssi = updateSmoothedRssi(rssi);
            if(!missionCOntrolIsActivae){
                smoothedRssi = rssi;
                PongRssi = rssi;
                
                // 🚀 AUTO-ADAPTATION: Mission Control reconnected - trigger adaptive update after this packet
                addLog("🔄 Mission Control reconnected - scheduling adaptive LoRa update");
                // Set a flag to trigger adaptation in next loop cycle
                lastAdaptiveSwitchTime = millis() - ADAPTIVE_SWITCH_INTERVAL; // Force immediate adaptation
                sensorCache.valid = false; // Force fresh readings
                
                // 🔄 Включаем адаптивное переключение при подключении пульта
                allowAdaptiveLoraSwitch = true;
                addLog("✅ Adaptive LoRa switching enabled (MissionControl connected)");
            }
            missionCOntrolIsActivae = true;
        }

        // Quick check for critical commands (emergency stop, etc.)
        if (hdr.packetType == CMD_COMMAND_STRING)
        {
            // Handle critical commands immediately with minimal stack usage
            processCommandPacket(hdr, buf, senderId);
        }
        else if (hdr.packetType == CMD_PING || hdr.packetType == CMD_PONG)
        {
            // Handle ping/pong immediately (lightweight)
            processPingPongPacket(hdr, senderId);
        }
        else
        {
            // Process other packets immediately to match oldHandlePacket behavior
            processGeneralPacket(hdr, buf, senderId);
        }
    }

    // Control and navigation updates with reduced frequency
    void updateControlAndNavigation()
    {
        static unsigned long lastControlUpdate = 0;
        static unsigned long lastNavUpdate = 0;

        unsigned long now = millis();

        // Basic control - каждые 20ms (50Hz)
        if (now - lastControlUpdate >= 20)
        {
            // Minimal scope для экономии стека
            {
                // Basic engine state updates only
                // Heavy operations moved to main loop
            }
            lastControlUpdate = now;
        }

        // Navigation - каждые 100ms (10Hz) для сложных вычислений
        if (now - lastNavUpdate >= 100)
        {
            updateNavigationSystem();
            lastNavUpdate = now;
        }
    }

    // Navigation system update with stack optimization
    void updateNavigationSystem()
    {
        // Only update if auto navigation is active and we have valid GNSS
        if (autoNav.getMode() == AutoNavigation::MANUAL || !gnss.hasValidFix())
        {
            return;
        }

        // Log GPS time sync status periodically
        static unsigned long lastTimeSyncCheck = 0;
        if (millis() - lastTimeSyncCheck > 30000)
        { // Every 30 seconds
            time_t now = time(nullptr);
            bool timeSynced = (now > 1600000000);
            addLog("🕒 GPS Time Status: " + String(timeSynced ? "SYNCED" : "NOT_SYNCED") +
                   " (GPS updated: " + String(gnss.isTimeUpdated() ? "YES" : "NO") + ")");
            lastTimeSyncCheck = millis();
        }

        // Use local variables sparingly to save stack
        float lat = gnss.getLatitude();
        float lon = gnss.getLongitude();
        float heading = gnss.getHeading();
        float speed = gnss.getSpeed();

        auto navOutput = autoNav.update(lat, lon, heading, speed);

        if (navOutput.navigationActive && !failsafeTriggered)
        {
            // Apply navigation commands
            if (navOutput.rudderAngle != 0)
            {
                rudder.setAngle((int)navOutput.rudderAngle);
            }

            // Log navigation status periodically
            static unsigned long lastNavLog = 0;
            if (millis() - lastNavLog > 5000)
            {
                addLog("[NAV] " + navOutput.statusMessage +
                       " R:" + String(navOutput.rudderAngle, 1) +
                       "° T:" + String(navOutput.throttle, 1) + "%");
                lastNavLog = millis();
            }
        }
    }

    // Function to get current time string from system time (synced from GNSS)
    String timeStr()
    {
        time_t now = time(nullptr);
        unsigned long currentMillis = millis();
        uint16_t milliseconds = currentMillis % 1000;

        if (now > 1600000000) // Check if time is valid (after 2020)
        {
            struct tm *t = localtime(&now);
            char buf[13];
            strftime(buf, 9, "%H:%M:%S", t);
            char fullBuf[13];
            snprintf(fullBuf, sizeof(fullBuf), "%s.%03d", buf, milliseconds);
            return String(fullBuf);
        }

        // If system time is not synced, show millis with indicator
        char defaultBuf[16];
        snprintf(defaultBuf, sizeof(defaultBuf), "~%02lu:%02lu.%03d",
                 (currentMillis / 60000) % 60,
                 (currentMillis / 1000) % 60,
                 milliseconds);
        return String(defaultBuf);
    }

    // Helper functions for packet processing with minimal stack usage
    void processCommandPacket(const PacketBase &hdr, const uint8_t *buf, uint8_t senderId)
    {
        // Validate payload length to prevent memory corruption
        if (hdr.payloadLen > MAX_LORA_PAYLOAD) {
            addLog("❌ Command packet: Invalid payload length " + String(hdr.payloadLen));
            return;
        }
        
        PacketCommand cmd{};
        cmd.packetType = hdr.packetType;
        cmd.packetId = hdr.packetId;
        cmd.payloadLen = hdr.payloadLen;
        
        // Safe copy with bounds checking
        size_t copyLen = std::min<size_t>(hdr.payloadLen, MAX_LORA_PAYLOAD);
        memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase), buf, copyLen);

        // Convert to string with minimal stack usage (matching oldHandlePacket)
        String s;
        s.reserve(copyLen + 1);
        for (size_t i = 0; i < copyLen; ++i)
        {
            s += char(buf[i]);
        }
        
        // Log the command like in oldHandlePacket
        addLog("Got COMMND: TYPE" + (char)cmd.packetType + String(":") + s);
        
        // 🚀 Command deduplication check
        unsigned long now = millis();
        if (lastProcessedCommand.packetId == cmd.packetId && 
            lastProcessedCommand.command == s &&
            (now - lastProcessedCommand.timestamp) < COMMAND_DEDUP_TIMEOUT_MS) {
            addLog("🔄 Skipping duplicate command: " + s + " (packet " + String(cmd.packetId) + 
                   ", " + String(now - lastProcessedCommand.timestamp) + "ms ago)");
            return;
        }
        
        // Update deduplication tracking
        lastProcessedCommand.packetId = cmd.packetId;
        lastProcessedCommand.command = s;
        lastProcessedCommand.timestamp = now;
        
        // Process the command
        parser.processLine(s);

        // Update RSSI and mission control status (no need to duplicate here as it's done in handleLoRaPacketOptimized)
    }

    void processPingPongPacket(const PacketBase &hdr, uint8_t senderId)
    {
        // RSSI handling is already done in handleLoRaPacketOptimized for MissionControl

        if (hdr.packetType == CMD_PING)
        {
            addLog("Got CMD_PING: smoothedRssi " + String(smoothedRssi) + "dBm");
            
            // 🚀 FIX: Send PONG response to PING
            PacketCommand pong{};
            pong.packetType = CMD_PONG;
            pong.packetId = nextPacketId++;
            pong.payloadLen = 0;
            
            loraComm->sendPacketBase(senderId, pong, nullptr, false);
            addLog("📥 Received PING from " + String(senderId) + ", sent PONG response");
        }
        else if (hdr.packetType == CMD_PONG)
        {
            // PONG - важная команда, оставляем лог для диагностики связи (matching oldHandlePacket)
            addLog("Got CMD_PONG");
            
            // 🚀 AUTO-ADAPTATION: При получении PONG от MissionControl запускаем адаптацию
            if (senderId == MISSION_CONTROL_ID && !waitingForASAAck) {
                // Invalidate cache to force fresh RSSI/SNR readings
                sensorCache.valid = false;
                
                // Trigger immediate adaptive update for faster profile optimization
                adaptiveLoraUpdate();
                addLog("🔄 Auto-triggered adaptive update after PONG from MC");
            }
        }
    }

    void processGeneralPacket(const PacketBase &hdr, const uint8_t *buf, uint8_t senderId)
    {
        // Validate payload length to prevent memory corruption
        if (hdr.payloadLen > MAX_LORA_PAYLOAD) {
            addLog("❌ General packet: Invalid payload length " + String(hdr.payloadLen));
            return;
        }
        
        // This function handles non-critical packets with full processing
        // to match the complete functionality from oldHandlePacket
        size_t safeLen = std::min<size_t>(hdr.payloadLen, MAX_LORA_PAYLOAD);

        switch (hdr.packetType)
        {
        case CMD_TELEMETRY_FRAGMENT:
        {
            addLog("Got CMD_TELEMETRY_FRAGMENT");
            PacketTelemetry tel{};
            tel.packetType = hdr.packetType;
            tel.packetId = hdr.packetId;
            tel.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&tel) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onTelemetry(tel);
            break;
        }
        case CMD_INFO_ENGINE:
        {
            addLog("Got CMD_INFO_ENGINE");
            PacketInfoEngine info{};
            info.packetType = hdr.packetType;
            info.packetId = hdr.packetId;
            info.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&info) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onInfoEngine(info);
            break;
        }
        case CMD_STATUS:
        {
            addLog("Got CMD_STATUS");
            PacketStatus st{};
            st.packetType = hdr.packetType;
            st.packetId = hdr.packetId;
            st.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&st) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onStatus(st);
            break;
        }
        case CMD_ACK:
        {
            addLog("Got CMD_ACK");
            PacketAck ackIn{};
            ackIn.packetType = hdr.packetType;
            ackIn.packetId = hdr.packetId;
            ackIn.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&ackIn) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onAck(ackIn);
            break;
        }
        case CMD_CONFIG:
        {
            addLog("Got CMD_CONFIG");
            PacketConfig cfg{};
            cfg.packetType = hdr.packetType;
            cfg.packetId = hdr.packetId;
            cfg.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&cfg) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onConfig(cfg);
            break;
        }
        case CMD_NAV:
        {
            addLog("Got CMD_NAV");
            PacketNav nav{};
            nav.packetType = hdr.packetType;
            nav.packetId = hdr.packetId;
            nav.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&nav) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onNav(nav);
            break;
        }
        case CMD_HEARTBEAT:
        {
            addLog("Got CMD_HEARTBEAT");
            PacketHeartbeat hb{};
            hb.packetType = hdr.packetType;
            hb.packetId = hdr.packetId;
            hb.payloadLen = hdr.payloadLen;
            memcpy(reinterpret_cast<uint8_t *>(&hb) + sizeof(PacketBase),
                   buf,
                   safeLen);
            onHeartbeat(hb);
            break;
        }
        case CMD_REPOSNCE_ASA:
            handleASAResponse(hdr, buf);
            break;
        case CMD_GET_BOAT_STATUS:
            addLog("Got CMD_GET_BOAT_STATUS");
            sendStatusJsonFragmentsTask();
            break;
        case CMD_REQUEST_INFO:
            handleInfoRequest(hdr, buf, senderId);
            break;
        case CMD_BULK_ACK:
        {
            // Обработка bulk ACK от Mission Control (full implementation from oldHandlePacket)
            if (hdr.payloadLen < sizeof(uint8_t))
            {
                addLog("[BOAT] ⚠️ Invalid BULK ACK packet size");
                break;
            }

            uint8_t count = buf[0];
            if (count > 10 || hdr.payloadLen != sizeof(uint8_t) + (count * sizeof(uint16_t)))
            {
                addLog("[BOAT] ⚠️ Invalid BULK ACK count: " + String(count));
                break;
            }

            addLog("[BOAT] ✅ Received BULK ACK for " + String(count) + " packets from MC");
            for (uint8_t i = 0; i < count; i++)
            {
                uint16_t ackedId;
                memcpy(&ackedId, &buf[1 + i * sizeof(uint16_t)], sizeof(uint16_t));
                addLog("[BOAT] 📨 Confirmed packet ID: " + String(ackedId));
            }
            break;
        }
        // 🚀 NEW: Structured data packet handling
        case CMD_STRUCTURED_HEARTBEAT:
        {
            UltraCompactHeartbeat hb;
            if (StructuredDataManager::parseHeartbeat(buf, hdr.payloadLen, hb)) {
                addLog("📡 Ultra heartbeat: " + hb.toString());
                // Обновляем информацию о MissionControl
                if (senderId == MISSION_CONTROL_ID) {
                    lastPacketReceived = millis();
                    missionCOntrolIsActivae = true;
                }
                
                // Статистика размеров пакетов - печатать раз в 30 секунд
                static unsigned long lastStatsOutput = 0;
                if (millis() - lastStatsOutput > 30000) {
                    addLog("📊 " + StructuredDataManager::getStats());
                    lastStatsOutput = millis();
                }
            } else {
                addLog("❌ Failed to parse ultra heartbeat");
            }
            break;
        }
        case CMD_STRUCTURED_GPS:
        {
            GPSStatus gps;
            if (StructuredDataManager::parseGPS(buf, hdr.payloadLen, gps)) {
                addLog("📍 Received GPS data: " + gps.toString());
            } else {
                addLog("❌ Failed to parse GPS data");
            }
            break;
        }
        case CMD_STRUCTURED_MOTORS:
        case CMD_STRUCTURED_SENSORS:
        case CMD_STRUCTURED_NAVIGATION:
        case CMD_STRUCTURED_FRAGMENT:
        {
            addLog("📦 Received structured packet type: " + String((char)hdr.packetType) + ", size: " + String(hdr.payloadLen));
            // TODO: Implement parsing for other structured packet types
            break;
        }
        default:
            addLog("Got Unknown LoRa cmd: " + String((char)hdr.packetType));
            break;
        }

        // Send ACK for non-ACK packets (matching oldHandlePacket logic)
        if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_BULK_ACK && 
            hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG && 
            hdr.packetType != CMD_RSSI_REPORT)
        {
            // Используем bulk ACK вместо мгновенной отправки
            addToBulkAck(hdr.packetId);
            // lastPacketReceived уже обновлен выше для пакетов от MissionControl
        }
    }

    void handleASAResponse(const PacketBase &hdr, const uint8_t *buf)
    {
        addLog("Got CMD_REPOSNCE_ASA");
        waitingForASAAck = false;
        
        // Удаляем ASA запрос из pending списка, чтобы избежать ретраев
        if (lastAsaRequestId != 0 && loraComm) {
            if (loraComm->removePendingPacket(lastAsaRequestId)) {
                addLog("🗑️ Removed ASA request id=" + String(lastAsaRequestId) + " from pending (got response)");
            }
            lastAsaRequestId = 0;
        }
        
        asaActive = true;
        asaLastSwitchTime = millis();

        if (hdr.payloadLen == sizeof(uint8_t))
        {
            uint8_t profileIndex = buf[0];
            uint8_t currentProfile = loraComm->getCurrentProfileIndex();
            
            // 🚨 КРИТИЧЕСКИЙ БАГ ФИХ: Проверяем валидность предложенного профиля
            if (profileIndex >= LORA_PROFILE_COUNT) {
                addLog("❌ ASA: Invalid profile index " + String(profileIndex) + ", ignoring");
                return;
            }
            
            // Анализируем тип перехода
            String transitionType;
            if (profileIndex > currentProfile) {
                transitionType = "upgrade";
            } else if (profileIndex < currentProfile) {
                transitionType = "downgrade";
            } else {
                transitionType = "same profile";
            }
            
            addLog("✅ ASA response: " + transitionType + " " + 
                   String(currentProfile) + "→" + String(profileIndex));
            
            // Проверяем, нужен ли такой переход на основе текущих условий
            float currentRssi = PongRssi; // Используем последний известный RSSI
            int optimalProfile = selectOptimalProfile(currentRssi, 15.0f, false); // Тихая проверка
            
            // 🔒 ЗАЩИТА: Отклоняем переход на профиль 0 с хорошего профиля, если сигнал хороший
            if (profileIndex == 0 && currentProfile >= 8 && currentRssi > -60.0f) {
                addLog("🛡️ ASA: Rejecting downgrade to profile 0 with good signal " + 
                       String(currentRssi) + "dBm, staying on " + String(currentProfile));
                return;
            }
            
            // 🎯 ВАЛИДАЦИЯ: Проверяем, соответствует ли предложенный профиль оптимальному
            if (abs(profileIndex - optimalProfile) > 2) {
                addLog("⚠️ ASA: Proposed profile " + String(profileIndex) + 
                       " differs from optimal " + String(optimalProfile) + 
                       " (RSSI=" + String(currentRssi) + "), but applying anyway");
            }
            
            applyProfile(profileIndex);
        }
        else
        {
            addLog("⚠️ Invalid ASA payload size");
        }
    }

    void handleAckReceived(PacketId_t packetId, uint8_t senderId, uint8_t originalPacketType)
    {
        // Логируем все ACK для отладки
        char typeStr[16];
        snprintf(typeStr, sizeof(typeStr), "0x%02X", originalPacketType);
        addLog("🔔 ACK callback: id=" + String(packetId) + ", from=" + String(senderId) + 
               ", origType=" + String(typeStr) + " (" + String((char)originalPacketType) + ")");
        
        // Обрабатываем ACK для CMD_REQUEST_ASA
        if (originalPacketType == CMD_REQUEST_ASA && waitingForASAAck)
        {
            addLog("✅ Got ACK for ASA request id=" + String(packetId) + " from device " + String(senderId));
            waitingForASAAck = false;
            // НЕ применяем профиль здесь - ждем CMD_REPOSNCE_ASA
        }
        
        // Можно добавить обработку других типов пакетов при необходимости
        // if (originalPacketType == CMD_SOME_OTHER && someCondition) { ... }
    }

    void handleInfoRequest(const PacketBase &hdr, const uint8_t *buf, uint8_t senderId)
    {
        // RSSI handling is already done in handleLoRaPacketOptimized for MissionControl
        addLog("SPECIAL Got CMD_REQUEST_INFO");

        CommandType what = static_cast<CommandType>(buf[0]);
        switch (what)
        {
        case CMD_BOAT_STATUS_REPORT:
            sendStatusJsonFragmentsTask();
            break;
        case CMD_INFO_ENGINE:
        {
            PacketInfoEngine info{};
            info.packetType = CMD_INFO_ENGINE;
            info.packetId = nextPacketId++;
            info.payloadLen = 0;
            loraComm->sendPacketBase(senderId, info, nullptr);
            break;
        }
        case CMD_CONFIG:
        {
            PacketConfig cfg{};
            cfg.packetType = CMD_CONFIG;
            cfg.packetId = nextPacketId++;
            cfg.payloadLen = 0;
            loraComm->sendPacketBase(senderId, cfg, nullptr);
            break;
        }
        case CMD_NAV:
        {
            PacketNav nav{};
            nav.packetType = CMD_NAV;
            nav.packetId = nextPacketId++;
            loraComm->sendPacketBase(senderId, nav, nullptr);
            break;
        }
        default:
            addLog("Boat: Unsupported info request '" + String((char)what) + "'");
            break;
        }
    }

    void sendAckPacket(uint16_t ackedId, uint8_t receiverId)
    {
        PacketAck ackOut{};
        ackOut.packetType = CMD_ACK;
        ackOut.packetId = nextPacketId++;
        ackOut.ackedId = ackedId;
        ackOut.payloadLen = sizeof(ackOut.ackedId);

        uint8_t ackBuf[sizeof(ackOut.ackedId)];
        memcpy(ackBuf, &ackOut.ackedId, sizeof(uint16_t));

        loraComm->sendPacketBase(receiverId, ackOut, ackBuf, false);
    }
};
