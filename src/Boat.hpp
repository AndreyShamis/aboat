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
#include "lora_comm.hpp"
#include <Adafruit_PWMServoDriver.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"  // Для мьютексов FreeRTOS
#include "PacketClasses.hpp"

#include "LoRaCore.hpp"
#include "PacketAsaExchange.hpp"
#include "boat_utils.hpp"
#include "auto_navigation.hpp"
#include "stack_monitor.hpp"

using namespace ArduinoJson;

// ============================================================================
// PERFORMANCE OPTIMIZATION CONSTANTS
// ============================================================================
static constexpr unsigned long CHANNEL_PRINT_INTERVAL = 2000;
static constexpr unsigned long CONTROL_INTERVAL = 100; // 10 Hz
static constexpr unsigned long RSSI_REPORT_INTERVAL = 35000;
static constexpr unsigned long PING_INTERVAL = 20000;
static constexpr unsigned long ASA_TIMEOUT = 15000;
static constexpr unsigned long ACTIVITY_TIMEOUT = 45000;
static constexpr unsigned long ADAPTIVE_SWITCH_INTERVAL = 25011;

// Cache для оптимизации частых операций
struct SensorCache {
    float lastRSSI = -120.0f;
    float lastSNR = 0.0f;
    unsigned long lastUpdate = 0;
    bool valid = false;
    
    static constexpr unsigned long CACHE_VALIDITY_MS = 100; // 100ms cache
    
    bool isValid() const {
        return valid && (millis() - lastUpdate) < CACHE_VALIDITY_MS;
    }
    
    void update(float rssi, float snr) {
        lastRSSI = rssi;
        lastSNR = snr;
        lastUpdate = millis();
        valid = true;
    }
};

// Performance metrics для мониторинга
struct PerformanceMetrics {
    unsigned long keepCycleCount = 0;
    unsigned long maxKeepDuration = 0;
    unsigned long totalKeepTime = 0;
    unsigned long lastResetTime = 0;
    
    void recordKeepCycle(unsigned long duration) {
        keepCycleCount++;
        totalKeepTime += duration;
        if (duration > maxKeepDuration) {
            maxKeepDuration = duration;
        }
    }
    
    float getAverageKeepTime() const {
        return keepCycleCount > 0 ? (float)totalKeepTime / keepCycleCount : 0.0f;
    }
    
    void reset() {
        keepCycleCount = 0;
        maxKeepDuration = 0;
        totalKeepTime = 0;
        lastResetTime = millis();
    }
};

// ============================================================================
// MAIN BOAT CLASS WITH OPTIMIZATIONS
// ============================================================================
static unsigned long lastChannelPrint = 0;
static unsigned long lastRandomRudderTime = 0;
static unsigned long nextRandomRudderDelay = 0;
static unsigned long lastControlUpdate = 0;

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
    LoRaComm *loraComm; // Новый объект для LoRa связи
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

    TaskHandle_t sendStatusTaskHandle = nullptr;
    bool statusTaskRunning = false;

    bool waitingForASAAck = false;
    bool asaActive = false;
    unsigned long asaProposalTime = 0;
    unsigned long lastPacketReceived = 0; // обновляется при каждом принятом пакете
    unsigned long asaLastSwitchTime = 0;
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
        loraComm = new LoRaComm(BOAT_DEVICE_ID, this);
        // Создаем мьютекс для потокобезопасного логирования
        logMutex = xSemaphoreCreateMutex();
        if (logMutex == nullptr) {
            Serial.println("ERROR: Failed to create log mutex!");
        }
    }
    ~Boat()
    {
        delete loraComm;
        // Освобождаем мьютекс
        if (logMutex != nullptr) {
            vSemaphoreDelete(logMutex);
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

        if (!ina3221.begin(0X41, &Wire)) // Инициализация INA3221 на адресе 0x41
        {
            addLog("-INA3221 not found at address 0x41");
        }
        else
        {
            addLog("+INA3221 found at address 0x41");
            ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);
            for (uint8_t i = 0; i < 3; i++)
            {
                ina3221.setShuntResistance(i, 0.1);
            }

            // Set a power valid alert to tell us if ALL channels are between the two
            // limits:
            ina3221.setPowerValidLimits(4.0 /* lower limit */, 18.0 /* upper limit */);
        }
        if (!ina3221_low.begin(0X43, &Wire)) // Инициализация INA3221 на адресе 0x43
        {
            addLog("-INA3221 LOW not found at address 0x43");
        }
        else
        {
            addLog("+INA3221v LOW found at address 0x43");
            ina3221_low.setAveragingMode(INA3221_AVG_16_SAMPLES);
            for (uint8_t i = 0; i < 3; i++)
            {
                ina3221.setShuntResistance(i, 0.1);
            }

            // Set a power valid alert to tell us if ALL channels are between the two
            // limits:
            ina3221_low.setPowerValidLimits(3.2 /* lower limit */, 10.0 /* upper limit */);
        }
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

        // Enhanced diagnostic commands
        parser.registerCommand("D", [this](const String &arg)
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
                                        // Full diagnostic report - build components safely
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
                                        response += fullDiag;
                                    } else if (arg == "extended" || arg == "ext") {
                                        // Extended diagnostic report with more details
                                        String extDiag = "System:OK;";
                                        extDiag += "Uptime:" + String(millis()/1000) + "s;";
                                        extDiag += "FreeHeap:" + String(ESP.getFreeHeap()) + ";";
                                        extDiag += "MinFreeHeap:" + String(ESP.getFlashChipSize()) + ";";
                                        extDiag += "FlashSize:" + String(ESP.getFlashChipSize()) + ";";
                                        extDiag += "CPUFreq:" + String(ESP.getCpuFreqMHz()) + "MHz;";
                                        extDiag += "ChipModel:" + String(ESP.getChipModel()) + ";";
                                        extDiag += "TempSensors:" + String(temps.getSensorCount()) + ";";
                                        extDiag += "WiFiStatus:" + String(WiFi.status()) + ";";
                                        extDiag += "LoRaProfile:" + String(currentProfileIndex) + ";";
                                        extDiag += "Safety:" + safetyMonitor.getStatusString() + ";";
                                        extDiag += "Performance:avg=" + String(performanceMetrics.getAverageKeepTime()) + "ms;";
                                        extDiag += "Stack:" + stackMonitor.getCompactStatus();
                                        
                                        addLog("[DIAG] Extended diagnostic (" + String(extDiag.length()) + " bytes): " + extDiag);
                                        response += extDiag;
                                    } else if (arg == "stack" || arg == "st") {
                                        // Stack diagnostic report
                                        String stackReport = stackMonitor.getReport();
                                        addLog("[DIAG] Stack report:");
                                        addLog(stackReport);
                                        response += "Stack Report:\n" + stackReport;
                                    } else {
                                        response += "Unknown diagnostic command:" + arg;
                                    }
                                    
                                    // Send response back to MissionControl
                                    sendResponseToMissionControl(response);
                               });

        parser.registerCommand("L", [this](const String &arg)
                               {
                                    String response = "L:";
                                    if (arg == "S" || arg == "status") {
                                        String status = "Profile:" + String(currentProfileIndex) + 
                                               ",RSSI:" + String(smoothedRssi) + "dBm";
                                        addLog("[LORA] " + status);
                                        response += status;
                                    } else if (arg.startsWith("P") || arg.startsWith("profile:")) {
                                        String profileStr = arg.startsWith("profile:") ? arg.substring(8) : arg.substring(1);
                                        int profileIndex = profileStr.toInt();
                                        if (profileIndex >= 0 && profileIndex < LORA_PROFILE_COUNT) {
                                            applyProfile(profileIndex);
                                            addLog("[LORA] Manually switched to profile " + String(profileIndex));
                                            response += "Profile changed to " + String(profileIndex);
                                        } else {
                                            response += "Invalid profile index:" + String(profileIndex);
                                        }
                                    } else if (arg == "adapt") {
                                        // Trigger adaptive LoRa
                                        adaptiveLoraUpdate();
                                        response += "Adaptive LoRa triggered";
                                    } else {
                                        response += "Current profile:" + String(currentProfileIndex) + ",RSSI:" + String(smoothedRssi) + "dBm";
                                    }
                                    
                                    // Send response back to MissionControl
                                    sendResponseToMissionControl(response);
                               });

        // Navigation commands
        parser.registerCommand("N", [this](const String &arg)
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
                                        // Set current position as home
                                        if (gnss.hasValidFix()) {
                                            autoNav.setHome(gnss.getLatitude(), gnss.getLongitude());
                                            String homePos = String(gnss.getLatitude(), 6) + "," + String(gnss.getLongitude(), 6);
                                            addLog("[NAV] Home position set: " + homePos);
                                            response += "Home set at " + homePos;
                                        } else {
                                            addLog("[NAV] Cannot set home - no GPS fix");
                                            response += "Error: No GPS fix available";
                                        }
                                    } else if (arg.startsWith("start:")) {
                                        // Start navigation to specific coordinates: start:lat,lon
                                        String coords = arg.substring(6);
                                        int commaIndex = coords.indexOf(',');
                                        if (commaIndex > 0) {
                                            float lat = coords.substring(0, commaIndex).toFloat();
                                            float lon = coords.substring(commaIndex + 1).toFloat();
                                            autoNav.setTarget(lat, lon);
                                            autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
                                            addLog("[NAV] Navigation started to: " + String(lat, 6) + "," + String(lon, 6));
                                            response += "Navigation started to " + String(lat, 6) + "," + String(lon, 6);
                                        } else {
                                            response += "Error: Invalid coordinates format";
                                        }
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
                                        String mode = arg.substring(5);
                                        if (mode == "manual") autoNav.setMode(AutoNavigation::MANUAL);
                                        else if (mode == "waypoint") autoNav.setMode(AutoNavigation::WAYPOINT_FOLLOWING);
                                        else if (mode == "home") autoNav.setMode(AutoNavigation::RETURN_TO_HOME);
                                        else if (mode == "station") autoNav.setMode(AutoNavigation::STATION_KEEPING);
                                        response += "Mode set to " + mode;
                                    } else {
                                        String status = autoNav.getStatusString();
                                        addLog("[NAV] Status: " + status);
                                        response += "Status:" + status;
                                    }
                                    
                                    // Send response back to MissionControl
                                    sendResponseToMissionControl(response);
                               });

        parser.registerCommand("W", [this](const String &arg)
                               {
                                    String response = "W:";
                                    if (arg.startsWith("A") || arg.startsWith("add:")) {
                                        // Format: WA55.123456,37.654321 or W add:55.123456,37.654321
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
                                        // Web interface status
                                        response += "Web interface active,clients:" + String(WiFi.softAPgetStationNum());
                                    } else if (arg == "update") {
                                        // Update web interface (placeholder)
                                        response += "Web interface updated";
                                    } else {
                                        response += "Waypoint count:" + String(autoNav.getWaypointCount());
                                    }
                                    
                                    // Send response back to MissionControl
                                    sendResponseToMissionControl(response);
                               });

        addLog("Commands registered: M (motor), E (engine), R (rudder), P (oil pump), D (diagnostics), L (LoRa), N (navigation), W (waypoints)");

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
            // loraAdapt = new AdaptiveLoRaManager(loraComm, this);
            // loraAdapt->begin();
            addLog(" + LoRaComm успешно инициализирован.");
            LoRaCore::init(loraComm, this); // запускает задачи на втором ядре
        }

        addLog("Strarting setup iBUS FlySky...");
        flysky.begin();
        addLog("Boat initialized successfully.");
        
        // Инициализация мониторинга стека
        stackMonitor.autoRegisterTasks();
        addLog("Stack monitor initialized with " + String(stackMonitor.getTaskCount()) + " tasks");
    }

    unsigned long lastSync = 0;

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
        
        if (millis() - lastSafetyCheck >= 100) { // 10Hz для безопасности
            safetyMonitor.checkSystemHealth(temps, battery, lastPacketReceived);
            
            // Emergency shutdown if critical conditions are met
            if (safetyMonitor.overtemperatureShutdown || safetyMonitor.lowVoltageShutdown) {
                engine.setState(engine.MOTOR_STOP);
                addLog("⚠️ EMERGENCY SHUTDOWN: " + safetyMonitor.getStatusString());
            }
            lastSafetyCheck = millis();
        }
        
        // Stack monitoring - каждые 10 секунд
        if (millis() - lastStackCheck >= 10000) {
            stackMonitor.update();
            if (stackMonitor.hasCriticalStackUsage()) {
                addLog("🚨 STACK WARNING: " + stackMonitor.getCriticalTasks());
            }
            lastStackCheck = millis();
        }
        
        // Optimize sensor updates - разделяем по времени
        updateSensorsOptimized();
        
        // Process LoRa packets - вынесено в отдельную функцию
        processLoRaPackets();
        
        // Control and navigation updates
        updateControlAndNavigation();

        static unsigned long lastOilPumpUpdate = 0;
        static unsigned long VuPDATE = 0;

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

        LoRaPacket pkt;
        if (LoRaCore::receive(pkt) && pkt.senderId != BOAT_DEVICE_ID)
        {
            uint8_t senderId = pkt.senderId;
            PacketBase hdr;
            hdr.packetType = pkt.packetType;
            hdr.packetId = pkt.packetId;
            hdr.payloadLen = pkt.payloadLen;
            const uint8_t *buf = pkt.payload;
            float snr = loraComm->getRadio().getSNR();
            float rssi = loraComm->getRadio().getRSSI();
            // Безопасное логирование с объединенным форматом
            // char logBuffer[256];
            // snprintf(logBuffer, sizeof(logBuffer), 
            //     "[P:%d][RSSI:%.2f/%.2fdBm,SNR=%.1fdB] T=%c from=%d id=%d P: %s",
            //     currentProfileIndex, rssi, smoothedRssi, snr, 
            //     (char)hdr.packetType, senderId, hdr.packetId, 
            //     hdr.toString().c_str());
            // addLog(String(logBuffer));

            // Диспетчер внутри Boat
            switch (hdr.packetType)
            {

            case CMD_COMMAND_STRING:
            {
                PongRssi = updateSmoothedRssi(rssi);
                missionCOntrolIsActivae = true;
                addLog("Got COMMND");
                PacketCommand cmd{};
                cmd.packetType = hdr.packetType;
                cmd.packetId = hdr.packetId;
                cmd.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase),
                       buf,
                       hdr.payloadLen);
                // выполнить строку команды
                String s;
                for (size_t i = 0; i < cmd.payloadLen; ++i)
                {
                    s += char(buf[i]);
                }
                parser.processLine(s);
                break;
            }

            case CMD_TELEMETRY_FRAGMENT:
            {
                addLog("Got CMD_TELEMETRY_FRAGMENT");
                PacketTelemetry tel{};
                tel.packetType = hdr.packetType;
                tel.packetId = hdr.packetId;
                tel.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&tel) + sizeof(PacketBase),
                       buf,
                       hdr.payloadLen);
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
                       hdr.payloadLen);
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
                       hdr.payloadLen);
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
                       hdr.payloadLen);
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
                       hdr.payloadLen);
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
                       hdr.payloadLen);
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
                       hdr.payloadLen);
                onHeartbeat(hb);
                break;
            }
            case CMD_PONG:
            {
                // PONG - важная команда, оставляем лог для диагностики связи
                addLog("Got CMD_PONG");
                PongRssi = updateSmoothedRssi(rssi);
                missionCOntrolIsActivae = true;
                break;
            }
            case CMD_REPOSNCE_ASA:
            {
                addLog("Got CMD_REPOSNCE_ASA");
                waitingForASAAck = false;
                asaActive = true;
                asaLastSwitchTime = millis();

                if (hdr.payloadLen == sizeof(uint8_t))
                {
                    uint8_t profileIndex = buf[0];
            
                    addLog("✅ ASA response received. Applying higher LoRa profile old/new index: " + String(currentProfileIndex) + "/" + String(profileIndex));
                    currentProfileIndex = profileIndex;
                    applyProfile(profileIndex);
                }
                else
                {
                    addLog("⚠️ Invalid ASA payload size");
                }

                break;
                // addLog("Received ASA response from MC");
                // loraAdapt->onPongReceived();
                // break;
            }
            case CMD_GET_BOAT_STATUS:
            {
                addLog("Got CMD_GET_BOAT_STATUS");
                sendStatusJsonFragmentsTask();
                break;
            }
            case CMD_PING:
            {
                PongRssi = updateSmoothedRssi(rssi);
                missionCOntrolIsActivae = true;
                addLog("Got CMD_PING: smoothedRssi" + String(smoothedRssi) + "dBm");
                break;
            }
            case CMD_REQUEST_INFO:
            {
                PongRssi = updateSmoothedRssi(rssi);
                addLog("SPECIAL Got CMD_REQUEST_INFO");
                // Универсальный “P” запрос — в payloadBuf[0] лежит нужный код ('T','I','S','F','G','D'…)
                CommandType what = static_cast<CommandType>(buf[0]);
                switch (what)
                {

                //  ——————————————————————————————————————————————————————————
                // Телеметрия / статус лодки (JSON-фрагменты)
                //  ——————————————————————————————————————————————————————————
                case CMD_BOAT_STATUS_REPORT:
                { // 'T' или 'D'
                    sendStatusJsonFragmentsTask();
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Информация о двигателе
                //  ——————————————————————————————————————————————————————————
                case CMD_INFO_ENGINE:
                { // 'I'
                    PacketInfoEngine info{};
                    info.packetType = CMD_INFO_ENGINE;
                    info.packetId = nextPacketId++;
                    info.payloadLen = 0;
                    LoRaCore::sendPacketBase(senderId, info, nullptr);
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Конфигурация
                //  ——————————————————————————————————————————————————————————
                case CMD_CONFIG:
                { // 'F'
                    PacketConfig cfg{};
                    cfg.packetType = CMD_CONFIG;
                    cfg.packetId = nextPacketId++;
                    cfg.payloadLen = 0; // или >0, если ты что-то пишешь в payload

                    LoRaCore::sendPacketBase(senderId, cfg, nullptr);
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Навигация
                //  ——————————————————————————————————————————————————————————
                case CMD_NAV:
                { // 'G'
                    PacketNav nav{};
                    nav.packetType = CMD_NAV;
                    nav.packetId = nextPacketId++;
                    // заполняем nav… (координаты, курс, скорость)
                    LoRaCore::sendPacketBase(senderId, nav, nullptr);
                    break;
                }

                default:
                    addLog("Boat: Unsupported info request '" + String((char)what) + "'");
                    break;
                }
                break;
            }

            default:
                addLog("Got Unknown LoRa cmd: " + String((char)hdr.packetType));
                break;
            }

            if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG)
            {
                PacketAck ackOut{};
                ackOut.packetType = CMD_ACK;      // 0x00 – нормально
                ackOut.packetId = nextPacketId++; // новый собственный ID
                ackOut.ackedId = hdr.packetId;    // чей подтверждаем

                ackOut.payloadLen = sizeof(ackOut.ackedId); // 2 байта

                uint8_t ackBuf[sizeof(ackOut.ackedId)];
                memcpy(ackBuf, &ackOut.ackedId, sizeof(uint16_t));

                LoRaCore::sendPacketBase(senderId, ackOut, ackBuf, false);
            }
            // В конце — отправляем ACK обратно отправителю

            lastPacketReceived = millis();
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

        if (millis() - lastPingSent >= PING_INTERVAL && !waitingForASAAck)
        {
            PacketBase ping{};
            ping.packetType = CMD_PING;
            ping.packetId = nextPacketId++;
            ping.payloadLen = 0;
            LoRaCore::sendPacketBase(MISSION_CONTROL_ID, ping, nullptr, false);
            addLog("[PROF:" + String(currentProfileIndex) + "] 🔄 Ping → MC");
            lastPingSent = millis();
        }

        if (waitingForASAAck && millis() - asaProposalTime > ASA_TIMEOUT)
        {
            addLog("❌ ASA: Нет ACK от управления. Отклоняем переход.");
            waitingForASAAck = false;
        }
        if (asaActive && millis() - lastPacketReceived > ACTIVITY_TIMEOUT)
        {
            addLog("⏳ ASA: Таймаут активности. Возврат к дефолтным LoRa-настройкам.");
            restoreDefaultLoRaSettings();
            asaActive = false;
            PongRssi = 0;
            missionCOntrolIsActivae = false;
        }
        if (missionCOntrolIsActivae)
        {
            adaptiveLoraUpdate();
        }
        
        // Синхронизация при первом валидном значении GPS времени
        static unsigned long lastSync = 0;
        if (gnss.isTimeUpdated() && millis() - lastSync > 160000)
        {
            gnss.syncSystemTimeFromGPS();
            lastSync = millis();
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
        
        // Performance tracking
        unsigned long keepDuration = millis() - keepStartTime;
        performanceMetrics.recordKeepCycle(keepDuration);
        
        // Warn if keep() cycle is taking too long
        if (keepDuration > 50) { // 50ms warning threshold
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

        LoRaCore::sendPacketBase(receiverId, pkt, reinterpret_cast<const uint8_t *>(&pkt) + sizeof(PacketBase));
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

    void adaptiveLoraUpdate()
    {
        if (waitingForASAAck || millis() - lastAdaptiveSwitchTime < ADAPTIVE_SWITCH_INTERVAL)
            return;

        lastAdaptiveSwitchTime = millis();

        // Use cached values if available, otherwise get fresh data
        float rssi, snr;
        if (sensorCache.isValid()) {
            rssi = sensorCache.lastRSSI;
            snr = sensorCache.lastSNR;
        } else {
            rssi = loraComm->getRadio().getRSSI();
            snr = loraComm->getRadio().getSNR();
            sensorCache.update(rssi, snr);
        }

        int bestIndex = currentProfileIndex;
        if (PongRssi >= 0)
        {
            PongRssi = rssi;
        }
        
        // Enhanced profile selection with SNR consideration
        bestIndex = selectOptimalProfile(PongRssi, snr);

        if (bestIndex != currentProfileIndex)
        {
            String direction = bestIndex > currentProfileIndex ? "upgrade" : "downgrade";
            addLog("[adaptiveLoraUpdate] rssi:" + String(PongRssi) + " snr:" + String(snr) +
                   " → Proposing " + direction + " to profile " + String(bestIndex));

            sendAsaRequest(nextPacketId++, bestIndex, MISSION_CONTROL_ID);
            waitingForASAAck = true;
            
            // For downgrades, apply immediately for safety
            if (direction == "downgrade")
            {
                applyProfile(bestIndex);
                addLog("⬇️ ASA: Emergency downgrade to profile " + String(bestIndex));
            }
            asaProposalTime = millis();
        }
    }
    
    // Enhanced profile selection with SNR consideration
    int selectOptimalProfile(float rssi, float snr) {
        // If SNR is very poor, force lower profile regardless of RSSI
        if (snr < -5.0f) {
            return min(2, (int)currentProfileIndex); // Force to profile 2 or lower
        }
        
        // If both RSSI and SNR are good, we can use higher profile
        if (rssi > -90.0f && snr > 8.0f) {
            return min(8, rssiToIndex(rssi) + 1); // One step higher than RSSI suggests
        }
        
        return rssiToIndex(rssi);
    }

    void applyProfile(uint8_t idx)
    {
        auto &p = loraProfiles[idx];
        loraComm->applySettings(p.spreadingFactor, p.codingRate, p.bandwidth);
    }

    void restoreDefaultLoRaSettings()
    {
        addLog("🔁 Восстановление стандартных LoRa параметров...");
        currentProfileIndex = 0;           // Сбрасываем индекс профиля
        applyProfile(currentProfileIndex); // Применяем профиль 0 (стандартный)
        asaActive = false;                 // Деактивируем ASA
        waitingForASAAck = false;          // Сбрасываем ожидание ACK
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
            LoRaCore::sendPacketBase(MISSION_CONTROL_ID, cmd, tempBuf);
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

        JsonObject motorObj = doc[F("motor")].to<JsonObject>();
        engine.toJSON(motorObj);

        JsonObject batteryObj = doc[F("b")].to<JsonObject>();
        battery.toJson(batteryObj);

        JsonObject receiverObj = doc["receiver"].to<JsonObject>();
        flysky.toJson(receiverObj);
        JsonArray inaArray = doc["ina3221"].to<JsonArray>();
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            JsonObject chObj = inaArray.add<JsonObject>();
            chObj["channel"] = ch + 1;
            chObj["bus"] = ina3221.getBusVoltage(ch);
            chObj["shunt"] = ina3221.getShuntVoltage(ch);
            chObj["current"] = ina3221.getCurrentAmps(ch);
        }

        JsonArray inaLowArray = doc["ina3221_low"].to<JsonArray>();
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            JsonObject chObj = inaLowArray.add<JsonObject>();
            chObj["channel"] = ch + 1;
            chObj["bus"] = ina3221_low.getBusVoltage(ch);
            chObj["shunt"] = ina3221_low.getShuntVoltage(ch);
            chObj["current"] = ina3221_low.getCurrentAmps(ch);
        }

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
        if (stackMonitor.hasCriticalStackUsage()) {
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
        loraObj[F("profile_index")] = currentProfileIndex;
        loraObj[F("smoothed_rssi")] = smoothedRssi;
        if (sensorCache.isValid()) {
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
        if (gnss.hasValidFix()) {
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
                if (!logBuffer.empty()) {
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
        
        if (response.length() <= maxPayloadSize) {
            // Короткое сообщение - отправляем как есть
            sendResponseFragment(response, 0, 1);
        } else {
            // Длинное сообщение - фрагментируем
            size_t totalLen = response.length();
            size_t chunks = (totalLen + maxPayloadSize - 1) / maxPayloadSize;
            
            // Безопасное логирование фрагментации
            char logBuffer[128];
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Response too long (%zu bytes), fragmenting into %zu parts", 
                totalLen, chunks);
            addLog(String(logBuffer));
            
            for (size_t i = 0; i < chunks; i++) {
                String fragment = response.substring(i * maxPayloadSize, (i + 1) * maxPayloadSize);
                
                // Безопасное формирование заголовка
                char headerBuffer[32];
                snprintf(headerBuffer, sizeof(headerBuffer), "[%zu/%zu]", i, chunks);
                String payload = String(headerBuffer) + fragment;
                
                sendResponseFragment(payload, i, chunks);
                vTaskDelay(pdMS_TO_TICKS(90));
            }
        }
    }
    
    void sendResponseFragment(const String &payload, size_t fragmentIndex, size_t totalFragments)
    {
        if (payload.length() > MAX_LORA_PAYLOAD) {
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
        LoRaCore::sendPacketBase(MISSION_CONTROL_ID, responsePacket, tempBuf);
        
        // Безопасное логирование результата
        char logBuffer[128];
        if (totalFragments > 1) {
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Fragment %zu/%zu sent (%zu bytes)", 
                fragmentIndex + 1, totalFragments, len);
        } else {
            snprintf(logBuffer, sizeof(logBuffer), "[RESP] Response sent (%zu bytes)", len);
        }
        addLog(String(logBuffer));
    }

private:
    float smoothedRssi = -120.0f;  // начальное значение
    const float rssiAlpha = 0.15f; // коэффициент сглаживания (0.1–0.3)
    
    float updateSmoothedRssi(float newRssi)
    {

        smoothedRssi = rssiAlpha * newRssi + (1.0f - rssiAlpha) * smoothedRssi;
        return smoothedRssi;
    }

    String inputBuffer;
    uint16_t nextPacketId = 0;
    bool lastTransmitterState = true; // было ли соединение ранее
    bool failsafeTriggered = false;   // уже обработали событие потери
    static constexpr size_t logCapacity = 90;
    std::vector<String> logBuffer;
    SemaphoreHandle_t logMutex = nullptr;  // Мьютекс для потокобезопасного логирования
    unsigned long lastPingSent = 0;
    uint8_t currentProfileIndex = 0;
    unsigned long lastAdaptiveSwitchTime = 0;
    float PongRssi = 0; // RSSI при получении PONG
    bool missionCOntrolIsActivae = false;

    SensorCache sensorCache;
    PerformanceMetrics performanceMetrics;
    StackMonitor stackMonitor;

    // Safety and monitoring improvements
    struct SafetyMonitor {
        bool overtemperatureShutdown = false;
        bool lowVoltageShutdown = false;
        bool communicationLoss = false;
        bool systemHealthy = true;
        unsigned long lastHealthCheck = 0;
        
        void checkSystemHealth(TempSensorManager& temps, const BatteryMonitor& battery, unsigned long lastComms) {
            if (millis() - lastHealthCheck < 1000) return; // Check every second
            
            // Temperature monitoring
            float motor1Temp = temps.get(MOTOR1);
            float motor2Temp = temps.get(MOTOR2);
            
            overtemperatureShutdown = (motor1Temp > 85.0f || motor2Temp > 85.0f);
            
            // Battery monitoring
            float voltage = battery.getVoltage();
            lowVoltageShutdown = (voltage < 11.0f && voltage > 5.0f); // Ignore invalid readings
            
            // Communication monitoring
            communicationLoss = (millis() - lastComms > 60000); // 1 minute timeout
            
            systemHealthy = !overtemperatureShutdown && !lowVoltageShutdown && !communicationLoss;
            lastHealthCheck = millis();
        }
        
        String getStatusString() const {
            String status = "System: ";
            status += systemHealthy ? "HEALTHY" : "WARNING";
            if (overtemperatureShutdown) status += " [OVERTEMP]";
            if (lowVoltageShutdown) status += " [LOW_VOLT]";
            if (communicationLoss) status += " [COMM_LOSS]";
            return status;
        }
    };

    SafetyMonitor safetyMonitor;
    
    // Optimized sensor update function
    void updateSensorsOptimized()
    {
        static unsigned long lastSensorUpdate = 0;
        static unsigned long lastGnssUpdate = 0;
        static unsigned long lastCacheUpdate = 0;
        
        unsigned long now = millis();
        
        // Battery and basic sensors - каждые 50ms (20Hz)
        if (now - lastSensorUpdate >= 50) {
            battery.prepareForRead();
            battery.readVoltage();
            lastSensorUpdate = now;
        }
        
        // GNSS - каждые 200ms (5Hz) для экономии ресурсов
        if (now - lastGnssUpdate >= 200) {
            gnss.update();
            lastGnssUpdate = now;
        }
        
        // LoRa cache - каждые 100ms (10Hz)
        if (now - lastCacheUpdate >= 100 && !sensorCache.isValid() && loraComm) {
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
        if (millis() - lastPacketCheck >= 10) { // 100Hz для LoRa
            // Use LoRaCore API instead of direct LoRaComm calls
            LoRaPacket pkt;
            if (LoRaCore::receive(pkt) && pkt.senderId != BOAT_DEVICE_ID) {
                lastPacketReceived = millis();
                // Process packet with optimized stack usage
                handleLoRaPacketOptimized(pkt);
            }
            lastPacketCheck = millis();
        }
    }
    
    // Lightweight packet handler with reduced stack usage
    void handleLoRaPacketOptimized(const LoRaPacket& pkt)
    {
        // Use minimal local variables to save stack
        uint8_t senderId = pkt.senderId;
        PacketBase hdr;
        hdr.packetType = pkt.packetType;
        hdr.packetId = pkt.packetId;
        hdr.payloadLen = pkt.payloadLen;
        const uint8_t *buf = pkt.payload;
        
        // Quick check for critical commands (emergency stop, etc.)
        if (hdr.packetType == CMD_COMMAND_STRING) {
            // Handle critical commands immediately with minimal stack usage
            processCommandPacket(hdr, buf, senderId);
        } else if (hdr.packetType == CMD_PING || hdr.packetType == CMD_PONG) {
            // Handle ping/pong immediately (lightweight)
            processPingPongPacket(hdr, senderId);
        } else {
            // Defer other packets to main processing in keep() loop
            // This reduces stack usage by not processing heavy operations here
            static unsigned long lastDeferredProcess = 0;
            if (millis() - lastDeferredProcess >= 50) { // Process deferred packets every 50ms
                processGeneralPacket(hdr, buf, senderId);
                lastDeferredProcess = millis();
            }
        }
    }
    
    // Control and navigation updates with reduced frequency
    void updateControlAndNavigation()
    {
        static unsigned long lastControlUpdate = 0;
        static unsigned long lastNavUpdate = 0;
        
        unsigned long now = millis();
        
        // Basic control - каждые 20ms (50Hz)
        if (now - lastControlUpdate >= 20) {
            // Minimal scope для экономии стека
            {
                // Basic engine state updates only
                // Heavy operations moved to main loop
            }
            lastControlUpdate = now;
        }
        
        // Navigation - каждые 100ms (10Hz) для сложных вычислений
        if (now - lastNavUpdate >= 100) {
            updateNavigationSystem();
            lastNavUpdate = now;
        }
    }
    
    // Navigation system update with stack optimization
    void updateNavigationSystem()
    {
        // Only update if auto navigation is active and we have valid GNSS
        if (autoNav.getMode() == AutoNavigation::MANUAL || !gnss.hasValidFix()) {
            return;
        }
        
        // Use local variables sparingly to save stack
        float lat = gnss.getLatitude();
        float lon = gnss.getLongitude();
        float heading = gnss.getHeading();
        float speed = gnss.getSpeed();
        
        auto navOutput = autoNav.update(lat, lon, heading, speed);
        
        if (navOutput.navigationActive && !failsafeTriggered) {
            // Apply navigation commands
            if (navOutput.rudderAngle != 0) {
                rudder.setAngle((int)navOutput.rudderAngle);
            }
            
            // Log navigation status periodically
            static unsigned long lastNavLog = 0;
            if (millis() - lastNavLog > 5000) {
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
        if (now > 1600000000) // Check if time is valid (after 2020)
        {
            struct tm *t = localtime(&now);
            char buf[9];
            strftime(buf, sizeof buf, "%H:%M:%S", t);
            return String(buf);
        }
        
        // If system time is not synced, return default
        return F("00:00:00");
    }

    // Helper functions for packet processing with minimal stack usage
    void processCommandPacket(const PacketBase& hdr, const uint8_t* buf, uint8_t senderId)
    {
        PacketCommand cmd{};
        cmd.packetType = hdr.packetType;
        cmd.packetId = hdr.packetId;
        cmd.payloadLen = hdr.payloadLen;
        memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase), buf, hdr.payloadLen);
        
        // Convert to string with minimal stack usage
        String s;
        s.reserve(hdr.payloadLen + 1);
        for (size_t i = 0; i < cmd.payloadLen; ++i) {
            s += char(buf[i]);
        }
        parser.processLine(s);
        
        // Update RSSI and mission control status
        if (loraComm) {
            PongRssi = updateSmoothedRssi(loraComm->getRadio().getRSSI());
            missionCOntrolIsActivae = true;
        }
        addLog("Got COMMAND");
    }
    
    void processPingPongPacket(const PacketBase& hdr, uint8_t senderId)
    {
        if (loraComm) {
            PongRssi = updateSmoothedRssi(loraComm->getRadio().getRSSI());
            missionCOntrolIsActivae = true;
        }
        
        if (hdr.packetType == CMD_PING) {
            addLog("Got CMD_PING: smoothedRssi " + String(smoothedRssi) + "dBm");
        } else {
            addLog("Got CMD_PONG");
        }
    }
    
    void processGeneralPacket(const PacketBase& hdr, const uint8_t* buf, uint8_t senderId)
    {
        // This function handles non-critical packets with full processing
        // but is called less frequently to reduce stack pressure
        
        switch (hdr.packetType) {
            case CMD_TELEMETRY_FRAGMENT:
                addLog("Got CMD_TELEMETRY_FRAGMENT");
                break;
            case CMD_INFO_ENGINE:
                addLog("Got CMD_INFO_ENGINE");
                break;
            case CMD_STATUS:
                addLog("Got CMD_STATUS");
                break;
            case CMD_ACK:
                addLog("Got CMD_ACK");
                break;
            case CMD_CONFIG:
                addLog("Got CMD_CONFIG");
                break;
            case CMD_NAV:
                addLog("Got CMD_NAV");
                break;
            case CMD_HEARTBEAT:
                addLog("Got CMD_HEARTBEAT");
                break;
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
            default:
                addLog("Got Unknown LoRa cmd: " + String((char)hdr.packetType));
                break;
        }
        
        // Send ACK for non-ACK packets
        if (hdr.packetType != CMD_ACK && hdr.packetType != CMD_PING && hdr.packetType != CMD_PONG) {
            sendAckPacket(hdr.packetId, senderId);
        }
    }
    
    void handleASAResponse(const PacketBase& hdr, const uint8_t* buf)
    {
        addLog("Got CMD_REPOSNCE_ASA");
        waitingForASAAck = false;
        asaActive = true;
        asaLastSwitchTime = millis();

        if (hdr.payloadLen == sizeof(uint8_t)) {
            uint8_t profileIndex = buf[0];
            addLog("✅ ASA response received. Applying higher LoRa profile old/new index: " + 
                   String(currentProfileIndex) + "/" + String(profileIndex));
            currentProfileIndex = profileIndex;
            applyProfile(profileIndex);
        } else {
            addLog("⚠️ Invalid ASA payload size");
        }
    }
    
    void handleInfoRequest(const PacketBase& hdr, const uint8_t* buf, uint8_t senderId)
    {
        if (loraComm) {
            PongRssi = updateSmoothedRssi(loraComm->getRadio().getRSSI());
        }
        addLog("SPECIAL Got CMD_REQUEST_INFO");
        
        CommandType what = static_cast<CommandType>(buf[0]);
        switch (what) {
            case CMD_BOAT_STATUS_REPORT:
                sendStatusJsonFragmentsTask();
                break;
            case CMD_INFO_ENGINE: {
                PacketInfoEngine info{};
                info.packetType = CMD_INFO_ENGINE;
                info.packetId = nextPacketId++;
                info.payloadLen = 0;
                LoRaCore::sendPacketBase(senderId, info, nullptr);
                break;
            }
            case CMD_CONFIG: {
                PacketConfig cfg{};
                cfg.packetType = CMD_CONFIG;
                cfg.packetId = nextPacketId++;
                cfg.payloadLen = 0;
                LoRaCore::sendPacketBase(senderId, cfg, nullptr);
                break;
            }
            case CMD_NAV: {
                PacketNav nav{};
                nav.packetType = CMD_NAV;
                nav.packetId = nextPacketId++;
                LoRaCore::sendPacketBase(senderId, nav, nullptr);
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

        LoRaCore::sendPacketBase(receiverId, ackOut, ackBuf, false);
    }
};
