// BoatCommandHandler.hpp
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <time.h>

// Forward declarations
class Boat;
class TempSensorManager;
class AutoNavigation;
class GNSSManager;
class IMUModule;
class LoRaCore;
class StackMonitor;

/**
 * @brief Command handler class for boat commands
 * Separates command processing logic from the main Boat class
 */
class BoatCommandHandler
{
private:
    Boat* boat;  // Reference to the boat instance

public:
    explicit BoatCommandHandler(Boat* boatInstance) : boat(boatInstance) {}

    // Main command handlers
    String handleLoRaProfileCommand(const String& arg);
    String handleDiagnosticCommand(const String& arg);
    String handleNavigationCommand(const String& arg);
    String handleWaypointCommand(const String& arg);
    String handleTimeCommand(const String& arg);
    String handleDataModeCommand(const String& arg);
    String handleIMUCommand(const String& arg);
    String handleStructuredDataCommand(const String& command);

    // LoRa specific helpers
    String applyLoRaProfile(int profileIndex);
    String handleAdaptiveControl(const String& adaptiveStr);

    // Navigation specific helpers
    String handleSetHome();
    String handleNavigationStart(const String& coords);
    String handleNavigationMode(const String& mode);

    // Diagnostic helpers
    String buildFullDiagnostic();
    String buildExtendedDiagnostic();

private:
    // Helper methods that need access to boat internals
    void addLog(const String& message);
    String timeStr();
    void sendStructuredHeartbeat();
    void sendStructuredFullStatus();
    void sendStructuredGPS();
    void sendStructuredMotors();
    void sendStructuredSensors();
    void sendStructuredLoRaStatus();
    void sendStructuredNavigation();
    void sendStructuredSystemInfo();
    void adaptiveLoraUpdate();
    void applyProfile(uint8_t idx);
};
