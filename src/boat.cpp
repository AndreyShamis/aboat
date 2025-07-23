// boat.cpp
#include "Boat.hpp"
#include "BoatCommandHandler.hpp"

// Implement methods that need BoatCommandHandler here
// This solves the circular dependency issue

// Constructor implementation moved here to access complete BoatCommandHandler type
void Boat::initializeCommandHandler() {
    if (!commandHandler) {
        commandHandler = new BoatCommandHandler(this);
    }
}

// Destructor helper
void Boat::cleanupCommandHandler() {
    if (commandHandler) {
        delete commandHandler;
        commandHandler = nullptr;
    }
}

void Boat::registerCommandHandlers() {
    if (!commandHandler) {
        initializeCommandHandler();
    }

    // LoRa commands with simplified registration
    registerCommandWithResponse("L", [this](const String &arg) {
        return commandHandler->handleLoRaProfileCommand(arg);
    });

    // Diagnostic commands with simplified registration
    registerCommandWithResponse("D", [this](const String &arg) {
        return commandHandler->handleDiagnosticCommand(arg);
    });

    // Navigation commands with simplified registration
    registerCommandWithResponse("N", [this](const String &arg) {
        return commandHandler->handleNavigationCommand(arg);
    });

    // Waypoint commands with simplified registration
    registerCommandWithResponse("W", [this](const String &arg) {
        return commandHandler->handleWaypointCommand(arg);
    });

    // Time commands with simplified registration
    registerCommandWithResponse("T", [this](const String &arg) {
        return commandHandler->handleTimeCommand(arg);
    });

    // Data mode commands with simplified registration
    registerCommandWithResponse("DM", [this](const String &arg) {
        return commandHandler->handleDataModeCommand(arg);
    });

    // 🧭 IMU commands with simplified registration
    registerCommandWithResponse("I", [this](const String &arg) {
        return commandHandler->handleIMUCommand(arg);
    });
}
