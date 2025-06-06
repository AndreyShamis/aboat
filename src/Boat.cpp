#include "Boat.hpp"

void Boat::printStatus() const {
    Serial.println("=== Boat Status ===");
    Serial.printf("Motor1:    %.2f °C\n", motor1Temp);
    Serial.printf("Motor2:    %.2f °C\n", motor2Temp);
    Serial.printf("Radiator:  %.2f °C\n", radiatorTemp);
    Serial.printf("Oil:       %.2f °C\n", oilTemp);
    Serial.printf("Ambient:   %.2f °C\n", envTemp);
    Serial.println("====================");
}
