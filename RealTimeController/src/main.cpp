#include <Arduino.h>
#include "TiltingActuator.h"

// Maak twee actuator objecten aan (Global scope)
// Actuator 1: UP=28, DOWN=29, POT=27
// Actuator 2: UP=30, DOWN=31, POT=26 (adjust these pins as needed)
TiltingActuator actuator1(28, 29, 27); //PinUp, PinDown, PinPot
TiltingActuator actuator2(30, 31, 26); //PinUp, PinDown, PinPot

void setup() {
    Serial.begin(115200);
    
    // Resolutie instellen (belangrijk voor potmeter waarden in je class)
    analogReadResolution(10); 

    // Start beide actuators
    actuator1.begin();
    actuator2.begin();
    
    Serial.println("Avuloader Dual Tilting System Ready.");
    Serial.println("Typ een waarde 0-100 om beide actuators te bewegen.");
}

void loop() {
    // 1. Check for Serial Input
    if (Serial.available() > 0) {
        int input = Serial.parseInt();
         if(input >= 0 && input <= 100) {
            actuator1.setTargetPosition(input);
            actuator2.setTargetPosition(input);
            Serial.printf("Command received: %d - Moving both actuators\n", input);
        } 
        else {
            Serial.println("Invalid input. Please enter a value between 0 and 100.");
            delay(500);
        }
    }

    // Get current positions
    int pos1 = actuator1.getCurrentPosition();
    int pos2 = actuator2.getCurrentPosition();
    
    // Calculate average position
    int avgPos = (pos1 + pos2) / 2;
    
    // Calculate errors for synchronized movement
    int error1 = avgPos - pos1;
    int error2 = avgPos - pos2;
    
    // Update actuator states using the calculated errors for synchronization
    actuator1.updateWithError(error1);
    actuator2.updateWithError(error2);
    
    // Print synchronization info every 100ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        Serial.printf("Avg: %d | Pos1: %d (err: %d) | Pos2: %d (err: %d)\n", 
                      avgPos, pos1, error1, pos2, error2);
        lastPrint = millis();
    }

    delay(10);
}