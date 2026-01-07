#include <Arduino.h>
#include "TiltingActuator.h"

// Maak het object aan (Global scope)
// Pinnen: UP=28, DOWN=29, POT=27
TiltingActuator tilting(28, 29, 27);
void setup() {
    Serial.begin(115200);
    
    // Resolutie instellen (belangrijk voor potmeter waarden in je class)
    analogReadResolution(10); 

    // Start de actuator
    tilting.begin();
    
    Serial.println("Avuloader Tilting System Ready.");
    Serial.println("Typ een waarde 0-100 om te bewegen.");
}

void loop() {
    // 1. Check voor Serial Input
    if (Serial.available() > 0) {
        int input = Serial.parseInt();
        
        // Filter ongeldige inputs (newlines geven vaak 0)
        if (Serial.read() == '\n') { 
             // Wees zeker dat het een echte input was
             if(input >= 0 && input <= 100) {
                 tilting.setTargetPosition(input);
                 Serial.printf("Command received: %d%%\n", input);
             }
        }
    }

    // 2. Update de control loop (Elke cycle uitvoeren!)
    tilting.update();

    // 3. Debug Prints (optioneel, niet te vaak doen)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        // Je kunt nu makkelijk publieke functies aanroepen
        // Serial.println(tilting.getCurrentPosition());
        lastPrint = millis();
    }
    
    delay(10);
}