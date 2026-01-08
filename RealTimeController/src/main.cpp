#include <Arduino.h>
#include "TiltingActuator.h"
#include "LiftingActuators.h"

// Maak het object aan (Global scope)
// Pinnen: UP=28, DOWN=29, POT=27
TiltingActuator tilting(28, 29, 27); //pinIN1, pinIN2, PinPot
LiftingActuators lifting(8, 7, 6, 5, 4, 3, 26, 25); //pinIN1A, pinIN1B, pinIN2A, pinIN2B, EnableA, EnableB, PinPotA, PinPotB

void setup() {
    Serial.begin(115200);
    
    // Resolutie instellen (belangrijk voor potmeter waarden in je class)
    analogReadResolution(10); 

    // Start de actuator
    tilting.begin();
    lifting.begin();

    Serial.println("Avuloader Tilting System Ready.");
    Serial.println("Typ een waarde 0-100 om te bewegen.");
}

void loop() {
    // 1. Check for Serial Input
    if (Serial.available() > 0) {
        int input = Serial.parseInt();
         if(input >= 0 && input <= 100) {
            tilting.setTargetPosition(input);
            lifting.setTargetPosition(input);

            Serial.printf("Command received: %d\n", input);
        } 
        else {
            Serial.println("Invalid input. Please enter a value between 0 and 100.");
            delay(500);
        }
    }

    //Update actuator state (read potentiometer and adjust motor)
    tilting.update();
    lifting.update();


    delay(10);
}