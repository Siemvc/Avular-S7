#include "TiltingActuator.h"

// 1. De Constructor (Instellingen opslaan)
TiltingActuator::TiltingActuator(int pinUp, int pinDown, int pinPot) {
    _pinUp = pinUp;
    _pinDown = pinDown;
    _pinPot = pinPot;

    // Standaard waarden (kun je hier aanpassen of via setters doen)
    _minPWM = 60;
    _maxPWM = 245;
    _deadband = 4;
    _kp = 5;
    
    // Potmeter kalibratie
    _minPot = 30;
    _maxPot = 785;

    _targetPos = -1; // -1 betekent: nog geen doel
}

// 2. Setup (pinModes)
void TiltingActuator::begin() {
    pinMode(_pinUp, OUTPUT);
    pinMode(_pinDown, OUTPUT);
    pinMode(_pinPot, INPUT);
    
    // Lees startpositie zodat hij niet wegspringt
    _currentPos = analogRead(_pinPot);
    _targetPos = _currentPos;
}

// 3. Interne motor aansturing
void TiltingActuator::setMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(_pinUp, speed);
        analogWrite(_pinDown, 0);
    } else if (speed < 0) {
        analogWrite(_pinDown, -speed); // Positief maken
        analogWrite(_pinUp, 0);
    } else {
        analogWrite(_pinDown, 0);
        analogWrite(_pinUp, 0);
    }
}

// 4. De Update Loop (P-Controller)
void TiltingActuator::update() {
    _currentPos = analogRead(_pinPot);

    // Veiligheid: Als target nog niet gezet is, doe niks
    if (_targetPos == -1) return;

    int error = _targetPos - _currentPos;

    if (abs(error) > _deadband) {
        int speed = error * _kp;

        // Limiet erop
        if (speed > 0) speed = constrain(speed, _minPWM, _maxPWM);
        else speed = constrain(speed, -_maxPWM, -_minPWM);

        setMotorSpeed(speed);
    } else {
        setMotorSpeed(0);
    }
}

// 5. Setter voor het doel (Accepteert 0 - 100%)
void TiltingActuator::setTargetPosition(int percentage) {
    // Veiligheid: begrens input
    percentage = constrain(percentage, 0, 100);
    
    // Map percentage naar Potmeter waarden
    _targetPos = map(percentage, 0, 100, _minPot, _maxPot);
    
    // Serial print kan hier ook, of in main
    // Serial.printf("Nieuw doel ingesteld: %d%%\n", percentage);
}

// 6. Getter voor huidige positie
int TiltingActuator::getCurrentPosition() {
    return _currentPos;
}