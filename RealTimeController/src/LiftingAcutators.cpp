#include "LiftingActuators.h"

// 1. De Constructor (Instellingen opslaan)
LiftingActuators::LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB) {
    _pinIN1A = pinIN1A;
    _pinIN1B = pinIN1B;
    _pinIN2A = pinIN2A;
    _pinIN2B = pinIN2B;
    _pinENA = pinENA;
    _pinENB = pinENB;
    _pinPotA = pinPotA;
    _pinPotB = pinPotB;

    // Standaard waarden (kun je hier aanpassen of via setters doen)
    _minPWM = 60;
    _maxPWM = 245;
    _deadband = 5;
    _kp = 7;
    _speed = 0;
    
    // Potmeter kalibratie
    _minPot = 30;
    _maxPot = 785;

    _targetPos = -1; // -1 betekent: nog geen doel
}

// 2. Setup (pinModes)
void LiftingActuators::begin() {
    pinMode(_pinIN1A, OUTPUT);
    pinMode(_pinIN1B, OUTPUT);
    pinMode(_pinIN2A, OUTPUT);
    pinMode(_pinIN2B, OUTPUT);
    pinMode(_pinENA, OUTPUT);
    pinMode(_pinENB, OUTPUT);
    pinMode(_pinPotA, INPUT);
    pinMode(_pinPotB, INPUT);

    // Lees startpositie zodat hij niet wegspringt
    _currentPos = analogRead(_pinPotA); // Assuming PotA for current position
    _targetPos = _currentPos;
}

// 3. Interne motor aansturing
void LiftingActuators::setMotorSpeed(int speed) {
    if (speed > 0) {
        analogWrite(_pinIN1A, speed);
        analogWrite(_pinIN1B, 0);
    } else if (speed < 0) {
        analogWrite(_pinIN1B, -speed); // Positief maken
        analogWrite(_pinIN1A, 0);
    } else {
        analogWrite(_pinIN2A, 0);
        analogWrite(_pinIN2B, 0);
    }
}

// 4. De Update Loop (P-Controller)
void LiftingActuators::update() {
    _currentPos = analogRead(_pinPotA);
    // Veiligheid: Als target nog niet gezet is, doe niks
    if (_targetPos == -1) return;

    int error = _targetPos - _currentPos;

    if (abs(error) > _deadband) {
        _speed = error * _kp;
                
        // Limit speed to min/max PWM
        if (_speed > 0) _speed = constrain(_speed, _minPWM, _maxPWM);
        else _speed = constrain(_speed, -_maxPWM, -_minPWM);

        setMotorSpeed(_speed);

    } else {
        setMotorSpeed(0);
        
    }
    //Debug
    Serial.printf("Pos: %d | Target: %d | Error: %d | Speed: %d\n", _currentPos, _targetPos, error, _speed);

 
}

// 5. Setter voor het doel (Accepteert 0 - 100%)
void LiftingActuators::setTargetPosition(int distance) {
    // Veiligheid: begrens input
    distance = constrain(distance, 0, 100);
    
    // Map distance naar Potmeter waarden
    _targetPos = map(distance, 0, 100, _minPot, _maxPot);
    
    // Serial print kan hier ook, of in main
    // Serial.printf("Nieuw doel ingesteld: %d%%\n", distance);
}

// 6. Getter voor huidige positie
int LiftingActuators::getCurrentPosition() {
    return _currentPos;
}