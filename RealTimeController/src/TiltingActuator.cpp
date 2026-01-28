#include "TiltingActuator.h"

// 1. De Constructor (Instellingen opslaan)
TiltingActuator::TiltingActuator(int pinUp, int pinDown, int pinPot) {
    _pinUp = pinUp;
    _pinDown = pinDown;
    _pinPot = pinPot;

    // Standaard waarden (kun je hier aanpassen of via setters doen)
    _minPWM = 60;
    _maxPWM = 245;
    _deadband = 5;
    _kp = 7;
    _speed = 0;
    _externalError = 0;
    
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
void TiltingActuator::setTargetPosition(int percentage) {
    // Veiligheid: begrens input
    percentage = constrain(percentage, 0, 100);
    
    // Map percentage naar Potmeter waarden
    _targetPos = map(percentage, 0, 100, _minPot, _maxPot);
    
    // Serial print kan hier ook, of in main
    // Serial.printf("Nieuw doel ingesteld: %d%%\n", percentage);
}

// 6. Update met externe error (voor gesynchroniseerde beweging)
void TiltingActuator::updateWithError(int error) {
    _currentPos = analogRead(_pinPot);
    _externalError = error;

    if (abs(error) > _deadband) {
        _speed = error * _kp;
                
        // Limit speed to min/max PWM
        if (_speed > 0) _speed = constrain(_speed, _minPWM, _maxPWM);
        else _speed = constrain(_speed, -_maxPWM, -_minPWM);

        setMotorSpeed(_speed);

    } else {
        setMotorSpeed(0);
    }
    
    // Debug - show external error being used
    Serial.printf("Pos: %d | ExtError: %d | Speed: %d\n", _currentPos, error, _speed);
}

// 7. Getter voor huidige positie
int TiltingActuator::getCurrentPosition() {
    return _currentPos;
}