#include "LiftingActuators.h"

LiftingActuators::LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB) {
    _pinIN1A = pinIN1A; _pinIN2A = pinIN2A; _pinENA = pinENA;
    _pinIN1B = pinIN1B; _pinIN2B = pinIN2B; _pinENB = pinENB;
    _pinPotA = pinPotA; _pinPotB = pinPotB;

    _minPWM = 70; _maxPWM = 255;
    _deadband = 10; _kp = 6.0;
    _minPot = 30; _maxPot = 785;
    _targetPos = -1;
    _manualMode = true;
}

void LiftingActuators::begin() {
    pinMode(_pinIN1A, OUTPUT); pinMode(_pinIN2A, OUTPUT); pinMode(_pinENA, OUTPUT);
    pinMode(_pinIN1B, OUTPUT); pinMode(_pinIN2B, OUTPUT); pinMode(_pinENB, OUTPUT);
    pinMode(_pinPotA, INPUT);  pinMode(_pinPotB, INPUT);
    
    digitalWrite(_pinENA, HIGH); digitalWrite(_pinENB, HIGH);
    _currentPos = analogRead(_pinPotA);
}

void LiftingActuators::setMotorSpeed(int speed) {
    // --- SOFT LIMIT CHECK ---
    if (_currentPos < _minPot && speed < 0) speed = 0;
    if (_currentPos > _maxPot && speed > 0) speed = 0;

    _speed = speed;

    if (speed == 0) {
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, 0);
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, 0);
        return;
    }

    if (speed > 0) { // OMHOOG
        analogWrite(_pinIN1A, speed); analogWrite(_pinIN2A, 0);
        analogWrite(_pinIN1B, speed); analogWrite(_pinIN2B, 0);
    } else { // OMLAAG
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, -speed);
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, -speed);
    }
}

void LiftingActuators::update() {
    _currentPos = analogRead(_pinPotA);
    
    // In Manual Mode doet deze update loop niks (behalve potmeter lezen)
    if (_manualMode) return;

    // Auto Mode (PID)
    if (_targetPos == -1) return;

    int error = _targetPos - _currentPos;
    if (abs(error) > _deadband) {
        int calcSpeed = error * _kp;
        if (calcSpeed > 0) calcSpeed = constrain(calcSpeed, _minPWM, _maxPWM);
        else calcSpeed = constrain(calcSpeed, -_maxPWM, -_minPWM);
        setMotorSpeed(calcSpeed);
    } else {
        setMotorSpeed(0);
    }
}

void LiftingActuators::setTargetPosition(int percentage) {
    _manualMode = false; // PID AAN
    percentage = constrain(percentage, 0, 100);
    _targetPos = map(percentage, 0, 100, _minPot, _maxPot);
}

void LiftingActuators::setManualSpeed(float input) {
    _manualMode = true; // PID UIT, Direct PWM
    int pwm = (int)(input * 255.0);
    if (abs(pwm) < 40) pwm = 0; 
    setMotorSpeed(pwm);
}

int LiftingActuators::getCurrentPosition() { return map(_currentPos, _minPot, _maxPot, 0, 100); }
int LiftingActuators::getTargetPositionRaw() { return _targetPos; }
int LiftingActuators::getLastSpeed() { return _speed; }