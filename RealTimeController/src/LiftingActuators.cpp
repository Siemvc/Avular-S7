#include "LiftingActuators.h"

LiftingActuators::LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB) {
    _pinIN1A = pinIN1A; _pinIN2A = pinIN2A; _pinENA = pinENA;
    _pinIN1B = pinIN1B; _pinIN2B = pinIN2B; _pinENB = pinENB;
    _pinPotA = pinPotA; _pinPotB = pinPotB;

    _minPWM = 60; _maxPWM = 250;
    _deadband = 10; 
    _kp = 8.0; 

    // KALIBRATIE (0mm = Ingeschoven, 300mm = Uitgeschoven)
    // Waardes dalen naarmate hij uitgaat (volgens jouw meting)
    
    // Links (A)
    _minPotA = 985; // 0mm
    _maxPotA = 385; // 300mm
    
    // Rechts (B)
    _minPotB = 946; // 0mm
    _maxPotB = 403; // 300mm

    _targetPosA = -1;
    _targetPosB = -1;
    _manualMode = true;
}

void LiftingActuators::begin() {
    pinMode(_pinIN1A, OUTPUT); pinMode(_pinIN2A, OUTPUT); pinMode(_pinENA, OUTPUT); 
    pinMode(_pinIN1B, OUTPUT); pinMode(_pinIN2B, OUTPUT); pinMode(_pinENB, OUTPUT);
    pinMode(_pinPotA, INPUT);  pinMode(_pinPotB, INPUT);
    
    digitalWrite(_pinENA, HIGH); digitalWrite(_pinENB, HIGH);
    
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
}

void LiftingActuators::setMotorASpeed(int speed) {
    // Soft limits A
    if (_currentPosA > _minPotA && speed < 0) speed = 0; 
    if (_currentPosA < _maxPotA && speed > 0) speed = 0;

    _speedA = speed;
    if (speed == 0) {
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, 0);
    } else if (speed > 0) { 
        analogWrite(_pinIN1A, speed); analogWrite(_pinIN2A, 0);
    } else { 
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, -speed);
    }
}

void LiftingActuators::setMotorBSpeed(int speed) {
    // Soft limits B
    if (_currentPosB > _minPotB && speed < 0) speed = 0;
    if (_currentPosB < _maxPotB && speed > 0) speed = 0;

    _speedB = speed;
    if (speed == 0) {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, 0);
    } else if (speed > 0) {
        analogWrite(_pinIN1B, speed); analogWrite(_pinIN2B, 0);
    } else {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, -speed);
    }
}

void LiftingActuators::update() {
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
    
    if (_manualMode) return;
    if (_targetPosA == -1) return;

    // --- P Control Loops ---
    // Verschil berekenen
    // Let op: Bij jouw potmeters is (Huidig - Doel) positief als we UIT moeten.
    // Voorbeeld: Huidig=985(0mm), Doel=385(300mm). Verschil = 600.
    // Wij willen positieve speed voor UIT. Dus error * KP klopt.
    
    int errorA = _currentPosA - _targetPosA; 
    if (abs(errorA) > _deadband) {
        int pwmA = errorA * _kp;
        if (pwmA > 0) pwmA = constrain(pwmA, _minPWM, _maxPWM);
        else pwmA = constrain(pwmA, -_maxPWM, -_minPWM);
        setMotorASpeed(pwmA);
    } else {
        setMotorASpeed(0);
    }

    int errorB = _currentPosB - _targetPosB;
    if (abs(errorB) > _deadband) {
        int pwmB = errorB * _kp;
        if (pwmB > 0) pwmB = constrain(pwmB, _minPWM, _maxPWM);
        else pwmB = constrain(pwmB, -_maxPWM, -_minPWM);
        setMotorBSpeed(pwmB);
    } else {
        setMotorBSpeed(0);
    }
}

// AANGEPAST: Input is nu millimeters (0-300)
void LiftingActuators::setTargetPosition(int mm) {
    _manualMode = false; 
    mm = constrain(mm, 0, _strokeLength); // 0 tot 300
    
    _targetPosA = map(mm, 0, _strokeLength, _minPotA, _maxPotA);
    _targetPosB = map(mm, 0, _strokeLength, _minPotB, _maxPotB);
}

void LiftingActuators::setManualSpeed(float input) {
    _manualMode = true; 
    
    // FIX: Reset de targets als we handmatig overnemen. 
    // Dit voorkomt dat de main loop in de war raakt.
    _targetPosA = -1;
    _targetPosB = -1;

    int pwm = (int)(input * 255.0);
    // Deadzone filter voor PWM output (voorkomt zoemen op lage spanning)
    if (abs(pwm) < 40) pwm = 0; 
    
    setMotorASpeed(pwm);
    setMotorBSpeed(pwm);
}

int LiftingActuators::getCurrentPosition() { 
    int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);
    return (mmA + mmB) / 2; 
}

// DEBUG IMPLEMENTATIE
int LiftingActuators::getPosA()    { return _currentPosA; }
int LiftingActuators::getPosB()    { return _currentPosB; }
int LiftingActuators::getTargetA() { return _targetPosA; }
int LiftingActuators::getTargetB() { return _targetPosB; }
int LiftingActuators::getSpeedA()  { return _speedA; }
int LiftingActuators::getSpeedB()  { return _speedB; }

// NIEUW
bool LiftingActuators::isManualMode() { return _manualMode; }

int LiftingActuators::getTargetPositionRaw() { return _targetPosA; }