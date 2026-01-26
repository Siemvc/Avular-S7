#include "TiltingActuator.h"

TiltingActuator::TiltingActuator(int pinIN1, int pinIN2, int pinPot) {
    _pinIN1 = pinIN1; _pinIN2 = pinIN2; _pinPot = pinPot;
    _minPWM = 60;   _maxPWM = 250;
    _deadband = 10; _kp = 6.0;
    _minPot = 30;   _maxPot = 785; 
    _targetPos = -1;
    _manualMode = true; // Begin in manual mode (veilig)
}

void TiltingActuator::begin() {
    pinMode(_pinIN1, OUTPUT); pinMode(_pinIN2, OUTPUT); pinMode(_pinPot, INPUT);
    _currentPos = analogRead(_pinPot);
}

void TiltingActuator::setMotorSpeed(int speed) {
    // --- SOFT LIMIT CHECK ---
    // Voorkom dat we de actuator kapot trekken in manual mode
    // Als we te ver IN zijn (lage pot waarde) en we willen nog verder IN (negatieve speed): STOP
    if (_currentPos < _minPot && speed < 0) speed = 0;
    
    // Als we te ver UIT zijn (hoge pot waarde) en we willen nog verder UIT (positieve speed): STOP
    if (_currentPos > _maxPot && speed > 0) speed = 0;

    _speed = speed; // Opslaan voor debug

    if (speed == 0) {
        analogWrite(_pinIN1, 0); analogWrite(_pinIN2, 0);
    } else if (speed > 0) {
        analogWrite(_pinIN1, speed); analogWrite(_pinIN2, 0);
    } else {
        analogWrite(_pinIN1, 0); analogWrite(_pinIN2, -speed);
    }
}

void TiltingActuator::update() {
    _currentPos = analogRead(_pinPot);

    // Als we in MANUAL mode zitten, hoeft de PID niks te doen.
    // We vertrouwen op setManualSpeed()
    if (_manualMode) return;

    // --- AUTO MODE (PID) ---
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

void TiltingActuator::setTargetPosition(int distance) {
    _manualMode = false; // Schakel over naar AUTO
    distance = constrain(distance, 0, 100);
    _targetPos = map(distance, 0, 100, _minPot, _maxPot);
}

void TiltingActuator::setManualSpeed(float input) {
    _manualMode = true; // Schakel over naar MANUAL
    
    // Input is -1.0 tot 1.0 (van joystick)
    // Map naar PWM (-255 tot 255)
    int pwm = (int)(input * _maxPWM);
    
    // Simpele deadzone op de joystick zelf
    if (abs(pwm) < 40) pwm = 0; 

    setMotorSpeed(pwm);
}

int TiltingActuator::getCurrentPosition() { return map(_currentPos, _minPot, _maxPot, 0, 100); }
int TiltingActuator::getTargetPositionRaw() { return _targetPos; }
int TiltingActuator::getLastSpeed() { return _speed; }