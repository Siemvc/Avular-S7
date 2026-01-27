#include "TiltingActuator.h"

TiltingActuator::TiltingActuator(int pinIN1, int pinIN2, int pinPot) {
    _pinIN1 = pinIN1; _pinIN2 = pinIN2; _pinPot = pinPot;
    _minPWM = 60;   _maxPWM = 250;
    _deadband = 10; _kp = 6.0;
    _minPot = 215;   _maxPot = 550;  //Min and max pysical potmeter values
    _targetPos = -1;
    _manualMode = true; //Begin in manual mode
}

void TiltingActuator::begin() {
    pinMode(_pinIN1, OUTPUT); pinMode(_pinIN2, OUTPUT); pinMode(_pinPot, INPUT);
    _currentPos = analogRead(_pinPot);
}

void TiltingActuator::setMotorSpeed(int speed) {
    //Soft limiter
    if (_currentPos < _minPot && speed < 0) speed = 0;
    if (_currentPos > _maxPot && speed > 0) speed = 0;

    _speed = speed; //Safe for debugging

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
    //If we are in manual mode, skip auto control
    if (_manualMode) return;

    // Auto mode, P-controller
    if (_targetPos == -1) return;

    int error = _targetPos - _currentPos;

    if (abs(error) > _deadband) {
        int calculatedSpeed = error * _kp; // P-control
        if (calculatedSpeed > 0) calculatedSpeed = constrain(calculatedSpeed, _minPWM, _maxPWM);
        else calculatedSpeed = constrain(calculatedSpeed, -_maxPWM, -_minPWM);
        setMotorSpeed(calculatedSpeed);

    } else { // Within deadband
        setMotorSpeed(0);
    }
}

void TiltingActuator::setTargetPosition(int distance) {
    _manualMode = false; // Switch to auto mode
    distance = constrain(distance, 25, 69);
    _targetPos = map(distance, 0, 100, _minPot, _maxPot);
}

void TiltingActuator::setManualSpeed(float input) {
    _manualMode = true; //Switch to manual mode
    //Input is -1.0 to 1.0 from joystick, map to PWM (-255 to 255)
    int pwm = (int)(input * _maxPWM);

    //Simple deadzone on the joystick itself
    if (abs(pwm) < 40) pwm = 0;
    setMotorSpeed(pwm);
}

// Getters for debugging
int TiltingActuator::getCurrentPosition() { return map(_currentPos, _minPot, _maxPot, 0, 100); }
int TiltingActuator::getTargetPositionRaw() { return _targetPos; }
int TiltingActuator::getLastSpeed() { return _speed; }