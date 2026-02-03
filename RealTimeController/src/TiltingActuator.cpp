#include "TiltingActuator.h"

TiltingActuator::TiltingActuator(int pinIN1, int pinIN2, int pinPWM, int pinPot) {
    _pinIN1 = pinIN1;
    _pinIN2 = pinIN2;
    _pinPWM = pinPWM;
    _pinPot = pinPot;

    _minPWM = 60;   _maxPWM = 250; // Do not exceed 250, to avoid overloading motor driver
    _deadband = 10; _kp = 6.0;
    _minPot = 215;   _maxPot = 550;  //Min and max pysical potmeter values of bucket tilt
    _targetPos = -1;
    _manualMode = true; //Begin in manual mode
    _lastSpeed = 0;
}

void TiltingActuator::begin() {
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);
    pinMode(_pinPWM, OUTPUT);
    pinMode(_pinPot, INPUT);
    //Initialize motor stopped
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, LOW);
    analogWrite(_pinPWM, 0);
    _currentPos = analogRead(_pinPot);
}

void TiltingActuator::setMotorSpeed(int speed) {
    // Soft Limits 
    _currentPos = analogRead(_pinPot);
    if (_currentPos >= _maxPot && speed > 0) speed = 0;
    if (_currentPos <= _minPot && speed < 0) speed = 0;

    _lastSpeed = speed;
    int pwmVal = abs(speed); // PWM has to be positive

    if (speed == 0) {
        // STOP
        digitalWrite(_pinIN1, LOW);
        digitalWrite(_pinIN2, LOW);
        analogWrite(_pinPWM, 0);
    } 
    else if (speed > 0) {
        //Outward (IN1=HIGH, IN2=LOW)
        digitalWrite(_pinIN1, HIGH);
        digitalWrite(_pinIN2, LOW);
        analogWrite(_pinPWM, pwmVal);
    } 
    else {
        // Inward (IN1=LOW, IN2=HIGH)
        digitalWrite(_pinIN1, LOW);
        digitalWrite(_pinIN2, HIGH);
        analogWrite(_pinPWM, pwmVal);
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
        int pwm = error * _kp;
         // Clip to max speed
        if (pwm > 0) pwm = constrain(pwm, _minPWM, _maxPWM);
        else pwm = constrain(pwm, -_maxPWM, -_minPWM);
        setMotorSpeed(pwm);
    } else {
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
int TiltingActuator::getLastSpeed() { return _lastSpeed; }