#ifndef LIFTING_ACTUATORS_H
#define LIFTING_ACTUATORS_H

#include <Arduino.h>

class LiftingActuators {
private:
    int _pinIN1A, _pinIN2A, _pinENA;
    int _pinIN1B, _pinIN2B, _pinENB;
    int _pinPotA, _pinPotB;

    int _minPWM, _maxPWM, _deadband;
    float _kp;
    int _minPot, _maxPot;
    
    int _targetPos;
    int _currentPos;
    int _speed;
    bool _manualMode; 

    void setMotorSpeed(int speed);

public:
    LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB);
    void begin();
    void update();
    
    void setTargetPosition(int percentage); // Auto
    void setManualSpeed(float input);       // Manual

    int getCurrentPosition();
    int getTargetPositionRaw();
    int getLastSpeed();
};

#endif