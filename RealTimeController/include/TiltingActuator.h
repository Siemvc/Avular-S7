#ifndef TILTING_ACTUATOR_H
#define TILTING_ACTUATOR_H

#include <Arduino.h>

class TiltingActuator {
private:
    int _pinIN1, _pinIN2, _pinPot;
    int _minPWM, _maxPWM, _deadband;
    float _kp;
    int _minPot, _maxPot;
    
    int _targetPos;
    int _currentPos;
    int _speed;
    
    
    bool _manualMode; 

    void setMotorSpeed(int speed);

public:
    TiltingActuator(int pinIN1, int pinIN2, int pinPot);
    void begin();
    
    // Regular update call for position control and min max distance checking
    void update(); 
    
    //Auto: Go to position with P-controller
    void setTargetPosition(int percentage); 
    
    // Manual: Direct speed (-1.0 to 1.0)
    void setManualSpeed(float input); 

    // Getters for debugging
    int getCurrentPosition();
    int getTargetPositionRaw();
    int getLastSpeed();
};

#endif