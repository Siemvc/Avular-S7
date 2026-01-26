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
    
    // NIEUW: Houdt bij of we handmatig sturen of automatisch
    bool _manualMode; 

    void setMotorSpeed(int speed);

public:
    TiltingActuator(int pinIN1, int pinIN2, int pinPot);
    void begin();
    
    // Update moet altijd draaien (voor PID én safety limits)
    void update(); 
    
    // AUTO: Gaat naar positie met PID
    void setTargetPosition(int percentage); 
    
    // MANUAL: Directe snelheid (-1.0 tot 1.0)
    void setManualSpeed(float input); 

    // Getters voor debug
    int getCurrentPosition();
    int getTargetPositionRaw();
    int getLastSpeed();
};

#endif