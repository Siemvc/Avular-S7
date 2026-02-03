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
    float _syncKp;
    
    //Configuration
    const int _strokeLength = 300; 
    int _minPotA, _maxPotA; 
    int _minPotB, _maxPotB; 
    
    int _targetPosA, _targetPosB; //Targetposition in mm
    int _currentPosA, _currentPosB;
    
    int _speedA, _speedB;
    bool _manualMode; 

    float _currentSpeedA = 0; 
    float _currentSpeedB = 0;
    float _acceleration = 5.0; // Speed change per update call

    float rampValue(float current, int target, float rampRate);

    void setMotorASpeed(int speed);
    void setMotorBSpeed(int speed);

public:
    LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB);
    void begin();
    void update();
    
    void setTargetPosition(int mm); 
    void setManualSpeed(float input);       

    int getCurrentPosition(); 
    
    // DEBUG GETTERS 
    int getPosA();    
    int getPosB();
    int getTargetA(); 
    int getTargetB();
    int getSpeedA(); 
    int getSpeedB();
    bool isManualMode();
    int getTargetPositionRaw(); // Gives target of actuator A in raw potentiometer value
};

#endif