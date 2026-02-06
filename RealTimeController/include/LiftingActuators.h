#ifndef LIFTING_ACTUATORS_H
#define LIFTING_ACTUATORS_H

#include <Arduino.h>

class LiftingActuators {
private:
    HardwareSerial* _serial; // Pointer naar de Seriële poort (bijv. Serial1)
    
    int _pinPotA, _pinPotB;

    int _minSpeed, _maxSpeed, _deadband;
    float _kp;
    float _syncKp;
    
    // Configuration
    const int _strokeLength = 300; 
    int _minPotA, _maxPotA; 
    int _minPotB, _maxPotB; 
    
    int _targetPosA, _targetPosB; 
    int _currentPosA, _currentPosB;
    
    int _speedA, _speedB;
    bool _manualMode; 

    float _currentSpeedA = 0; 
    float _currentSpeedB = 0;
    float _acceleration = 7.0; 

    float rampValue(float current, int target, float rampRate);

    // Nieuwe interne functie voor Sabertooth communicatie
    void sendSabertoothCommand(byte address, byte command, byte value);
    void setMotorASpeed(int speed);
    void setMotorBSpeed(int speed);

public:
    // Constructor aangepast: verwacht nu Serial en Potentiometers
    LiftingActuators(HardwareSerial& serial, int pinPotA, int pinPotB);
    
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
    int getTargetPositionRaw(); 
};

#endif