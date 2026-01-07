#ifndef TILTING_ACTUATOR_H
#define TILTING_ACTUATOR_H

#include <Arduino.h>

class TiltingActuator {
private:
    // Pinnen (Intern gebruik)
    int _pinUp;
    int _pinDown;
    int _pinPot;

    // Instellingen (Intern gebruik)
    int _minPWM;
    int _maxPWM;
    int _deadband;
    int _kp;
    int _minPot;
    int _maxPot;

    // Status variabelen
    int _targetPos;
    int _currentPos;

    // Interne hulpfunctie (hoeft main.cpp niet te zien)
    void setMotorSpeed(int speed);

public:
    // Constructor: Hier geef je de pinnen en instellingen door
    TiltingActuator(int pinUp, int pinDown, int pinPot);

    // Setup functie (aanroepen in setup())
    void begin();

    // Loop functie (aanroepen in loop())
    void update();

    // Functies om de actuator te besturen
    void setTargetPosition(int percentage); // 0-100%
    int getCurrentPosition();               // ADC waarde
};

#endif