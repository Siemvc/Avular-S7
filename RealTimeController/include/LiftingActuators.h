#ifndef LIFTINGACTUATORS_H
#define LIFTINGACTUATORS_H

#include <Arduino.h>

class LiftingActuators {
  private:
    // Pin configuratie
    int _pinIN1A;
    int _pinIN1B;
    int _pinIN2A;
    int _pinIN2B;
    int _pinENA;
    int _pinENB;
    int _pinPotA;
    int _pinPotB;
    
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
    int _speed;

    // Interne hulpfunctie (hoeft main.cpp niet te zien)
    void setMotorSpeed(int speed);


  public:
    // Constructor: Hier geef je de pinnen en instellingen door
    LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB);

    // Setup functie (aanroepen in setup())
    void begin();

    // Loop functie (aanroepen in loop())
    void update();

    // Functies om de actuator te besturen
    void setTargetPosition(int distance); // 0-300mm
    int getCurrentPosition();               // ADC waarde
};

#endif