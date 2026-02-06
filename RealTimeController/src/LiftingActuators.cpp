#include "LiftingActuators.h"

// Constructor
LiftingActuators::LiftingActuators(HardwareSerial& serial, int pinPotA, int pinPotB) {
    _serial = &serial; // Sla de verwijzing naar Serial1 op
    _pinPotA = pinPotA; 
    _pinPotB = pinPotB;

    _minSpeed = 15;  // Minimale start snelheid (Sabertooth reageert vlot)
    _maxSpeed = 127; // MAXIMAAL 127 bij Sabertooth protocol! 
    _deadband = 3; 
    
    _kp = 4.0;       // Misschien opnieuw tunen, Sabertooth is krachtiger!
    _syncKp = 0.2; 
    
    _currentSpeedA = 0;
    _currentSpeedB = 0;
    _acceleration = 5.0; // Iets lager beginnen, kijken hoe hij reageert
    
    // Potentiometer Kalibratie (Check of dit nog klopt!)
    _minPotA = 905; // 0mm
    _maxPotA = 620; // 300mm    
    _minPotB = 905; 
    _maxPotB = 620;   
    
    _targetPosA = -1;
    _targetPosB = -1;
    _manualMode = true;
}

void LiftingActuators::begin() {
    // Start de Serieele poort voor de Sabertooth (9600 baud is default) [cite: 382]
    _serial->begin(9600);
    
    pinMode(_pinPotA, INPUT);  
    pinMode(_pinPotB, INPUT);
    
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
}

// --- SABERTOOTH PROTOCOL FUNCTIE ---
void LiftingActuators::sendSabertoothCommand(byte address, byte command, byte value) {
    // Protocol: Address, Command, Data, Checksum [cite: 373]
    byte checksum = (address + command + value) & 0b01111111; // [cite: 534, 535]
    _serial->write(address);
    _serial->write(command);
    _serial->write(value);
    _serial->write(checksum);
}

void LiftingActuators::setMotorASpeed(int speed) {
    // Soft limits A
    if (_currentPosA > _minPotA && speed < 0) speed = 0; 
    if (_currentPosA < _maxPotA && speed > 0) speed = 0;

    _speedA = speed;
    
    // Map speed (-127 tot 127) naar Sabertooth commando's
    // Motor 1 Address = 128 (default) 
    // Cmd 0 = Forward, Cmd 1 = Backward [cite: 445, 448]
    
    byte val = abs(speed);
    val = constrain(val, 0, 127); // Safety clip

    if (speed > 0) {
        sendSabertoothCommand(128, 0, val); // 0 = Drive Forward M1
    } else {
        sendSabertoothCommand(128, 1, val); // 1 = Drive Backward M1
    }
}

void LiftingActuators::setMotorBSpeed(int speed) {
    // Soft limits B
    if (_currentPosB > _minPotB && speed > 0) speed = 0;
    if (_currentPosB < _maxPotB && speed < 0) speed = 0;
  
    _speedB = speed;

    // Motor 2
    // Cmd 4 = Forward, Cmd 5 = Backward [cite: 466, 469]
    
    byte val = abs(speed);
    val = constrain(val, 0, 127);

    if (speed > 0) {
        sendSabertoothCommand(128, 4, val); // 4 = Drive Forward M2
    } else {
        sendSabertoothCommand(128, 5, val); // 5 = Drive Backward M2
    }
}

float LiftingActuators::rampValue(float current, int target, float rampRate) {
    if (current < target) {
        current += rampRate;
        if (current > target) current = target; 
    } else if (current > target) {
        current -= rampRate;
        if (current < target) current = target; 
    }
    return current;
}

void LiftingActuators::update() {
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
    
    if (_manualMode) return;
    if (_targetPosA == -1) return;
    
    int currentMM_A = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int currentMM_B = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);
    int targetMM    = map(_targetPosA,  _minPotA, _maxPotA, 0, _strokeLength); 
    
    int avgCurrent = (currentMM_A + currentMM_B) / 2;
    int distanceError = targetMM - avgCurrent;

    int targetSpeed_A = 0;
    int targetSpeed_B = 0;

    if (abs(distanceError) >= _deadband) {
        int baseSpeed = distanceError * _kp;
        int maxBaseSpeed = 100; // Iets lager dan 127 om ruimte te houden voor correcties
        
        if (baseSpeed > 0) baseSpeed = constrain(baseSpeed, _minSpeed, maxBaseSpeed);
        else baseSpeed = constrain(baseSpeed, -maxBaseSpeed, -_minSpeed);
        
        // Sync calculation
        int syncError = currentMM_A - currentMM_B; 
        int syncCorrection = syncError * _syncKp;
        
        targetSpeed_A = baseSpeed - syncCorrection;
        targetSpeed_B = baseSpeed + syncCorrection;
        
        // Safety constraints (Sabertooth max = 127)
        targetSpeed_A = constrain(targetSpeed_A, -127, 127);
        targetSpeed_B = constrain(targetSpeed_B, -127, 127);
    }
    
    _currentSpeedA = rampValue(_currentSpeedA, targetSpeed_A, _acceleration);
    _currentSpeedB = rampValue(_currentSpeedB, targetSpeed_B, _acceleration);
    
    setMotorASpeed((int)_currentSpeedA);
    setMotorBSpeed((int)_currentSpeedB);
}

void LiftingActuators::setTargetPosition(int mm) {
    _manualMode = false; 
    mm = constrain(mm, 0, _strokeLength);
    _targetPosA = map(mm, 0, _strokeLength, _minPotA, _maxPotA);
    _targetPosB = map(mm, 0, _strokeLength, _minPotB, _maxPotB);
}

void LiftingActuators::setManualSpeed(float input) {
    _manualMode = true; 
    _targetPosA = -1; 
    _targetPosB = -1;
    
    // Input is -1.0 tot 1.0. Sabertooth wil max 127.
    int rawSpeed = (int)(input * 127.0); 
    
    int targetSpeed_A = 0;
    int targetSpeed_B = 0;

    if (abs(rawSpeed) >= 20) { // Deadzone iets kleiner bij Serial
        int maxBaseSpeed = 100;
        int baseSpeed = constrain(rawSpeed, -maxBaseSpeed, maxBaseSpeed);

        int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
        int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);

        int syncError = mmA - mmB; 
        int syncCorrection = syncError * _syncKp;

        targetSpeed_A = baseSpeed - syncCorrection;
        targetSpeed_B = baseSpeed + syncCorrection;

        targetSpeed_A = constrain(targetSpeed_A, -127, 127);
        targetSpeed_B = constrain(targetSpeed_B, -127, 127);
    }
    
     _currentSpeedA = rampValue(_currentSpeedA, targetSpeed_A, _acceleration);
    _currentSpeedB = rampValue(_currentSpeedB, targetSpeed_B, _acceleration);
    
    setMotorASpeed((int)_currentSpeedA);
    setMotorBSpeed((int)_currentSpeedB);
}

int LiftingActuators::getCurrentPosition() { 
    int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);
    return (mmA + mmB) / 2; 
}

//Debug getters
int LiftingActuators::getPosA()             { return _currentPosA; }
int LiftingActuators::getPosB()             { return _currentPosB; }
int LiftingActuators::getTargetA()          { return _targetPosA; }
int LiftingActuators::getTargetB()          { return _targetPosB; }
int LiftingActuators::getSpeedA()           { return _speedA; }
int LiftingActuators::getSpeedB()           { return _speedB; }
bool LiftingActuators::isManualMode()       { return _manualMode; }
int LiftingActuators::getTargetPositionRaw(){ return _targetPosA; }