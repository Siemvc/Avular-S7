#include "LiftingActuators.h"

LiftingActuators::LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB) {
    _pinIN1A = pinIN1A; _pinIN2A = pinIN2A; _pinENA = pinENA;
    _pinIN1B = pinIN1B; _pinIN2B = pinIN2B; _pinENB = pinENB;
    _pinPotA = pinPotA; _pinPotB = pinPotB;

    _minPWM = 60; 
    _maxPWM = 180; //Do not make this higher than 250, to avoid overloading the motor driver! (See datasheet)
    _deadband = 3; 
    //Control gains
    _kp = 8.0; 
    _syncKp = 0.3; //Correction factor for synchronization
    //Acceleration
    _currentSpeedA = 0;
    _currentSpeedB = 0;
    _acceleration = 7.0; 
    //Left (A)
    _minPotA = 905; // 0mm
    _maxPotA = 620; // 300mm    
    //Right (B)
    _minPotB = 905; // 0mm
    _maxPotB = 620; // 300mm    
    //Initial states
    _targetPosA = -1;
    _targetPosB = -1;
    _manualMode = true;
}

void LiftingActuators::begin() {
    pinMode(_pinIN1A, OUTPUT); pinMode(_pinIN2A, OUTPUT); pinMode(_pinENA, OUTPUT); 
    pinMode(_pinIN1B, OUTPUT); pinMode(_pinIN2B, OUTPUT); pinMode(_pinENB, OUTPUT);
    pinMode(_pinPotA, INPUT);  pinMode(_pinPotB, INPUT);
    //Turn on lifting drivers
    digitalWrite(_pinENA, HIGH); digitalWrite(_pinENB, HIGH);
    // Initial position read
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
}

void LiftingActuators::setMotorASpeed(int speed) {
    // Soft limits A
    if (_currentPosA > _minPotA && speed < 0) speed = 0; 
    if (_currentPosA < _maxPotA && speed > 0) speed = 0;

    _speedA = speed;
    //Brake
    if (speed == 0) {
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, 0);
    } 
    //Move outward
    else if (speed > 0) { //
        analogWrite(_pinIN1A, speed); analogWrite(_pinIN2A, 0);
    } 
    //Move inward
    else { 
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, -speed);
    }
}

void LiftingActuators::setMotorBSpeed(int speed) {
    // Soft limits B
    if (_currentPosB > _minPotB && speed > 0) speed = 0;
    if (_currentPosB < _maxPotB && speed < 0) speed = 0;
  
    _speedB = speed;
    //Brake
    if (speed == 0) {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, 0);
    } 
    //Move outward
    else if (speed > 0) {
        analogWrite(_pinIN1B, speed); analogWrite(_pinIN2B, 0);
    } 
    //Move inward
    else {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, -speed);
    }
}

float LiftingActuators::rampValue(float current, int target, float rampRate) {
    if (current < target) {
        current += rampRate;
        if (current > target) current = target; // Don't overshoot
    } else if (current > target) {
        current -= rampRate;
        if (current < target) current = target; // Don't overshoot
    }
    return current;
}

void LiftingActuators::update() {
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
    
    if (_manualMode) return;
    if (_targetPosA == -1) return;
    //Convert everything to mm for easier calculations
    int currentMM_A = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int currentMM_B = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);
    int targetMM    = map(_targetPosA,  _minPotA, _maxPotA, 0, _strokeLength); //Goal of A is leading
    //Calculate average position and error
    int avgCurrent = (currentMM_A + currentMM_B) / 2;
    int distanceError = targetMM - avgCurrent;

    int targetPWM_A = 0;
    int targetPWM_B = 0;

    if (abs(distanceError) >= 3) {
        int basePWM = distanceError * _kp;
        int maxBaseSpeed = 200; 
        
        if (basePWM > 0) basePWM = constrain(basePWM, _minPWM, maxBaseSpeed);
        else basePWM = constrain(basePWM, -maxBaseSpeed, -_minPWM);
        // Sync calculation
        int syncError = currentMM_A - currentMM_B; 
        int syncCorrection = syncError * _syncKp;
        // Calculate DESIRED PWM
        targetPWM_A = basePWM - syncCorrection;
        targetPWM_B = basePWM + syncCorrection;
        // Safety constraints
        targetPWM_A = constrain(targetPWM_A, -_maxPWM, _maxPWM);
        targetPWM_B = constrain(targetPWM_B, -_maxPWM, _maxPWM);
    }
    
    _currentSpeedA = rampValue(_currentSpeedA, targetPWM_A, _acceleration);
    _currentSpeedB = rampValue(_currentSpeedB, targetPWM_B, _acceleration);
    // Send the ramped value to the motors
    setMotorASpeed((int)_currentSpeedA);
    setMotorBSpeed((int)_currentSpeedB);
}


void LiftingActuators::setTargetPosition(int mm) {
    _manualMode = false; 
    mm = constrain(mm, 0, _strokeLength); // 0 to 300
    
    _targetPosA = map(mm, 0, _strokeLength, _minPotA, _maxPotA);
    _targetPosB = map(mm, 0, _strokeLength, _minPotB, _maxPotB);
}

void LiftingActuators::setManualSpeed(float input) {
    _manualMode = true; 
    _targetPosA = -1; // Reset auto targets
    _targetPosB = -1;
    
    //Deadzone and base PWM calculation
    int rawPWM = (int)(input * _maxPWM);
    
    int targetPWM_A = 0;
    int targetPWM_B = 0;

    // Only if we have input greater than deadzone
    if (abs(rawPWM) >= 40) {
        int maxBaseSpeed = 200;
        int basePWM = constrain(rawPWM, -maxBaseSpeed, maxBaseSpeed);

        int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
        int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);

        int syncError = mmA - mmB; 
        int syncCorrection = syncError * _syncKp;

        targetPWM_A = basePWM - syncCorrection;
        targetPWM_B = basePWM + syncCorrection;

        targetPWM_A = constrain(targetPWM_A, -_maxPWM, _maxPWM);
        targetPWM_B = constrain(targetPWM_B, -_maxPWM, _maxPWM);
    }
    // Ramp the speeds for smoothness
     _currentSpeedA = rampValue(_currentSpeedA, targetPWM_A, _acceleration);
    _currentSpeedB = rampValue(_currentSpeedB, targetPWM_B, _acceleration);
    
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