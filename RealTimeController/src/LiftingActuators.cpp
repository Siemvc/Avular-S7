#include "LiftingActuators.h"

LiftingActuators::LiftingActuators(int pinIN1A, int pinIN1B, int pinIN2A, int pinIN2B, int pinENA, int pinENB, int pinPotA, int pinPotB) {
    _pinIN1A = pinIN1A; _pinIN2A = pinIN2A; _pinENA = pinENA;
    _pinIN1B = pinIN1B; _pinIN2B = pinIN2B; _pinENB = pinENB;
    _pinPotA = pinPotA; _pinPotB = pinPotB;

    _minPWM = 60; 
    _maxPWM = 250; //Do not make this higher than 250, to avoid overloading the motor driver! (See datasheet)
    _deadband = 10; 

    _kp = 8.0; 
    _syncKp = 10.0; //Correction factor for synchronization
    
    //Left (A)
    _minPotA = 985; // 0mm
    _maxPotA = 385; // 300mm
    
    //Right (B)
    _minPotB = 946; // 0mm
    _maxPotB = 403; // 300mm

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
    
    _currentPosA = analogRead(_pinPotA);
    _currentPosB = analogRead(_pinPotB);
}

void LiftingActuators::setMotorASpeed(int speed) {
    // Soft limits A
    if (_currentPosA > _minPotA && speed < 0) speed = 0; 
    if (_currentPosA < _maxPotA && speed > 0) speed = 0;

    _speedA = speed;
    if (speed == 0) {
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, 0);
    } else if (speed > 0) { 
        analogWrite(_pinIN1A, speed); analogWrite(_pinIN2A, 0);
    } else { 
        analogWrite(_pinIN1A, 0); analogWrite(_pinIN2A, -speed);
    }
}

void LiftingActuators::setMotorBSpeed(int speed) {
    // Soft limits B
    if (_currentPosB > _minPotB && speed < 0) speed = 0;
    if (_currentPosB < _maxPotB && speed > 0) speed = 0;

    _speedB = speed;
    if (speed == 0) {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, 0);
    } else if (speed > 0) {
        analogWrite(_pinIN1B, speed); analogWrite(_pinIN2B, 0);
    } else {
        analogWrite(_pinIN1B, 0); analogWrite(_pinIN2B, -speed);
    }
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

    if (abs(distanceError) < 3) {
        setMotorASpeed(0);
        setMotorBSpeed(0);
        return;
    }
    //Calculate base PWM from distance error
    int basePWM = distanceError * _kp;
    int maxBaseSpeed = 200;  //Take ~80% of max speed for synchronization corrections
    
    if (basePWM > 0) basePWM = constrain(basePWM, _minPWM, maxBaseSpeed);
    else basePWM = constrain(basePWM, -maxBaseSpeed, -_minPWM);

    // Synchronization error , if A is ahead, B needs to catch up, vice versa
    int syncError = currentMM_A - currentMM_B; 
    //Calculate correction based on sync error
    int syncCorrection = syncError * _syncKp;

    //Final PWM values
    int pwmA = basePWM - syncCorrection;
    int pwmB = basePWM + syncCorrection;

    //Safety constraints
    pwmA = constrain(pwmA, -_maxPWM, _maxPWM);
    pwmB = constrain(pwmB, -_maxPWM, _maxPWM);
    //Send PWM to motors
    setMotorASpeed(pwmA);
    setMotorBSpeed(pwmB);
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
    if (abs(rawPWM) < 40) {
        setMotorASpeed(0);
        setMotorBSpeed(0);
        return;
    }

    //Cap base speed to leave room for synchronization correction
    int maxBaseSpeed = 200;
    int basePWM = constrain(rawPWM, -maxBaseSpeed, maxBaseSpeed);

    //Determine position in mm
    int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);

    //Calculate synchronization error, positive if A is higher than B
    int syncError = mmA - mmB; 
    
    //Calculate correction 
    int syncCorrection = syncError * _syncKp;

    //Final speeds   
    int pwmA = basePWM - syncCorrection;
    int pwmB = basePWM + syncCorrection;

    //Safety: Clip to hardware limit
    pwmA = constrain(pwmA, -_maxPWM, _maxPWM);
    pwmB = constrain(pwmB, -_maxPWM, _maxPWM);
    
    setMotorASpeed(pwmA);
    setMotorBSpeed(pwmB);
}

int LiftingActuators::getCurrentPosition() { 
    int mmA = map(_currentPosA, _minPotA, _maxPotA, 0, _strokeLength);
    int mmB = map(_currentPosB, _minPotB, _maxPotB, 0, _strokeLength);
    return (mmA + mmB) / 2; 
}

//Debug getters
int LiftingActuators::getPosA()    { return _currentPosA; }
int LiftingActuators::getPosB()    { return _currentPosB; }
int LiftingActuators::getTargetA() { return _targetPosA; }
int LiftingActuators::getTargetB() { return _targetPosB; }
int LiftingActuators::getSpeedA()  { return _speedA; }
int LiftingActuators::getSpeedB()  { return _speedB; }
bool LiftingActuators::isManualMode() { return _manualMode; }
int LiftingActuators::getTargetPositionRaw() { return _targetPosA; }