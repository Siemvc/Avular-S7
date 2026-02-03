#include "MotorDriver.h"

static const uint32_t SMART_VELOCITY_BASE = 0x20504C0;
static const uint32_t STATUS_1_BASE     = 0x2051840; 
static const uint32_t SEND_INTERVAL     = 50; //in ms

MotorDriver::MotorDriver(uint8_t deviceId) {
    _deviceId = deviceId;
    _targetRPM = 0.0f;
    _currentSetpoint = 0.0f;
    _actualRPM = 0.0f;
    _acceleration = 12000.0f;
    _sendTimer = 0;
}

void MotorDriver::setSpeed(float rpm) {
    _targetRPM = rpm;
}

void MotorDriver::setAcceleration(float rpmPerSecond) {
    _acceleration = rpmPerSecond;
}

float MotorDriver::getTargetRPM() {return _targetRPM;}
float MotorDriver::getActualRPM() {return _actualRPM;}
float MotorDriver::getCurrentSetpoint() {return _currentSetpoint;}

void MotorDriver::update(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus) { //creates the can message
    float dt = (float)_sendTimer / 1000.0f; 
    _sendTimer = 0;

    float maxStep = _acceleration * dt;

    if (_currentSetpoint < _targetRPM) {
        _currentSetpoint += maxStep;
        if (_currentSetpoint > _targetRPM) _currentSetpoint = _targetRPM;
    }
    else if (_currentSetpoint > _targetRPM) {
        _currentSetpoint -= maxStep;
        if (_currentSetpoint < _targetRPM) _currentSetpoint = _targetRPM;
    }

    CAN_message_t msg;
    msg.id = SMART_VELOCITY_BASE + _deviceId;
    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(&msg.buf[0], &_currentSetpoint, 4);
    memset(&msg.buf[4], 0, 4);
    bus.write(msg);
}

// This retrieves the speed from the CAN message
void MotorDriver::parseCanMessage(const CAN_message_t &msg) {
    // Check if this message is a Status message for THIS motor
    if (msg.id == (STATUS_1_BASE + _deviceId) && msg.len >= 4) {
        memcpy(&_actualRPM, &msg.buf[0], 4);
    }
}