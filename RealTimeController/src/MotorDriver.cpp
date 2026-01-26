#include "MotorDriver.h"

// ID definities
static const uint32_t HEARTBEAT_ID = 0x2052480;
static const uint32_t SPEED_SET_ID = 0x2050480;
static const uint32_t STATUS_1_ID  = 0x2051840;

MotorDriver::MotorDriver(uint32_t deviceId) {
    _deviceId = deviceId;
    _currentRpmTarget = 0.0f;
    _actualRpm = 0.0f;
    _heartbeatTimer = 0;
    _commandTimer = 0;
}

// Update neemt nu de bus als referentie
void MotorDriver::update(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus) {
    
    // 1. Heartbeat elke 20ms
    if (_heartbeatTimer >= 20) {
        _heartbeatTimer = 0;
        
        CAN_message_t msg;
        msg.id = HEARTBEAT_ID;
        msg.flags.extended = 1;
        msg.len = 8;
        memset(msg.buf, 0xFF, 8);
        bus.write(msg);
    }

    // 2. Stuur snelheid commando elke 50ms
    if (_commandTimer >= 50) {
        _commandTimer = 0;
        
        CAN_message_t msg;
        msg.id = SPEED_SET_ID + _deviceId;
        msg.flags.extended = 1;
        msg.len = 8;
        
        // Pack float RPM (Little Endian)
        memcpy(&msg.buf[0], &_currentRpmTarget, 4);
        memset(&msg.buf[4], 0, 4);
        bus.write(msg);
    }
}

void MotorDriver::setSpeed(float rpm) {
    _currentRpmTarget = rpm;
}

void MotorDriver::stop() {
    _currentRpmTarget = 0.0f;
}

float MotorDriver::getActualRPM() {
    return _actualRpm;
}

void MotorDriver::processCanMessage(const CAN_message_t &msg) {
    // Check of dit bericht voor DEZE motor is
    if ((msg.id & 0x1FFFFFFF) == (STATUS_1_ID + _deviceId)) {
        memcpy(&_actualRpm, &msg.buf[0], 4);
    }
}