#include "MotorDriver.h"

static const uint32_t SMART_VELOCITY_BASE = 0x20504C0;
static const uint32_t STATUS_1_BASE     = 0x2051840; // Uit testcode
static const uint32_t SEND_INTERVAL     = 50;

MotorDriver::MotorDriver(uint8_t deviceId) {
    _deviceId = deviceId;
    _targetRPM = 0.0f;
    _actualRPM = 0.0f;
    _sendTimer = 0;
}

void MotorDriver::setSpeed(float rpm) {
    _targetRPM = rpm;
}

float MotorDriver::getTargetRPM() {
    return _targetRPM;
}

float MotorDriver::getActualRPM() {
    return _actualRPM;
}

void MotorDriver::update(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus) {
    if (_sendTimer < SEND_INTERVAL) return;
    _sendTimer = 0;

    CAN_message_t msg;
    msg.id = SMART_VELOCITY_BASE + _deviceId;
    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(&msg.buf[0], &_targetRPM, 4);
    memset(&msg.buf[4], 0, 4);
    bus.write(msg);
}

// NIEUW: Deze haalt de snelheid uit het CAN bericht
void MotorDriver::parseCanMessage(const CAN_message_t &msg) {
    // Check of dit bericht een Status bericht is voor DEZE motor
    if (msg.id == (STATUS_1_BASE + _deviceId) && msg.len >= 4) {
        memcpy(&_actualRPM, &msg.buf[0], 4);
    }
}