#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

class MotorDriver {
public:
    MotorDriver(uint8_t deviceId);

    void update(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus);
    void setSpeed(float rpm);
    
    float getTargetRPM(); // Wat we willen
    float getActualRPM(); // Wat we meten via CAN

    // Nieuw: Verwerk inkomende berichten
    void parseCanMessage(const CAN_message_t &msg);

private:
    uint8_t _deviceId;
    float _targetRPM;
    float _actualRPM; // Nieuw
    elapsedMillis _sendTimer;
};

#endif