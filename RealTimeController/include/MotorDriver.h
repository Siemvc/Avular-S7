#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

class MotorDriver {
public:
    // Constructor
    MotorDriver(uint32_t deviceId);

    // Update logica (hartslag sturen, etc)
    // We geven nu de CAN bus mee als parameter!
    void update(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus);
    
    void setSpeed(float rpm);
    void stop();
    float getActualRPM();
    
    // Deze roepen we aan vanuit main.cpp als er een bericht binnenkomt
    void processCanMessage(const CAN_message_t &msg);

private:
    uint32_t _deviceId;
    float _currentRpmTarget;
    float _actualRpm;
    
    elapsedMillis _heartbeatTimer;
    elapsedMillis _commandTimer;
};

#endif