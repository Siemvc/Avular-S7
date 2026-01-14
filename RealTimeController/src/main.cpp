
#include <FlexCAN_T4.h>

// Use CAN3 hardware; the object name here is 'Can0' (that's fine).
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

// Match the CAN ID you set in the REV Client
const int SPARKMAX_ID = 3; 

// REV CAN extended IDs
enum ControlMode {
  Speed_Set        = 0x2050480, // Closed-loop velocity (units: RPM, 32-bit float)
  Heartbeat_Base   = 0x2052C80  // Non-RIO heartbeat (address + DeviceID)
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // SPARK MAX requires 1 Mbit/s
  Can0.begin();
  Can0.setBaudRate(1000000); 

  Serial.println("Teensy CAN start (Speed/RPM)");
}



// Send heartbeat addressed to this device
void sendHeartbeat(uint8_t devID) {
  CAN_message_t msg;
  msg.id = Heartbeat_Base + devID;   // address specific device
  msg.flags.extended = 1;
  msg.len = 8;
  for (uint8_t i = 0; i < 8; i++) msg.buf[i] = 0xFF; // "enabled" bytes
  Can0.write(msg);
}

// Send closed-loop velocity command in RPM
void sendSpeedSetRPM(int devID, float rpm) {
  CAN_message_t msg;
  msg.id = Speed_Set + devID;        // extended ID + device ID
  msg.flags.extended = 1;
  msg.len = 8;

  // First 4 bytes: IEEE-754 float (little-endian) for target RPM
  memcpy(msg.buf, &rpm, 4);

  // Aux/ArbFF/PID slot left at default (all zeros)
  msg.buf[4] = 0; msg.buf[5] = 0; msg.buf[6] = 0; msg.buf[7] = 0;

  Can0.write(msg);

  // Optional debug
  // Serial.print("SpeedSet ID: "); Serial.println(msg.id, HEX);
}

void loop() {
  // 1) Keep device enabled with heartbeat (each loop)
  sendHeartbeat(SPARKMAX_ID);

  // 2) Send speed setpoint in RPM (example: 1500 RPM)
  sendSpeedSetRPM(SPARKMAX_ID, 1500.0f);

  // Re-send every ~20 ms to avoid timeout
  delay(20);
}

