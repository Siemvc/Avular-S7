//Code voor meerdere motoren aan te sturen via CAN op een Teensy 4.1 board.
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// ---------------- CONFIG ----------------
static const uint32_t CAN_BAUD = 1000000;

// Motor CAN IDs
static const uint8_t NUM_MOTORS = 4;
static const uint32_t MOTOR_IDS[NUM_MOTORS] = {1, 2, 3, 4};

// Speed settings
static const float SPEED_A_RPM = 500.0f;
static const float SPEED_B_RPM = 1500.0f;
static const uint32_t SPEED_SWITCH_MS = 5000;

// REV CAN base IDs (extended)
static const uint32_t HEARTBEAT_ID = 0x2052480;
static const uint32_t SPEED_SET_ID = 0x2050480;
static const uint32_t STATUS_1_ID  = 0x2051840;
// ---------------------------------------

elapsedMillis speedTimer;
elapsedMillis heartbeatTimer;

float currentSetpoint = SPEED_A_RPM;
float actualVelocityRPM[NUM_MOTORS] = {0};

// ---------------- UTILITIES ----------------

void sendHeartbeat() {
  CAN_message_t msg;
  msg.id = HEARTBEAT_ID;
  msg.flags.extended = 1;
  msg.len = 8;

  for (int i = 0; i < 8; i++) {
    msg.buf[i] = 0xFF;
  }

  Can1.write(msg);
}

void sendSpeedCommandAll(float rpm) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.len = 8;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    msg.id = SPEED_SET_ID + MOTOR_IDS[i];

    memcpy(&msg.buf[0], &rpm, 4);
    for (int b = 4; b < 8; b++) {
      msg.buf[b] = 0;
    }

    Can1.write(msg);
  }
}

void handleStatusFrame(const CAN_message_t &msg) {
  uint32_t baseId = msg.id & 0x1FFFFFFF;

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (baseId == (STATUS_1_ID + MOTOR_IDS[i])) {
      memcpy(&actualVelocityRPM[i], &msg.buf[0], 4);

      Serial.print("Motor ");
      Serial.print(MOTOR_IDS[i]);
      Serial.print(" Velocity (RPM): ");
      Serial.println(actualVelocityRPM[i]);
    }
  }
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Teensy 4.1 - 4 Motor CAN Velocity Control");

  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();

  Can1.onReceive([](const CAN_message_t &msg) {
    handleStatusFrame(msg);
  });

  speedTimer = 0;
  heartbeatTimer = 0;
}

// ---------------- LOOP ----------------

void loop() {

  // Heartbeat every 20 ms
  if (heartbeatTimer >= 20) {
    heartbeatTimer = 0;
    sendHeartbeat();
  }

  // Switch speed every 5 seconds
  if (speedTimer >= SPEED_SWITCH_MS) {
    speedTimer = 0;

    if (currentSetpoint == SPEED_A_RPM) {
      currentSetpoint = SPEED_B_RPM;
    } else {
      currentSetpoint = SPEED_A_RPM;
    }

    Serial.print("New Commanded Speed (RPM): ");
    Serial.println(currentSetpoint);
  }

  // Continuously command all motors
  sendSpeedCommandAll(currentSetpoint);

  delay(10);
}
