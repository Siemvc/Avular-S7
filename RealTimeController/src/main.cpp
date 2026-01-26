//Code voor meerdere motoren aan te sturen via CAN op een Teensy 4.1 board.
#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

// ---------------- CONFIG ----------------
const uint32_t CAN_BAUD = 1000000;

// Motor configuratie (hier bepaal je het aantal motoren)
const uint8_t MOTOR_IDS[] = {1, 2};
const uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

// Extended CAN IDs
enum ControlMode {
  SmartVelocity_Set = 0x20504C0,
  Heartbeat_Base    = 0x2052C80,
  Status_1          = 0x2051840
};

// Heartbeat interval
const uint32_t HEARTBEAT_INTERVAL_MS = 20;
const uint32_t VELOCITY_SEND_INTERVAL_MS = 50;  // Throttle velocity commands

// ---------------- VARIABLES ----------------
float motorSetpointRPM[MOTOR_COUNT] = {0};
float actualVelocityRPM[MOTOR_COUNT] = {0};

uint32_t lastHeartbeat = 0;
uint32_t lastVelocitySend = 0;

// Forward declaration for lambda in initCAN
void handleStatusFrame(const CAN_message_t &msg);

// ---------------- CAN INIT ----------------
void initCAN() {
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive([](const CAN_message_t &msg) {
    handleStatusFrame(msg);
  });

  Serial.println("Teensy 4.1 FlexCAN initialized");
}

// ---------------- SERIAL PARSER ----------------
void handleSerialInput() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0)
    return;

  char buffer[128];
  line.toCharArray(buffer, sizeof(buffer));

  char *token = strtok(buffer, " ");
  uint8_t index = 0;

  while (token != nullptr && index < MOTOR_COUNT) {
    motorSetpointRPM[index] = atof(token);
    index++;
    token = strtok(nullptr, " ");
  }

  if (index != MOTOR_COUNT) {
    Serial.println("Fout: aantal snelheden komt niet overeen met aantal motoren");
    return;
  }

  Serial.print("Nieuwe setpoints: ");
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    Serial.print(motorSetpointRPM[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// ---------------- CAN SEND ----------------
void sendHeartbeat() {
  CAN_message_t msg;
  msg.id = Heartbeat_Base;
  msg.flags.extended = 1;
  msg.len = 8;
  memset(msg.buf, 0xFF, 8);
  Can1.write(msg);
  
  // Debug output
  Serial.print("[CAN TX] Heartbeat - ID: 0x");
  Serial.print(msg.id, HEX);
  Serial.print(" Data: ");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(msg.buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void sendSmartVelocityToAll() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    CAN_message_t msg;
    msg.id = SmartVelocity_Set + MOTOR_IDS[i];
    msg.flags.extended = 1;
    msg.len = 8;

    memcpy(msg.buf, &motorSetpointRPM[i], 4);
    memset(msg.buf + 4, 0, 4);

    Can1.write(msg);
    
    // Debug output
    Serial.print("[CAN TX] SmartVelocity - Motor ");
    Serial.print(MOTOR_IDS[i]);
    Serial.print(" ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print(" RPM Setpoint: ");
    Serial.print(motorSetpointRPM[i]);
    Serial.print(" Data: ");
    for (uint8_t j = 0; j < msg.len; j++) {
      Serial.print(msg.buf[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// ---------------- STATUS HANDLING ----------------
void handleStatusFrame(const CAN_message_t &msg) {
  if (!msg.flags.extended || msg.len < 4)
    return;

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if ((uint32_t)msg.id == (Status_1 + MOTOR_IDS[i])) {
      memcpy(&actualVelocityRPM[i], msg.buf, 4);

      Serial.print("Motor ");
      Serial.print(MOTOR_IDS[i]);
      Serial.print(" RPM: ");
      Serial.println(actualVelocityRPM[i]);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Teensy 4.1 Multi-Motor Serial Velocity Control");
  Serial.println("Motor spinning at 500 RPM");

  // Set motor to spin at 500 RPM
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    motorSetpointRPM[i] = 4000.0;
  }

  initCAN();
}

// ---------------- LOOP ----------------
void loop() {
  uint32_t now = millis(); //don't remove this

  handleSerialInput();

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  if (now - lastVelocitySend >= VELOCITY_SEND_INTERVAL_MS) {
    lastVelocitySend = now;
    sendSmartVelocityToAll();
  }

  CAN_message_t rx_msg;
  while (Can1.read(rx_msg)) {
    handleStatusFrame(rx_msg);
  }
}