//Code voor meerdere motoren aan te sturen via CAN op een Teensy 4.1 board met microROS.
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

// ---------------- CONFIG ----------------
const uint32_t CAN_BAUD = 1000000;

// Motor configuratie
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4};
const uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

// Extended CAN IDs
enum ControlMode {
  SmartVelocity_Set = 0x20504C0,
  Heartbeat_Base    = 0x2052C80,
  Status_1          = 0x2051840
};

// Heartbeat interval
const uint32_t HEARTBEAT_INTERVAL_MS = 20;

// ---------------- VARIABLES ----------------
float motorSetpointRPM[MOTOR_COUNT] = {0};
float actualVelocityRPM[MOTOR_COUNT] = {0};

uint32_t lastHeartbeat = 0;

// microROS variables
rcl_subscription_t motor_subscribers[MOTOR_COUNT];
std_msgs__msg__Float32 motor_msg[MOTOR_COUNT];
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Forward declarations
void handleStatusFrame(const CAN_message_t &msg);
void motor_callback(const void * msgin, void * motorIndex);

// Callback function for motor subscriptions
void motor_callback(const void * msgin, void * motorIndex) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  int index = (int)motorIndex;
  
  if (index >= 0 && index < MOTOR_COUNT) {
    motorSetpointRPM[index] = msg->data;
    sendSmartVelocity(MOTOR_IDS[index], msg->data);
  }
}

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
}

// ---------------- CAN SEND ----------------
void sendHeartbeat() {
  CAN_message_t msg;
  msg.id = Heartbeat_Base;
  msg.flags.extended = 1;
  msg.len = 8;
  memset(msg.buf, 0xFF, 8);
  Can1.write(msg);
}

void sendSmartVelocity(uint8_t motorID, float rpm) {
  CAN_message_t msg;
  msg.id = SmartVelocity_Set + motorID;
  msg.flags.extended = 1;
  msg.len = 8;

  memcpy(msg.buf, &rpm, 4);
  memset(msg.buf + 4, 0, 4);

  Can1.write(msg);
}

// ---------------- STATUS HANDLING ----------------
void handleStatusFrame(const CAN_message_t &msg) {
  if (!msg.flags.extended || msg.len < 4)
    return;

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if ((uint32_t)msg.id == (Status_1 + MOTOR_IDS[i])) {
      memcpy(&actualVelocityRPM[i], msg.buf, 4);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize CAN
  initCAN();

  // Initialize microROS with serial transport
  allocator = rcl_get_default_allocator();
  
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_controller", "", &support));

  // Create subscribers for each motor
  char topic_name[30];
  for (int i = 0; i < MOTOR_COUNT; i++) {
    snprintf(topic_name, sizeof(topic_name), "/motor/%d/setpoint", MOTOR_IDS[i]);
    
    RCCHECK(rclc_subscription_init_default(
      &motor_subscribers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      topic_name));
  }

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, MOTOR_COUNT, &allocator));

  // Add subscriptions to executor with callbacks
  for (int i = 0; i < MOTOR_COUNT; i++) {
    RCCHECK(rclc_executor_add_subscription(
      &executor,
      &motor_subscribers[i],
      &motor_msg[i],
      &motor_callback,
      (void *)(intptr_t)i));
  }
}

// ---------------- LOOP ----------------
void loop() {
  uint32_t now = millis();

  // Send heartbeat periodically
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  // Read CAN messages and execute microROS callbacks
  CAN_message_t rx_msg;
  while (Can1.read(rx_msg)) {
    handleStatusFrame(rx_msg);
  }

  // Process microROS subscriptions
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}