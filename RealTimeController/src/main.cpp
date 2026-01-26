#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// Macro for error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

// ---------------- CONFIG ----------------
const uint32_t CAN_BAUD = 1000000;
const uint8_t MOTOR_IDS[] = {1, 2, 3, 4};
const uint8_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

enum ControlMode {
  SmartVelocity_Set = 0x20504C0,
  Heartbeat_Base    = 0x2052C80,
  Status_1          = 0x2051840
};

const uint32_t HEARTBEAT_INTERVAL_MS = 20;

// ---------------- VARIABLES ----------------
float motorSetpointRPM[MOTOR_COUNT] = {0};
float actualVelocityRPM[MOTOR_COUNT] = {0};
uint32_t lastHeartbeat = 0;

// microROS variables
rcl_subscription_t motor_subscribers[MOTOR_COUNT];
std_msgs__msg__Float32 motor_msg[MOTOR_COUNT];
rcl_publisher_t debug_publisher;
std_msgs__msg__String debug_msg;
char debug_buffer[100]; 

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Forward declarations
void handleStatusFrame(const CAN_message_t &msg);
void motor_callback(const void * msgin);
void sendSmartVelocity(uint8_t motorID, float rpm);
void sendHeartbeat();
void initCAN();

// Debug helper function
void publish_debug(const char* info) {
    snprintf(debug_msg.data.data, debug_msg.data.capacity, "%s", info);
    debug_msg.data.size = strlen(debug_msg.data.data);
    rcl_publish(&debug_publisher, &debug_msg, NULL);
}

// Callback function for motor subscriptions
void motor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motorSetpointRPM[i] = msg->data;
    sendSmartVelocity(MOTOR_IDS[i], msg->data);
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

  // RECALL LOGIC: Check if message was actually sent
  if (Can1.write(msg) != 1) {
    char err[50];
    snprintf(err, sizeof(err), "CAN Send Fail: Motor %d", motorID);
    publish_debug(err);
  } else {
    // Optional: publish_debug("CAN Send Success"); // High traffic if enabled
  }
}

// ---------------- STATUS HANDLING ----------------
void handleStatusFrame(const CAN_message_t &msg) {
  if (!msg.flags.extended || msg.len < 4) return;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if ((uint32_t)msg.id == (Status_1 + MOTOR_IDS[i])) {
      memcpy(&actualVelocityRPM[i], msg.buf, 4);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  set_microros_transports(); // Essential for Teensy <-> Agent comms
  
  delay(500);
  initCAN();

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_controller", "", &support));

  // Initialize Debug Publisher
  debug_msg.data.data = debug_buffer;
  debug_msg.data.capacity = sizeof(debug_buffer);
  debug_msg.data.size = 0;
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher, &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), 
    "/teensy/debug_out"));

  // Create subscribers
char topic_name[30];
for (int i = 0; i < MOTOR_COUNT; i++) {
  // Add an 'm' before the ID to ensure it starts with a letter
  snprintf(topic_name, sizeof(topic_name), "/motor/m%d/setpoint", MOTOR_IDS[i]);
  
  RCCHECK(rclc_subscription_init_default(
    &motor_subscribers[i],
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name));
}


  RCCHECK(rclc_executor_init(&executor, &support.context, MOTOR_COUNT, &allocator));

  for (int i = 0; i < MOTOR_COUNT; i++) {
    RCCHECK(rclc_executor_add_subscription(
      &executor, &motor_subscribers[i], &motor_msg[i],
      &motor_callback, ON_NEW_DATA));
  }
  
  publish_debug("Teensy node started");
}

// ---------------- LOOP ----------------
void loop() {
  uint32_t now = millis();

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  CAN_message_t rx_msg;
  while (Can1.read(rx_msg)) {
    handleStatusFrame(rx_msg);
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
