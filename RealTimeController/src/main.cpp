#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <FlexCAN_T4.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>

// --- CAN SETUP ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

// Motor CAN Configuration
static const uint32_t DEVICE_ID = 1;
static const uint32_t CAN_BAUD = 1000000;
static const uint32_t HEARTBEAT_ID = 0x2052480;
static const uint32_t SPEED_SET_ID = 0x2050480;
static const uint32_t STATUS_1_ID = 0x2051840;

// Motor RPM limits
static const float MAX_RPM = 4000.0f;
static const float DEADZONE = 0.05f;

// Motor state
float motor_rpm = 0.0f;
elapsedMillis heartbeat_timer;
elapsedMillis motor_command_timer;

// --- MICRO-ROS OBJECTEN ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Publishers & Subscribers
rcl_publisher_t debug_pub; // NIEUW: Om tekst terug te sturen naar de PC
rcl_subscription_t sub_actuator;
rcl_subscription_t sub_lights;
rcl_subscription_t sub_standby;
rcl_subscription_t sub_drive_mode;

// Messages
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Bool msg_lights;
std_msgs__msg__Bool msg_standby;
std_msgs__msg__String msg_drive_mode;
std_msgs__msg__String msg_debug; // Het bericht dat we terugsturen

// Buffer voor de debug tekst
char debug_buffer[200]; 
// Buffer voor inkomende string (drive mode)
char drive_mode_buffer[50]; 

// Hulpfunctie om te publishen
void publish_debug(const char* text) {
  // Vul het message object
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;

  // Stuur naar PC
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

// --- CAN MOTOR FUNCTIONS ---

void send_heartbeat() {
  CAN_message_t msg;
  msg.id = HEARTBEAT_ID;
  msg.flags.extended = 1;
  msg.len = 8;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = 0xFF;
  }
  Can1.write(msg);
}

void send_motor_command(uint32_t device_id, float rpm) {
  CAN_message_t msg;
  msg.id = SPEED_SET_ID + device_id;
  msg.flags.extended = 1;
  msg.len = 8;
  
  // Pack float RPM into first 4 bytes (little-endian)
  memcpy(&msg.buf[0], &rpm, 4);
  
  // Remaining bytes zero
  for (int i = 4; i < 8; i++) {
    msg.buf[i] = 0;
  }
  
  Can1.write(msg);
}

void handle_status_frame(const CAN_message_t &msg) {
  // Status Frame 1 contains motor velocity as IEEE float
  uint32_t msg_id = msg.id & 0x1FFFFFFF;
  
  if (msg_id == (STATUS_1_ID + DEVICE_ID)) {
    float actual_rpm;
    memcpy(&actual_rpm, &msg.buf[0], 4);
    sprintf(debug_buffer, "MOTOR | Actual RPM: %.1f", actual_rpm);
    publish_debug(debug_buffer);
  }
}

// Apply deadzone to joystick input
float apply_deadzone(float value) {
  if (abs(value) < DEADZONE) {
    return 0.0;
  }
  return value;
}

// Convert joystick values (-1.0 to 1.0) to motor RPM
void joystick_to_rpm(float linear_y, float &motor_rpm_out) {
  // Apply deadzone
  linear_y = apply_deadzone(linear_y);
  
  // Clamp to -1.0 to 1.0 range
  linear_y = constrain(linear_y, -1.0, 1.0);
  
  // Convert to RPM
  motor_rpm_out = linear_y * MAX_RPM;
}



void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Convert joystick to RPM
  joystick_to_rpm(msg->linear.y, motor_rpm);
  
  // Format debug message with motor RPM
  sprintf(debug_buffer, "ACTUATOR | Gas: %.2f | Motor RPM: %.1f", 
          msg->linear.y, motor_rpm);
          
  publish_debug(debug_buffer);
}

void callback_lights(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  sprintf(debug_buffer, "LIGHTS | Status: %s", msg->data ? "AAN" : "UIT");
  publish_debug(debug_buffer);
}

void callback_standby(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  sprintf(debug_buffer, "STANDBY | Status: %s", msg->data ? "ACTIEF" : "INACTIEF");
  publish_debug(debug_buffer);
  
  // Stop motor in standby
  if (msg->data) {
    motor_rpm = 0.0f;
  }
}

void callback_mode(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  sprintf(debug_buffer, "MODE | Nieuwe modus: %s", msg->data.data);
  publish_debug(debug_buffer);
}

// --- SETUP & LOOP ---

void setup() {
  set_microros_transports(); // Serial (USB) wordt gebruikt voor ROS

  delay(2000);

  // Initialize CAN
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive([](const CAN_message_t &msg) {
    handle_status_frame(msg);
  });

  allocator = rcl_get_default_allocator();

  // Create Node
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_motor_node", "", &support);

  // 1. Maak de Debug Publisher (Hier luister je naar op je PC)
  rclc_publisher_init_default(
    &debug_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/teensy_debug"
  );

  // 2. Maak Subscribers (Zelfde topics als Python code)
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  rclc_subscription_init_default(&sub_lights, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lights_toggle");
  rclc_subscription_init_default(&sub_standby, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/standby");
  rclc_subscription_init_default(&sub_drive_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/drive_mode");

  // Geheugen reserveren voor inkomende strings
  msg_drive_mode.data.data = drive_mode_buffer;
  msg_drive_mode.data.capacity = sizeof(drive_mode_buffer);

  // Executor
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &callback_actuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_lights, &msg_lights, &callback_lights, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_standby, &msg_standby, &callback_standby, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_drive_mode, &msg_drive_mode, &callback_mode, ON_NEW_DATA);
  
  // Stuur een startberichtje
  publish_debug("Teensy Motor Node Started. Ready for motor commands...");
  
  heartbeat_timer = 0;
  motor_command_timer = 0;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  
  // Heartbeat every 20 ms
  if (heartbeat_timer >= 20) {
    heartbeat_timer = 0;
    send_heartbeat();
  }
  
  // Send motor commands every 50 ms
  if (motor_command_timer >= 50) {
    motor_command_timer = 0;
    send_motor_command(DEVICE_ID, motor_rpm);
  }
  
  delay(5);
}