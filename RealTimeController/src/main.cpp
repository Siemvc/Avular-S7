#include <Arduino.h>
#include <micro_ros_arduino.h> // <--- Terug naar de Arduino versie
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
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

// --- MOTOR CONFIGURATIE (VESC) ---
static const uint32_t DEVICE_ID = 1;
static const uint32_t CAN_BAUD = 1000000; 
static const uint32_t CMD_SET_RPM = 3;  

// Motor Limieten
static const float MAX_RPM = 4000.0f; 
static const float DEADZONE = 0.05f; 

// Variabelen
float target_rpm = 0.0f;

elapsedMillis heartbeat_timer;
elapsedMillis motor_command_timer;
elapsedMillis last_ros_msg_timer;
const unsigned long WATCHDOG_TIMEOUT = 500; 

// --- MICRO-ROS ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t debug_pub;
rcl_subscription_t sub_actuator;
rcl_subscription_t sub_lights;
rcl_subscription_t sub_standby;
rcl_subscription_t sub_drive_mode;

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Bool msg_lights;
std_msgs__msg__Bool msg_standby;
std_msgs__msg__String msg_drive_mode;
std_msgs__msg__String msg_debug;

char debug_buffer[200]; 
char drive_mode_buffer[50]; 

// --- FUNCTIES ---

void publish_debug(const char* text) {
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

void send_vesc_rpm(uint32_t controller_id, float rpm) {
  CAN_message_t msg;
  msg.id = (uint32_t)(CMD_SET_RPM << 8) | controller_id;
  msg.flags.extended = 1;
  msg.len = 4;

  int32_t send_rpm = (int32_t)rpm;
  msg.buf[0] = (send_rpm >> 24) & 0xFF;
  msg.buf[1] = (send_rpm >> 16) & 0xFF;
  msg.buf[2] = (send_rpm >> 8)  & 0xFF;
  msg.buf[3] = (send_rpm >> 0)  & 0xFF;

  Can1.write(msg);
}

// --- CALLBACKS ---

void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_ros_msg_timer = 0; 

  float throttle = constrain(msg->linear.y, -1.0, 1.0);
  if (abs(throttle) < DEADZONE) throttle = 0.0;
  target_rpm = throttle * MAX_RPM;

  sprintf(debug_buffer, "DRIVE | Input: %.2f | RPM Target: %.0f", throttle, target_rpm);
  publish_debug(debug_buffer);
}

void callback_lights(const void * msgin) { }

void callback_standby(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    target_rpm = 0;
    publish_debug("STANDBY ACTIVE - MOTOR STOP");
  }
}

void callback_mode(const void * msgin) { }

// --- SETUP & LOOP ---

void setup() {
  // AANGEPAST: Terug naar de standaard functie
  set_microros_transports(); 
  
  delay(2000);

  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_single_motor_node", "", &support);

  rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/teensy_debug");
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  rclc_subscription_init_default(&sub_lights, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/lights_toggle");
  rclc_subscription_init_default(&sub_standby, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/standby");
  rclc_subscription_init_default(&sub_drive_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/drive_mode");

  msg_drive_mode.data.data = drive_mode_buffer;
  msg_drive_mode.data.capacity = sizeof(drive_mode_buffer);

  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &callback_actuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_lights, &msg_lights, &callback_lights, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_standby, &msg_standby, &callback_standby, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_drive_mode, &msg_drive_mode, &callback_mode, ON_NEW_DATA);

  publish_debug("Teensy Single Motor Ready");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  if (last_ros_msg_timer > WATCHDOG_TIMEOUT) {
    if (target_rpm != 0) target_rpm = 0;
  }

  if (motor_command_timer >= 20) {
    motor_command_timer = 0;
    send_vesc_rpm(DEVICE_ID, target_rpm);
  }
}