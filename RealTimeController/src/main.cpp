#include <Arduino.h>
#include <micro_ros_arduino.h> // of micro_ros_platformio als je die gebruikt
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <FlexCAN_T4.h>

#include "MotorDriver.h"

// --- 1. GLOBAL CAN OBJECT (Centraal) ---
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// --- 2. CONFIGURATIE ---
#define MAX_RPM 500.0f
#define DEADZONE 0.05f

// Motoren (Let op: CAN IDs checken!)
MotorDriver motorFrontLeft(3);
MotorDriver motorRearLeft(4);
MotorDriver motorFrontRight(2);
MotorDriver motorRearRight(1);

bool INVERT_LEFT_SIDE  = false;
bool INVERT_RIGHT_SIDE = true; 

// --- 3. CAN CALLBACK (Centraal) ---
// Deze functie ontvangt ALLE berichten en stuurt ze door naar de juiste motor class
void canSniff(const CAN_message_t &msg) {
  motorFrontLeft.processCanMessage(msg);
  motorRearLeft.processCanMessage(msg);
  motorFrontRight.processCanMessage(msg);
  motorRearRight.processCanMessage(msg);
}

// --- ROS GLOBALEN ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t debug_pub;
rcl_subscription_t sub_actuator;
rcl_subscription_t sub_standby;

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Bool msg_standby;
std_msgs__msg__String msg_debug;
char debug_buffer[200]; 

elapsedMillis last_ros_msg_time;
bool isStandby = false;

// --- HULPFUNCTIES ---

void publish_debug(const char* text) {
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

// --- CALLBACKS ---

void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_ros_msg_time = 0; 

  if (isStandby) return;

  float throttle = msg->linear.y; 
  float steering = msg->angular.z; 

  if (abs(throttle) < DEADZONE) throttle = 0;
  if (abs(steering) < DEADZONE) steering = 0;

  float left_out  = throttle - steering;
  float right_out = throttle + steering;

  left_out  = constrain(left_out, -1.0, 1.0);
  right_out = constrain(right_out, -1.0, 1.0);

  float rpmLeft  = left_out * MAX_RPM;
  float rpmRight = right_out * MAX_RPM;

  if (INVERT_LEFT_SIDE)  rpmLeft  *= -1;
  if (INVERT_RIGHT_SIDE) rpmRight *= -1;

  motorFrontLeft.setSpeed(rpmLeft);
  motorRearLeft.setSpeed(rpmLeft);
  motorFrontRight.setSpeed(rpmRight);
  motorRearRight.setSpeed(rpmRight);

  sprintf(debug_buffer, "RPM: L=%.0f R=%.0f", rpmLeft, rpmRight);
  publish_debug(debug_buffer);
}

void callback_standby(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  isStandby = msg->data;
  if (isStandby) {
    motorFrontLeft.stop();
    motorRearLeft.stop();
    motorFrontRight.stop();
    motorRearRight.stop();
  }
}

// --- SETUP & LOOP ---

void setup() {
  // 1. CAN Starten
  Can3.begin();
  Can3.setBaudRate(1000000);
  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff); // Koppel de globale functie

  // 2. ROS Starten
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_drive_node", "", &support);

  rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/teensy_debug");
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  rclc_subscription_init_default(&sub_standby, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/standby");

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &callback_actuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_standby, &msg_standby, &callback_standby, ON_NEW_DATA);
  
  publish_debug("4-Motor System Ready on CAN3");
}

void loop() {
  // Update Motoren (Geef het Can3 object mee!)
  motorFrontLeft.update(Can3);
  motorRearLeft.update(Can3);
  motorFrontRight.update(Can3);
  motorRearRight.update(Can3);
  
  // Verwerk inkomende CAN berichten (belangrijk!)
  Can3.events();

  // ROS Update
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Watchdog
  if (last_ros_msg_time > 500) {
    motorFrontLeft.stop();
    motorRearLeft.stop();
    motorFrontRight.stop();
    motorRearRight.stop();
  }
}