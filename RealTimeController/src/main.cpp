#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>

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

// --- CALLBACKS (Alleen printen naar ROS topic) ---

void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Format een string met de waardes
  sprintf(debug_buffer, "ACTUATOR | LinY (Gas): %.2f | AngZ (Stuur): %.2f | AngX (Lift): %.2f | AngY (Tilt): %.2f", 
          msg->linear.y, msg->angular.z, msg->angular.x, msg->angular.y);
          
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

  allocator = rcl_get_default_allocator();

  // Create Node
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_debug_node", "", &support);

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
  publish_debug("Teensy Debugger Started. Waiting for data...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // Geen motor updates nodig nu
}