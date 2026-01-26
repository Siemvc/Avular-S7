#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <FlexCAN_T4.h>

#include "MotorDriver.h"      
#include "TiltingActuator.h"  
#include "LiftingActuators.h" 

elapsedMillis debugTimer;

// --- CONFIGURATIE ---
TiltingActuator tilt(28, 29, 27); 
LiftingActuators lift(8, 7, 6, 5, 4, 3, 26, 25);

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
MotorDriver motorFrontLeft(3);
MotorDriver motorRearLeft(4);
MotorDriver motorFrontRight(2);
MotorDriver motorRearRight(1);

// --- ROS Objects ---
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t debug_pub;
rcl_subscription_t sub_actuator; 
rcl_subscription_t sub_buttons;     // <--- Button Subscriber
rcl_subscription_t sub_drive_mode;  

geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32 msg_buttons;   // <--- Button Message
std_msgs__msg__String msg_debug;
char debug_buffer[200];

// --- HULPFUNCTIE ---
void publish_debug(const char* text) {
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

// --- CALLBACK: Joystick (Handmatig) ---
void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // 1. RIJDEN (Linker Stick) - Tank Drive
  // ... (Hier jouw bestaande motor-code laten staan) ...

  // 2. ACTUATORS (Rechter Stick)
  // Angular X = Lift, Angular Y = Tilt
  
  // CRUCIAAL: Alleen omschakelen naar Manual Mode als je de stick BEWEEGT.
  // Als de stick in het midden staat (0.0), doen we niks, 
  // zodat de PID (Preset) zijn werk kan afmaken.
  
  if (abs(msg->angular.x) > 0.1) {
      lift.setManualSpeed(msg->angular.x);
  } else if (lift.getLastSpeed() != 0 && lift.getTargetPositionRaw() == -1) {
      // Als we in manual mode waren (geen target), en we laten stick los -> STOP
      lift.setManualSpeed(0);
  }

  if (abs(msg->angular.y) > 0.1) {
      tilt.setManualSpeed(msg->angular.y);
  } else if (tilt.getLastSpeed() != 0 && tilt.getTargetPositionRaw() == -1) {
      // Als we in manual mode waren (geen target), en we laten stick los -> STOP
      tilt.setManualSpeed(0);
  }
}

// --- CALLBACK: Knoppen (Presets) ---
void callback_buttons(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // 0 = Kruisje (Laag/Scrape)
  if (msg->data == 0) { 
      lift.setTargetPosition(15); 
      tilt.setTargetPosition(5);  
      publish_debug("Preset: LOW");
  }
  // 1 = Rondje (Rijstand)
  else if (msg->data == 1) { 
      lift.setTargetPosition(30); 
      tilt.setTargetPosition(80); 
      publish_debug("Preset: DRIVE");
  }
  // 2 = Driehoekje (Dump Hoog)
  else if (msg->data == 2) { 
      lift.setTargetPosition(90); 
      tilt.setTargetPosition(100); 
      publish_debug("Preset: DUMP");
  }
}


void canSniff(const CAN_message_t &msg) {
    // Hier kun je evt. motor status verwerken
}

void setup() {
  analogReadResolution(10); 

  Can3.begin(); Can3.setBaudRate(1000000); 
  Can3.onReceive(canSniff);

  tilt.begin();
  lift.begin();

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_loader_node", "", &support);

  // Publishers
  rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/teensy_debug");
  
  // Subscribers
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  
  // NIEUW: Buttons Subscriber
  rclc_subscription_init_default(&sub_buttons, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/actuator_buttons");

  rclc_executor_init(&executor, &support.context, 3, &allocator); // Aantal handles verhoogd naar 3
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &callback_actuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_buttons, &msg_buttons, &callback_buttons, ON_NEW_DATA);
}

void loop() {
  // Actuator updates (PID en Soft Limits)
  tilt.update();
  lift.update();
  
  // Motor updates
  motorFrontLeft.update(Can3);
  motorRearLeft.update(Can3);
  motorFrontRight.update(Can3);
  motorRearRight.update(Can3);
  
  Can3.events();

  // ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Debug (Elke 200ms)
  if (debugTimer > 200) {
    debugTimer = 0;
    
    // Simpele debug string
    sprintf(debug_buffer, "L:%d%%/%d T:%d%%/%d", 
            lift.getCurrentPosition(), lift.getTargetPositionRaw(),
            tilt.getCurrentPosition(), tilt.getTargetPositionRaw());
    publish_debug(debug_buffer);
  }
}