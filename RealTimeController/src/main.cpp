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
elapsedMillis heartbeatTimer; // Timer voor de globale CAN heartbeat

// Pin Configuratie Actuators
TiltingActuator tilt(28, 29, 27); 
LiftingActuators lift(8, 7, 6, 5, 4, 3, 26, 25);

// Driving Configuration
#define MAX_RPM 4000.0f  // Max snelheid motoren
#define DEADZONE 0.05f   // Joystick deadzone

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
MotorDriver motorFrontLeft(3);
MotorDriver motorRearLeft(4);
MotorDriver motorFrontRight(2);
MotorDriver motorRearRight(1);

// Configuratie draairichting (Pas aan als ze verkeerd om draaien)
bool INVERT_LEFT = false;
bool INVERT_RIGHT = true;

//ROS Objects 
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
// Publishers & Subscribers
rcl_publisher_t debug_pub;
rcl_subscription_t sub_actuator; 
rcl_subscription_t sub_buttons;

// Messages
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32 msg_buttons;
std_msgs__msg__String msg_debug;
char debug_buffer[255];

//Debug publish function
void publish_debug(const char* text) {
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

void sendGlobalHeartbeat() {
    CAN_message_t msg;
    msg.id = 0x2052C80; // Heartbeat Base ID uit testcode
    msg.flags.extended = 1;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8); // Vullen met FF
    Can3.write(msg);
}

// CALLBACK: Joystick
void callback_actuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
// ____________Driving (Tank/Arcade Drive)____________
  // Linear Y = Gas (Vooruit/Achteruit)
  // Angular Z = Stuur (Links/Rechts)
   
  float throttle = msg->linear.y;
  float steering = msg->angular.z;

  // Deadzone filter
  if (abs(throttle) < DEADZONE) throttle = 0;
  if (abs(steering) < DEADZONE) steering = 0;

  // Mixen (Arcade Drive Formule)
  // Links = Gas - Stuur | Rechts = Gas + Stuur
  float leftOut = throttle - steering;
  float rightOut = throttle + steering;

  // Begrenzen op -1.0 tot 1.0
  leftOut = constrain(leftOut, -1.0, 1.0);
  rightOut = constrain(rightOut, -1.0, 1.0);

  // Omrekenen naar RPM
  float rpmLeft = leftOut * MAX_RPM;
  float rpmRight = rightOut * MAX_RPM;

  // Inverteren indien nodig
  if (INVERT_LEFT) rpmLeft *= -1;
  if (INVERT_RIGHT) rpmRight *= -1;

  // Stuur naar motoren
  motorFrontLeft.setSpeed(rpmLeft);
  motorRearLeft.setSpeed(rpmLeft);
  motorFrontRight.setSpeed(rpmRight);
  motorRearRight.setSpeed(rpmRight);


  //____________Actuators____________
  //Only switch to Manual Mode if you MOVE the stick.
  //Lifting Actuator
  if (abs(msg->angular.x) > 0.1) {
      lift.setManualSpeed(msg->angular.x);
  } else if (lift.getLastSpeed() != 0 && lift.getTargetPositionRaw() == -1) {
      //If we where in manual mode (no target), and we release the stick -> STOP
      lift.setManualSpeed(0);
  }
  //Tilting Actuator
  if (abs(msg->angular.y) > 0.1) {
      tilt.setManualSpeed(msg->angular.y);
  } else if (tilt.getLastSpeed() != 0 && tilt.getTargetPositionRaw() == -1) {
      //If we where in manual mode (no target), and we release the stick -> STOP
      tilt.setManualSpeed(0);
  }
}

//____________CALLBACK: Button Presets____________
void callback_buttons(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // 0 = Cross [Enter positon name here]
  if (msg->data == 0) { 
      lift.setTargetPosition(15); //Distance in mm
      tilt.setTargetPosition(5);  //Distance in mm
      publish_debug("Preset: MODE NAME");
  }
  // 1 = Round [Enter positon name here]
  else if (msg->data == 1) { 
      lift.setTargetPosition(30); //Distance in mm
      tilt.setTargetPosition(80); //Distance in mm
      publish_debug("Preset: MODE NAME");
  }
  // 2 = Driehoekje (Dump Hoog)
  else if (msg->data == 2) { 
      lift.setTargetPosition(90);  //Distance in mm
      tilt.setTargetPosition(100); //Distance in mm
      publish_debug("Preset: MODE NAME");
  }
}


void canSniff(const CAN_message_t &msg) {
    motorFrontLeft.parseCanMessage(msg);
    motorRearLeft.parseCanMessage(msg);
    motorFrontRight.parseCanMessage(msg);
    motorRearRight.parseCanMessage(msg);
}

void setup() {
  analogReadResolution(10); 

  Can3.begin(); 
  Can3.setBaudRate(1000000); 
  Can3.onReceive(canSniff);

  tilt.begin();
  lift.begin();

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_loader_node", "", &support);

  //Publishers
  rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/teensy_debug");
  
  //Subscribers
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  
  //Buttons Subscriber
  rclc_subscription_init_default(&sub_buttons, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/actuator_buttons");
  //Messages
  rclc_executor_init(&executor, &support.context, 3, &allocator); 
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &callback_actuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_buttons, &msg_buttons, &callback_buttons, ON_NEW_DATA);
}

void loop() {
  // Actuator updates
  tilt.update();
  lift.update();
  
  // Motor updates
  motorFrontLeft.update(Can3);
  motorRearLeft.update(Can3);
  motorFrontRight.update(Can3);
  motorRearRight.update(Can3);


  if (heartbeatTimer > 20) {
      heartbeatTimer = 0;
      sendGlobalHeartbeat();
  }
  
  Can3.events();

  // ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Debug
  if (debugTimer > 100) {
    debugTimer = 0;
    
    //Simpele debug string
    sprintf(debug_buffer, "LIFT:%d TILT:%d | L: T%.0f/A%.0f | R: T%.0f/A%.0f", 
            lift.getCurrentPosition(), 
            tilt.getCurrentPosition(),
            motorFrontLeft.getTargetRPM(),
            motorFrontLeft.getActualRPM(),
            motorFrontRight.getTargetRPM(),
            motorFrontRight.getActualRPM()
            );
    publish_debug(debug_buffer);
  }
}