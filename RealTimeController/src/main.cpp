#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <FlexCAN_T4.h>
#include <DHT.h>
#include "MotorDriver.h"      
#include "TiltingActuator.h"  
#include "LiftingActuators.h" 
#include "LedManager.h"
#include "BMS.h"

//Temp sensor configuration
#define DHTPIN 14    
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDHTReadTime = 0;
float currentTemp = 0.0;

//BMS Configuration
uint8_t BMS_ADDR_LIST[2] = { 0x01, 0x02 };
BMS bms[2] = { BMS(BMS_ADDR_LIST[0]), BMS(BMS_ADDR_LIST[1]) };
float batteryVoltage = 0.0f;
const float batteryLowVoltageThreshold = 19.0; // Voltage threshold for low battery indication
const float batteryCriticalVoltageThreshold = 18.35; // Voltage threshold for critical battery indication
const float batteryDeadVoltageThreshold = 18.1; // Voltage threshold for dead battery indication
const float overheatTemperatureThreshold = 60.0; // Temperature threshold for overheating indication
unsigned long lastCanBusBMSRead = 0;
const unsigned long CAN_BUS_BMS_READ_INTERVAL = 200; //in ms
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

//LEDS configuration
LedManager leds;

elapsedMillis heartBeatTimer; //Timer for the CAN heartbeat
unsigned long lastMovementTime = 0; // Timestamp of the last detected movement (ms)

// Configuration Actuators
TiltingActuator tilt(28, 29, 12, 27); //In1, IN2, PWM, Potentiometer
LiftingActuators lift(5, 7, 6, 8, 4, 3, 26, 25);//IN1A, IN1B, IN2A, IN2B, ENA, ENB, PotA, PotB

// Driving Configuration
const float maxRPM = 5000.0f;  // Max RPM of the motors
const float deadzone = 0.05f;   // Joystick deadzone
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
MotorDriver motorRearRight(11); //CAN ID's
MotorDriver motorFrontRight(12);
MotorDriver motorFrontLeft(13);
MotorDriver motorRearLeft(14);
bool invertMotorLeft = false; //Invert right side motors
bool invertMotorRight = true;

//ROS Objects 
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
// Publishers & Subscribers
rcl_publisher_t debug_pub;
rcl_publisher_t battery_voltage_pub;
rcl_subscription_t sub_actuator; 
rcl_subscription_t sub_buttons;
rcl_subscription_t sub_standby;
// Timers
rcl_timer_t battery_voltage_timer;
// Messages
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__Int32 msg_buttons;
std_msgs__msg__String msg_debug;
std_msgs__msg__Float32 msg_battery_voltage;
rcl_subscription_t sub_system;
std_msgs__msg__Int32 msg_system;
std_msgs__msg__Bool msg_standby;
// Standby and Shutdown flags
bool standby_active = true;
bool shutdown_active = false;
// Buffer for debug string
char debugBuffer[255]; 
// Debug variables
float debug_joy_lift = 0.0;
int debug_last_preset = -1;

//Debug publish function
void PublishDebug(const char* text) {
  msg_debug.data.data = (char*)text;
  msg_debug.data.size = strlen(text);
  msg_debug.data.capacity = strlen(text) + 1;
  rcl_publish(&debug_pub, &msg_debug, NULL);
}

void CallbackSystem(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // Code 99 = SHUTDOWN
  if (msg->data == 99) {
      shutdown_active = true;
      leds.setState(Shutdown);
      leds.update();
      //Stop motors
      motorFrontLeft.setSpeed(0);
      motorRearLeft.setSpeed(0);
      motorFrontRight.setSpeed(0);
      motorRearRight.setSpeed(0);
      //Stop actuators
      lift.setManualSpeed(0);
      tilt.setManualSpeed(0);
  }
}
//Battery voltage timer callback
void BatteryVoltageTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }
  msg_battery_voltage.data = batteryVoltage;
  rcl_publish(&battery_voltage_pub, &msg_battery_voltage, NULL);
}

// Check if any actuator or motor is moving
bool isMoving() {
  // Checkt of er een actief setpoint is (ook tijdens optrekken/afremmen)
  return abs(motorFrontLeft.getCurrentSetpoint()) > 1.0 || 
         abs(motorRearLeft.getCurrentSetpoint()) > 1.0 ||
         abs(motorFrontRight.getCurrentSetpoint()) > 1.0 || 
         abs(motorRearRight.getCurrentSetpoint()) > 1.0 ||
         lift.getSpeedA() != 0 || lift.getSpeedB() != 0 || tilt.getLastSpeed() != 0;
}

void sendGlobalHeartbeat() {
    CAN_message_t msg;
    msg.id = 0x2052C80; // Heartbeat Base ID 
    msg.flags.extended = 1;
    msg.len = 8;
    memset(msg.buf, 0xFF, 8); 
    can3.write(msg);
}

//Callback joysticks
void CallbackActuator(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  debug_joy_lift = msg->angular.x;
  if (standby_active) {
      // If in standby, ignore actuator commands
      motorFrontLeft.setSpeed(0);
      motorRearLeft.setSpeed(0);
      motorFrontRight.setSpeed(0);
      motorRearRight.setSpeed(0);
      lift.setManualSpeed(0);
      tilt.setManualSpeed(0);
      return;
    } 
  
//Driving
  // Linear Y = Gas (Forward/Backward)
  // Angular Z = Steering (Left/Right)
  float throttle = msg->linear.y;
  float steering = msg->angular.z;

  // Deadzone filter
  if (abs(throttle) < deadzone) throttle = 0;
  if (abs(steering) < deadzone) steering = 0;

  // Mixing (Arcade Drive Formula)
  // Left = Throttle - Steering | Right = Throttle + Steering
  float leftOut = throttle - steering;
  float rightOut = throttle + steering;

  // Constrain to -1.0 to 1.0
  leftOut = constrain(leftOut, -1.0, 1.0);
  rightOut = constrain(rightOut, -1.0, 1.0);

  // Convert to RPM
  float rpmLeft = leftOut * maxRPM;
  float rpmRight = rightOut * maxRPM;

  // Invert
  if (invertMotorLeft) rpmLeft *= -1;
  if (invertMotorRight) rpmRight *= -1;

    motorFrontLeft.setSpeed(rpmLeft);
  motorRearLeft.setSpeed(rpmLeft);
  motorFrontRight.setSpeed(rpmRight);
  motorRearRight.setSpeed(rpmRight);

//Linear actuators
  //Only switch to Manual Mode if you MOVE the stick.
  // LIFT
  if (abs(msg->angular.x) > 0.1) {
      lift.setManualSpeed(msg->angular.x);
  } 
  // FIX 2: Check of hij beweegt EN of we in Manual Mode zitten (veiliger dan Target check)
  else if (lift.getSpeedA() != 0 && lift.isManualMode()) {
      lift.setManualSpeed(0);
  }
  
  // TILT
  if (abs(msg->angular.y) > 0.1) {
      tilt.setManualSpeed(msg->angular.y);
  } else if (tilt.getLastSpeed() != 0 && tilt.getTargetPositionRaw() == -1) {
      tilt.setManualSpeed(0);
  }
}

//Callback buttons
void CallbackButtons(const void * msgin) {
    if (standby_active) {
      // If in standby, ignore actuator commands
      motorFrontLeft.setSpeed(0);
      motorRearLeft.setSpeed(0);
      motorFrontRight.setSpeed(0);
      motorRearRight.setSpeed(0);
      lift.setManualSpeed(0);
      tilt.setManualSpeed(0);
      return;
    } 
  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  debug_last_preset = msg->data;
  // 0 = Cross [Enter positon name here]
  if (msg->data == 0) { 
      lift.setTargetPosition(15); //Distance in mm
      tilt.setTargetPosition(50);  //Distance in mm
      PublishDebug("Preset: MODE NAME");
  }
  // 1 = Round [Enter positon name here]
  else if (msg->data == 1) { 
      lift.setTargetPosition(30); //Distance in mm
      tilt.setTargetPosition(100); //Distance in mm
      PublishDebug("Preset: MODE NAME");
  }
  // 2 = Triangle (Dump Hoog)
  else if (msg->data == 2) { 
      lift.setTargetPosition(90);  //Distance in mm
      tilt.setTargetPosition(200); //Distance in mm
      PublishDebug("Preset: MODE NAME");
  }
}

// CAN Sniffer for all motor drivers
void CanSniff(const CAN_message_t &msg) {
    motorFrontLeft.parseCanMessage(msg);
    motorRearLeft.parseCanMessage(msg);
    motorFrontRight.parseCanMessage(msg);
    motorRearRight.parseCanMessage(msg);
}
//Callback standby
void CallbackStandby(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  standby_active = msg->data;
  if (standby_active) {
    leds.setState(Standby);
    // VEILIGHEID: Direct motoren en actuatoren killen
    motorFrontLeft.setSpeed(0);
    motorRearLeft.setSpeed(0);
    motorFrontRight.setSpeed(0);
    motorRearRight.setSpeed(0);
    lift.setManualSpeed(0);
    tilt.setManualSpeed(0);
  }  
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  can2.begin();
  can2.setBaudRate(1000000);
  bms[0].init(can2);
  bms[1].init(can2);

  leds.begin(23, 8); //led_PIN , Num_leds

  analogReadResolution(10); 
  
  can3.begin(); 
  can3.setBaudRate(1000000); 
  can3.onReceive(CanSniff);

  tilt.begin();
  lift.begin();

  set_microros_transports();
  dht.begin(); // Initialize DHT sensor
  unsigned long timeLedStart = millis();
  leds.setState(Startup);

  while((millis() - timeLedStart) < 3000){
    leds.update();
    delay(1);
  }
 //keep in mind safety delay of 2000 removed due to startup led show already taking up 3000
  allocator = rcl_get_default_allocator();

  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    leds.setState(Linux_boot_ERR);
    leds.update();
    delay(1);
  }
  
  rclc_node_init_default(&node, "teensy_loader_node", "", &support);

  //Publishers
  rclc_publisher_init_default(&debug_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/teensy_debug");
  rclc_publisher_init_default(&battery_voltage_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_voltage");
  
  //Subscribers
  rclc_subscription_init_default(&sub_actuator, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/actuator_pub");
  rclc_subscription_init_default(&sub_buttons, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/actuator_buttons");
    rclc_subscription_init_default(&sub_standby, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/standby");
    rclc_subscription_init_default(&sub_system, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/system_command");

  //Messages
  rclc_executor_init(&executor, &support.context, 6, &allocator); 
  rclc_executor_add_subscription(&executor, &sub_actuator, &msg_twist, &CallbackActuator, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_buttons, &msg_buttons, &CallbackButtons, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_standby, &msg_standby, &CallbackStandby, ON_NEW_DATA);

  rclc_timer_init_default(
    &battery_voltage_timer,
    &support,
    RCL_MS_TO_NS(60000),
    BatteryVoltageTimerCallback);
  rclc_executor_add_timer(&executor, &battery_voltage_timer);

  leds.setState(Standby);
  leds.update();
}

void loop() {
  if (shutdown_active) {
      leds.setState(Shutdown);
      leds.update();
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); 
      return; // jump out of loop
  }
  if (standby_active) {
     leds.setState(Standby);
  } else {
    if (isMoving()) {
      lastMovementTime = millis();
      leds.setState(Driving);
    } else if (millis() - lastMovementTime < 1000) {
      // Keep showing Driving for 1 second after motion stops
      leds.setState(Driving);
    } else {
      leds.setState(Operational);
    }
  }
  
    if (millis() - lastCanBusBMSRead >= CAN_BUS_BMS_READ_INTERVAL) {
        batteryVoltage = 0;
        for (int i = 0; i < 2; i++) {
            //Status1 s1;
            // if (!bms[i].readStatus1(s1)){
            //     // No response; skip this BMS this cycle
            //     continue;
            // }

            PackVI p;
            TempData t;
            
            bms[i].readPackVI(p);
            bms[i].readTempData(t);
            if (t.avgTemp_C >= overheatTemperatureThreshold) {
                leds.setState(Overheating);
            }

            batteryVoltage += p.totalVoltage_V / 2.0f; // Average voltage from both BMS units
        }

         if (batteryVoltage >= batteryCriticalVoltageThreshold && batteryVoltage <= batteryLowVoltageThreshold) {
             leds.setState(Low_power);
         } else if (batteryVoltage >= batteryDeadVoltageThreshold && batteryVoltage < batteryCriticalVoltageThreshold) {
             leds.setState(battery_empty);
         } else if (batteryVoltage < batteryDeadVoltageThreshold) {
             leds.setState(battery_dead);
         } 
          lastCanBusBMSRead = millis();
    }
  //FIXME: Re-enable moving LED state when initial led testing of setup is done
  // Update LED state based on movement
  
  leds.update();
  tilt.update();
  lift.update();
  
  // Motor updates
  motorFrontLeft.update(can3);
  motorRearLeft.update(can3);
  motorFrontRight.update(can3);
  motorRearRight.update(can3);

// Send global heartbeat every 20ms to all nodes
  if (heartBeatTimer > 20) {
      heartBeatTimer = 0;
      sendGlobalHeartbeat();
  }
  
  can3.events();  // ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  float accel = 4000.0f; 
  
  motorFrontLeft.setAcceleration(accel);
  motorRearLeft.setAcceleration(accel);
  motorFrontRight.setAcceleration(accel);
  motorRearRight.setAcceleration(accel);
  
  //Temperature sensor
  if (millis() - lastDHTReadTime > 2000) {
    float newTemp = dht.readTemperature();
    // Check of de lezing gelukt is (geen NaN)
    if (!isnan(newTemp)) {
      currentTemp = newTemp;
    }
    lastDHTReadTime = millis();
  }
  
  // Debug
  if (debugTimer > 100) {
    debugTimer = 0;
    const char* modeStr = lift.isManualMode() ? "MAN" : "AUT";
    
    sprintf(debugBuffer, "Mode:%s JOY:%.2f Button:%d | Lift: pos %dmm posA:%d targetA:%d, posB:%d TargetB:%d | Temp: %.1f C", 
            modeStr,                  // Modus (MAN/AUT)
            debug_joy_lift,           // Joystick input (-1.0 tot 1.0)
            debug_last_preset,        // Laatste knop (0, 1, 2)
            lift.getCurrentPosition(),
            lift.getPosA(), lift.getTargetA(),
            lift.getPosB(), lift.getTargetB(),
            currentTemp
            );
    PublishDebug(debugBuffer);
  }
}