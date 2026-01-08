#include <Arduino.h>
#include <micro_ros_platformio.h>
// micro-ROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//Message types
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h> // Nieuwe message type voor de LED

// Definities
rcl_publisher_t publisher;
rcl_subscription_t subscriber; // Nieuwe subscriber
std_msgs__msg__Int32 msg_pub;
std_msgs__msg__Bool msg_sub;   // Bericht om te ontvangen

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    // Snel knipperen = error
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Functie die wordt aangeroepen als we data ontvangen
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  // Zet de LED aan als data true is, uit als false
  digitalWrite(LED_BUILTIN, (msg->data ? HIGH : LOW));
}

// Functie om data te sturen (elke seconde)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
    msg_pub.data++;
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Begin met LED uit

  delay(2000);

  allocator = rcl_get_default_allocator();

  // 1. Init micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // 2. Maak Node
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // 3. Maak Publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_teensy_count"));

  // 4. Maak Subscriber (NIEUW)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "teensy_led"));

  // 5. Maak Timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback));

  // 6. Executor Setup (Let op: handles verhoogd naar 2 omdat we nu pub én sub hebben)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Voeg subscriber toe aan executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  msg_pub.data = 0;
}

void loop() {
  // Checkt nu zowel de timer (verzenden) als inkomende data (ontvangen)
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}