#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// Globale variabelen voor micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Macro om errors te checken
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Als er iets fout gaat, knippert de ingebouwde LED snel
void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setup() {
  // Zet de transport layer op Serial (USB)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  delay(2000); // Geef de Agent even tijd om te verbinden

  allocator = rcl_get_default_allocator();

  // 1. Initialiseer support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // 2. Maak een Node aan genaamd "teensy_node"
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // 3. Maak een Publisher aan op topic "teensy_text"
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "teensy_text"));

  // Initialiseer het bericht
  msg.data.data = (char * ) malloc(100 * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = 100;
}

void loop() {
  // Vul het bericht
  sprintf(msg.data.data, "Actuator status: OK - Tijd: %lu", millis());
  msg.data.size = strlen(msg.data.data);

  // Verstuur het bericht
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  // Even wachten
  delay(1000);
}