#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <DHT.h> // De sensor library

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// We gebruiken Float32 voor decimalen
#include <std_msgs/msg/float32.h>

// Sensor instellingen
const int DHTPIN = 16;
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Micro-ROS variabelen
rcl_publisher_t temp_publisher;
rcl_publisher_t hum_publisher;
std_msgs__msg__Float32 temp_msg;
std_msgs__msg__Float32 hum_msg;
// Executor en support structuren
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error check macro's
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Deze functie wordt elke seconde aangeroepen
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // 1. Lees de sensor
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check of het lezen gelukt is (isnan = is not a number)
    if (isnan(humidity) || isnan(temperature )) {
        // Als het faalt, doen we even niks (of je kunt een error loggen)
        return;
    }

    // 2. Vul de berichten
    temp_msg.data = temperature;
    hum_msg.data = humidity;

    // 3. Publiceer de data
    RCSOFTCHECK(rcl_publish(&temp_publisher, &temp_msg, NULL));
    RCSOFTCHECK(rcl_publish(&hum_publisher, &hum_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Start de sensor
  dht.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Init micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_sensor_node", "", &support));

  // Maak Publisher voor Temperatuur
  RCCHECK(rclc_publisher_init_default(
    &temp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sensor/temperature"));

  // Maak Publisher voor Luchtvochtigheid
  RCCHECK(rclc_publisher_init_default(
    &hum_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sensor/humidity"));

  // Timer: 1 seconde), dit is de sample rate van de sensor (ja hij is echt zo traag)
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback));

  // Executor (1 handle is genoeg voor alleen de timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}