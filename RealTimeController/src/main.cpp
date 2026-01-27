#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include <cstring>

// ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

// ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// BMS #1 Publishers
rcl_publisher_t bms1_voltage_pub;
rcl_publisher_t bms1_current_pub;
rcl_publisher_t bms1_soc_pub;
rcl_publisher_t bms1_temp_pub;

// BMS #2 Publishers
rcl_publisher_t bms2_voltage_pub;
rcl_publisher_t bms2_current_pub;
rcl_publisher_t bms2_soc_pub;
rcl_publisher_t bms2_temp_pub;

// Message
std_msgs__msg__Float32 msg_float32;

// Timing
unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL = 1000;  // 1000ms

// Using CAN3 (pins 30 = TX, 31 = RX)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

// --- TWO BMS addresses ---
uint8_t BMS_ADDR_LIST[2] = { 0x01, 0x02 };

static uint8_t PC_ADDR = 0x40;

enum DalyDataId : uint8_t {
  DID_PACK_SOC_V_I  = 0x90,
  DID_STATUS1       = 0x94,
  DID_TEMP_MOD      = 0x92
};

struct PackVI {
  float cumulativeVoltage_V = NAN;
  float totalVoltage_V      = NAN;
  float current_A           = NAN;
  float soc_pct             = NAN;
};

struct Status1 {
  uint8_t nStrings=0, nTemps=0, chargerConnected=0, loadConnected=0, ioBits=0;
};

struct TempData {
  float maxTemp_C = NAN;
  float minTemp_C = NAN;
  float avgTemp_C = NAN;
};

static inline uint16_t be16(const uint8_t* p){
  return (uint16_t(p[0]) << 8) | p[1];
}

static inline uint32_t makeCanId(uint8_t DID, uint8_t DST, uint8_t SRC){
  return ((uint32_t)0x18 << 24) | ((uint32_t)DID << 16) | ((uint32_t)DST << 8) | SRC;
}

bool canInit(){
  Can0.begin();
  Can0.setBaudRate(250000);
  return true;
}

bool txRequest(uint8_t DID, uint8_t BMS_ADDR){
  CAN_message_t tx = {};
  tx.id = makeCanId(DID, BMS_ADDR, PC_ADDR);
  tx.flags.extended = 1;
  tx.len = 8;
  memset(tx.buf, 0, 8);
  return Can0.write(tx);
}

bool rxResponse(uint8_t DID, uint8_t BMS_ADDR, CAN_message_t &rx, uint32_t timeout_ms=120){
  uint32_t expect = makeCanId(DID, PC_ADDR, BMS_ADDR);
  elapsedMillis t = 0;

  while (t < timeout_ms){
    if (Can0.read(rx)){
      if (rx.id == expect && rx.flags.extended && rx.len == 8)
        return true;
    }
  }
  return false;
}

bool requestAndWait(uint8_t DID, uint8_t BMS_ADDR, CAN_message_t &rx){
  if (!txRequest(DID, BMS_ADDR)) return false;
  return rxResponse(DID, BMS_ADDR, rx);
}

// ----- Parsing -----

bool readPackVI(uint8_t BMS_ADDR, PackVI &o){
  CAN_message_t rx;
  if (!requestAndWait(DID_PACK_SOC_V_I, BMS_ADDR, rx)) return false;

  float v_cum = be16(&rx.buf[0]) * 0.1f;
  float v_gat = be16(&rx.buf[2]) * 0.1f;
  float i_raw = (int32_t(be16(&rx.buf[4])) - 30000) * 0.1f;
  float s_raw = be16(&rx.buf[6]) * 0.1f;

  o.cumulativeVoltage_V = v_cum;
  o.totalVoltage_V = v_gat;
  o.current_A = i_raw;
  o.soc_pct = s_raw;

  return true;
}

bool readStatus1(uint8_t BMS_ADDR, Status1 &o){
  CAN_message_t rx;
  if (!requestAndWait(DID_STATUS1, BMS_ADDR, rx)) return false;

  o.nStrings = rx.buf[0];
  o.nTemps   = rx.buf[1];
  o.chargerConnected = rx.buf[2];
  o.loadConnected    = rx.buf[3];
  o.ioBits = rx.buf[4];
  return true;
}

bool readTempData(uint8_t BMS_ADDR, TempData &o){
  CAN_message_t rx;
  if (!requestAndWait(DID_TEMP_MOD, BMS_ADDR, rx)) return false;

  float maxTemp = (int8_t)rx.buf[0];
  float minTemp = (int8_t)rx.buf[1];
  float avgTemp = (int8_t)rx.buf[2];

  o.maxTemp_C = maxTemp;
  o.minTemp_C = minTemp;
  o.avgTemp_C = avgTemp;

  return true;
}

// ---- MAIN ----

void setup(){
  if (!canInit()){
    while(1);
  }

  // Initialize ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "BMS_node", "", &support);

  // BMS #1 Publishers
  rclc_publisher_init_default(&bms1_voltage_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms1/voltage");
  rclc_publisher_init_default(&bms1_current_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms1/current");
  rclc_publisher_init_default(&bms1_soc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms1/soc");
  rclc_publisher_init_default(&bms1_temp_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms1/temp");

  // BMS #2 Publishers
  rclc_publisher_init_default(&bms2_voltage_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms2/voltage");
  rclc_publisher_init_default(&bms2_current_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms2/current");
  rclc_publisher_init_default(&bms2_soc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms2/soc");
  rclc_publisher_init_default(&bms2_temp_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/bms2/temp");

  lastPublishTime = millis();
}

void loop(){
  unsigned long currentTime = millis();

  if (currentTime - lastPublishTime >= PUBLISH_INTERVAL){
    lastPublishTime = currentTime;

    for (int i = 0; i < 2; i++){
      uint8_t BMS = BMS_ADDR_LIST[i];

      Status1 s1;
      if (!readStatus1(BMS, s1)){
        continue;
      }

      PackVI p;
      TempData t;
      if (readPackVI(BMS, p) && readTempData(BMS, t)){
        if (i == 0){
          msg_float32.data = p.totalVoltage_V;
          rcl_publish(&bms1_voltage_pub, &msg_float32, NULL);
          msg_float32.data = p.current_A;
          rcl_publish(&bms1_current_pub, &msg_float32, NULL);
          msg_float32.data = p.soc_pct;
          rcl_publish(&bms1_soc_pub, &msg_float32, NULL);
          msg_float32.data = t.avgTemp_C;
          rcl_publish(&bms1_temp_pub, &msg_float32, NULL);
        } else {
          msg_float32.data = p.totalVoltage_V;
          rcl_publish(&bms2_voltage_pub, &msg_float32, NULL);
          msg_float32.data = p.current_A;
          rcl_publish(&bms2_current_pub, &msg_float32, NULL);
          msg_float32.data = p.soc_pct;
          rcl_publish(&bms2_soc_pub, &msg_float32, NULL);
          msg_float32.data = t.avgTemp_C;
          rcl_publish(&bms2_temp_pub, &msg_float32, NULL);
        }
      }
    }
  }
}