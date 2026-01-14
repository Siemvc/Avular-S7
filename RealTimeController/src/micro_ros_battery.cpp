
#include "micro_ros_battery.h"

// --- micro-ROS PlatformIO includes ---
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// --- Messages ---
#include <sensor_msgs/msg/battery_state.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <rosidl_runtime_c/string_functions.h>

// --- micro-ROS handles ---
static rcl_allocator_t g_allocator;
static rclc_support_t  g_support;
static rcl_node_t      g_node;
static rclc_executor_t g_exec;

// Publishers for two batteries
static rcl_publisher_t pub_batt[2];
static rcl_publisher_t pub_cells[2];
static rcl_publisher_t pub_temps[2];

// Messages
static sensor_msgs__msg__BatteryState   batt_msg[2];
static std_msgs__msg__Float32MultiArray cell_msg[2];
static std_msgs__msg__Int8MultiArray    temp_msg[2];

// Static buffers
static float  cell_buf[2][48];
static int8_t temp_buf[2][21];

static uint8_t compute_status(float current_A, uint8_t chgMOS, uint8_t dsgMOS, uint8_t state) {
  const float eps = 0.2f;
  if (current_A >  eps) return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
  if (current_A < -eps) return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
  if (state == 1 || chgMOS) return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
  if (state == 2 || dsgMOS) return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
  return sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
}

void uros_init(uint32_t baud) {
  Serial.begin(baud);
  // --- Serial transport for micro_ros_platformio (Teensy 4.1 supported) ---
  set_microros_serial_transports(Serial);
  // If you want native Ethernet later:
  // byte mac[] = {0xAA,0xBB,0xCC,0xEE,0xDD,0xFF};
  // IPAddress local_ip(192,168,1,177), agent_ip(192,168,1,113); size_t agent_port=8888;
  // set_microros_native_ethernet_transports(mac, local_ip, agent_ip, agent_port);
  // ...and set board_microros_transport = native_ethernet in platformio.ini
  // (Transport options & supported boards are in the repo README.) // (ref)

  g_allocator = rcl_get_default_allocator();
  rclc_support_init(&g_support, 0, NULL, &g_allocator);

  rclc_node_init_default(&g_node, "teensy_dual_bms_node", "", &g_support);

  const char* batt_topics[2]  = {"battery_state_1", "battery_state_2"};
  const char* cells_topics[2] = {"cell_voltages_1", "cell_voltages_2"};
  const char* temps_topics[2] = {"bms_temps_1", "bms_temps_2"};

  for (int i = 0; i < 2; i++) {
    rclc_publisher_init_default(
      &pub_batt[i], &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      batt_topics[i]);

    rclc_publisher_init_default(
      &pub_cells[i], &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      cells_topics[i]);

    rclc_publisher_init_default(
      &pub_temps[i], &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8MultiArray),
      temps_topics[i]);

    sensor_msgs__msg__BatteryState__init(&batt_msg[i]);
    rosidl_runtime_c__String__init(&batt_msg[i].location);
    rosidl_runtime_c__String__assign(&batt_msg[i].location, "pack");
    rosidl_runtime_c__String__init(&batt_msg[i].serial_number);
    rosidl_runtime_c__String__assign(&batt_msg[i].serial_number, "");

    cell_msg[i].data.data     = cell_buf[i];
    cell_msg[i].data.size     = 0;
    cell_msg[i].data.capacity = 48;

    temp_msg[i].data.data     = temp_buf[i];
    temp_msg[i].data.size     = 0;
    temp_msg[i].data.capacity = 21;
  }

  rclc_executor_init(&g_exec, &g_support.context, 1, &g_allocator);
}

void uros_spin_once(uint32_t timeout_ms) {
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(timeout_ms));
}

void uros_publish_battery(
  uint8_t idx,
  float v_total,
  float current_A,
  float soc_pct,
  uint8_t n_cells,
  const std::vector<uint16_t>& cell_mV,
  const std::vector<int8_t>&  temps_C,
  uint8_t chgMOS, uint8_t dsgMOS, uint8_t state
) {
  if (idx < 1 || idx > 2) return;
  idx--; // 0-based

  batt_msg[idx].voltage    = v_total;
  batt_msg[idx].current    = current_A;
  batt_msg[idx].percentage = isnan(soc_pct) ? NAN : (soc_pct / 100.0f);
  batt_msg[idx].power_supply_status = compute_status(current_A, chgMOS, dsgMOS, state);
  batt_msg[idx].present    = true;
  batt_msg[idx].temperature = NAN;
  (void) rcl_publish(&pub_batt[idx], &batt_msg[idx], NULL);

  if (n_cells > 0 && !cell_mV.empty()) {
    uint8_t n = (n_cells > 48) ? 48 : n_cells;
    cell_msg[idx].data.size = n;
    for (uint8_t i = 0; i < n; ++i) cell_buf[idx][i] = cell_mV[i] / 1000.0f; // mV -> V
    (void) rcl_publish(&pub_cells[idx], &cell_msg[idx], NULL);
  }

  if (!temps_C.empty()) {
    uint8_t n = (temps_C.size() > 21) ? 21 : (uint8_t)temps_C.size();
    temp_msg[idx].data.size = n;
    for (uint8_t i = 0; i < n; ++i) temp_buf[idx][i] = temps_C[i];
    (void) rcl_publish(&pub_temps[idx], &temp_msg[idx], NULL);
  }
}
