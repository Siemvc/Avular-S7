
#pragma once
#include <Arduino.h>
#include <vector>

// Initialize micro-ROS node and publishers for two batteries
// baud is optional for USB CDC but kept for consistency
void uros_init(uint32_t baud = 115200);

// Spin micro-ROS executor (non-blocking)
// Call this in loop() to keep micro-ROS alive
void uros_spin_once(uint32_t timeout_ms = 0);

// Publish battery info for battery #1 or #2
// battery_index: 1 or 2
// v_total: Pack voltage [V]
// current_A: Pack current [A] (+ = discharge, - = charge)
// soc_pct: State of charge [%]
// n_cells: Number of series cells
// cell_mV: Per-cell voltages [mV] (size == n_cells)
// temps_C: Temperatures [°C]
// chgMOS/dsgMOS/state: MOSFET and state info from Daly
void uros_publish_battery(
  uint8_t battery_index,
  float v_total,
  float current_A,
  float soc_pct,
  uint8_t n_cells,
  const std::vector<uint16_t>& cell_mV,
  const std::vector<int8_t>& temps_C,
  uint8_t chgMOS,
  uint8_t dsgMOS,
  uint8_t state
);
