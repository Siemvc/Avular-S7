
#pragma once
#include <Arduino.h>
#include <vector>

// Initialize micro-ROS node and publishers for two batteries (USB CDC serial)
void uros_init(uint32_t baud = 115200);

// Spin micro-ROS executor (non-blocking). Call in loop().
void uros_spin_once(uint32_t timeout_ms = 0);

// Publish battery info for battery #1 or #2
void uros_publish_battery(
  uint8_t battery_index,          // 1 or 2
  float v_total,                  // Pack voltage [V]
  float current_A,                // Pack current [A] (+ = discharge, - = charge)
  float soc_pct,                  // SOC [%]
  uint8_t n_cells,                // Series cell count
  const std::vector<uint16_t>& cell_mV,  // per-cell mV (size==n_cells) (optional)
  const std::vector<int8_t>&  temps_C,   // °C (optional)
  uint8_t chgMOS, uint8_t dsgMOS, uint8_t state  // from Daly 0x93
);
