
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include "micro_ros_battery.h"

// ====== CAN Setup ======
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CanBus;
#define DALY_CAN_500K 0
#define BMS_ADDR_1 0x01
#define BMS_ADDR_2 0x02
#define PC_ADDR    0x40

// Data structures
struct PackVI { float totalVoltage_V = NAN, current_A = NAN, soc_pct = NAN; };
struct Status1 { uint8_t nStrings = 0; };
struct MosCap { uint8_t chgMOS = 0, dsgMOS = 0, state = 0; };

static inline uint16_t be16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
static inline uint32_t makeCanId(uint8_t dataId, uint8_t dst, uint8_t src) {
  return ((uint32_t)0x18 << 24) | ((uint32_t)dataId << 16) | ((uint32_t)dst << 8) | src;
}

// Request and wait for response from a specific BMS
bool requestAndWait(uint8_t dataId, uint8_t bmsAddr, CAN_message_t &rx, uint32_t timeout_ms = 120) {
  uint32_t reqId = makeCanId(dataId, bmsAddr, PC_ADDR);
  uint32_t respId = makeCanId(dataId, PC_ADDR, bmsAddr);
  CAN_message_t tx;
  tx.id = reqId; tx.flags.extended = 1; tx.len = 8; memset(tx.buf, 0, 8);
  CanBus.write(tx);

  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (CanBus.read(rx)) {
      if (rx.id == respId && rx.flags.extended && rx.len == 8) return true;
    }
  }
  return false;
}

// Read pack voltage/current/SOC
bool readPackVI(PackVI &o, uint8_t addr) {
  CAN_message_t rx; if (!requestAndWait(0x90, addr, rx)) return false;
  float v_cum = be16(&rx.buf[0]) * 0.1f;
  float v_gat = be16(&rx.buf[2]) * 0.1f;
  float i_raw = (int32_t(be16(&rx.buf[4])) - 30000) * 0.1f;
  float s_raw = be16(&rx.buf[6]) * 0.1f;
  o.totalVoltage_V = (v_gat > 5 && v_gat < 100) ? v_gat : v_cum;
  o.current_A = i_raw; o.soc_pct = s_raw;
  return true;
}

// Read basic status (cell count)
bool readStatus1(Status1 &o, uint8_t addr) {
  CAN_message_t rx; if (!requestAndWait(0x94, addr, rx)) return false;
  o.nStrings = rx.buf[0]; return true;
}

// Read MOS state
bool readMosCap(MosCap &o, uint8_t addr) {
  CAN_message_t rx; if (!requestAndWait(0x93, addr, rx)) return false;
  o.state = rx.buf[0]; o.chgMOS = rx.buf[1]; o.dsgMOS = rx.buf[2]; return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Teensy Dual Daly BMS CAN + micro-ROS...");
  CanBus.begin();
  CanBus.setBaudRate(DALY_CAN_500K ? 500000 : 250000);
  uros_init(115200); // Initialize micro-ROS
}

void loop() {
  // Battery 1
  PackVI p1; Status1 s1a; MosCap mos1;
  readPackVI(p1, BMS_ADDR_1);
  readStatus1(s1a, BMS_ADDR_1);
  readMosCap(mos1, BMS_ADDR_1);

  // Battery 2
  PackVI p2; Status1 s1b; MosCap mos2;
  readPackVI(p2, BMS_ADDR_2);
  readStatus1(s1b, BMS_ADDR_2);
  readMosCap(mos2, BMS_ADDR_2);

  // Optional: cell voltages and temps (empty for now)
  std::vector<uint16_t> cells1, cells2;
  std::vector<int8_t> temps1, temps2;

  // Publish to ROS 2
  uros_publish_battery(1, p1.totalVoltage_V, p1.current_A, p1.soc_pct, s1a.nStrings, cells1, temps1, mos1.chgMOS, mos1.dsgMOS, mos1.state);
  uros_publish_battery(2, p2.totalVoltage_V, p2.current_A, p2.soc_pct, s1b.nStrings, cells2, temps2, mos2.chgMOS, mos2.dsgMOS, mos2.state);

  // Keep micro-ROS alive
  uros_spin_once(0);

  delay(1000); // Poll interval
}
