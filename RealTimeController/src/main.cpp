/*
  Teensy 4.1 + Daly BMS CAN Reader
  Uses CAN3 interface (29-bit extended IDs), default 250 kbps
  
  CAN3 Pins on Teensy 4.1:
  - TX: Pin 30 (GPIO_EMC_32)
  - RX: Pin 31 (GPIO_EMC_33)
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>

// ====== CAN Setup ======
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CanBus;
#define DALY_CAN_500K 0
#define BMS_ADDR 0x01
#define PC_ADDR  0x40

enum DalyDataId : uint8_t {
  DID_PACK_SOC_V_I        = 0x90,
  DID_MAXMIN_VOLT         = 0x91,
  DID_MAXMIN_TEMP         = 0x92,
  DID_MOS_STATUS_CAP      = 0x93,
  DID_STATUS1             = 0x94,
  DID_CELL_VOLTAGES       = 0x95,
  DID_TEMPERATURES        = 0x96,
  DID_BALANCE_BITMAP      = 0x97,
  DID_FAULTS              = 0x98
};

// Data structures
struct PackVI { 
  float cumulativeVoltage_V = NAN;
  float totalVoltage_V = NAN;
  float current_A = NAN;
  float soc_pct = NAN;
};
struct Status1 { uint8_t nStrings = 0, nTemps = 0; };
struct MosCap { uint8_t chgMOS = 0, dsgMOS = 0, state = 0; };

static inline uint16_t be16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
static inline uint32_t be32(const uint8_t* p) { return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3]; }
static inline uint32_t makeCanId(uint8_t dataId, uint8_t dst, uint8_t src) {
  return ((uint32_t)0x18 << 24) | ((uint32_t)dataId << 16) | ((uint32_t)dst << 8) | src;
}

static void printFloatOrDash(float v, uint8_t digits=1){
  if (isnan(v)) Serial.print("—");
  else Serial.print(v, digits);
}

// Request and wait for response from BMS
bool requestAndWait(uint8_t dataId, CAN_message_t &rx, uint32_t timeout_ms = 120) {
  uint32_t reqId = makeCanId(dataId, BMS_ADDR, PC_ADDR);
  uint32_t respId = makeCanId(dataId, PC_ADDR, BMS_ADDR);
  CAN_message_t tx;
  tx.id = reqId;
  tx.flags.extended = 1;
  tx.len = 8;
  memset(tx.buf, 0, 8);
  CanBus.write(tx);

  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (CanBus.read(rx)) {
      if (rx.id == respId && rx.flags.extended && rx.len == 8) return true;
    }
  }
  return false;
}

static bool plausiblePackV(float v){ return (v > 5.0f && v < 100.0f); }

// Read pack voltage/current/SOC
bool readPackVI(PackVI &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_PACK_SOC_V_I, rx)) return false;
  const float v_cum = be16(&rx.buf[0]) * 0.1f;
  const float v_gat = be16(&rx.buf[2]) * 0.1f;
  const float i_raw = (int32_t(be16(&rx.buf[4])) - 30000) * 0.1f;
  const float s_raw = be16(&rx.buf[6]) * 0.1f;
  o.cumulativeVoltage_V = v_cum;
  if (plausiblePackV(v_gat)) o.totalVoltage_V = v_gat;
  else if (plausiblePackV(v_cum)) o.totalVoltage_V = v_cum;
  else o.totalVoltage_V = NAN;
  o.current_A = i_raw;
  o.soc_pct = s_raw;
  return true;
}

// Read basic status (cell count)
bool readStatus1(Status1 &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_STATUS1, rx)) return false;
  o.nStrings = rx.buf[0];
  o.nTemps = rx.buf[1];
  return true;
}

// Read MOS state
bool readMosCap(MosCap &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_MOS_STATUS_CAP, rx)) return false;
  o.state = rx.buf[0];
  o.chgMOS = rx.buf[1];
  o.dsgMOS = rx.buf[2];
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nDaly BMS CAN Reader (Teensy 4.1 + CAN3)");
  Serial.println("CAN3 Pins: TX=Pin 30, RX=Pin 31");
  Serial.printf("Baud: %s kbps\n", DALY_CAN_500K ? "500" : "250");
  
  CanBus.begin();
  CanBus.setBaudRate(DALY_CAN_500K ? 500000 : 250000);
  Serial.println("CAN3 started.");
  Serial.println("Listening for CAN messages...\n");
}

void loop() {
  // Just listen - don't send anything
  CAN_message_t msg;
  
  static uint32_t lastSerial = 0;
  static uint32_t lastBaudSwitch = 0;
  static bool use500k = false;
  
  // Try switching baud rate every 10 seconds
  if (millis() - lastBaudSwitch > 10000) {
    lastBaudSwitch = millis();
    use500k = !use500k;
    CanBus.setBaudRate(use500k ? 500000 : 250000);
    Serial.printf("Switched to %s kbps\n", use500k ? "500" : "250");
  }
  
  if (millis() - lastSerial > 1000) {
    lastSerial = millis();
    Serial.printf("ALIVE - waiting for CAN messages... (baud: %s kbps)\n", use500k ? "500" : "250");
  }
  
  if (CanBus.read(msg)) {
    Serial.printf("[RX] ID: 0x%08X  Len:%u  Data: ", msg.id, msg.len);
    for (int i = 0; i < msg.len; i++) {
      Serial.printf("%02X ", msg.buf[i]);
    }
    Serial.println();
  }
  
  delay(100);
}
