
/*
  Teensy 4.1 + Waveshare SN65HVD230
  Daly BMS CAN Reader - Supports TWO Daly BMS units
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include <cstring>


// Using CAN3 (pins 30 = TX, 31 = RX)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

// --- TWO BMS addresses ---
uint8_t BMS_ADDR_LIST[2] = { 0x01, 0x02 };

static uint8_t PC_ADDR = 0x40;

enum DalyDataId : uint8_t {
  DID_PACK_SOC_V_I  = 0x90,
  DID_STATUS1        = 0x94
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

static inline uint16_t be16(const uint8_t* p){
  return (uint16_t(p[0]) << 8) | p[1];
}

static inline uint32_t makeCanId(uint8_t DID, uint8_t DST, uint8_t SRC){
  return ((uint32_t)0x18 << 24) | ((uint32_t)DID << 16) | ((uint32_t)DST << 8) | SRC;
}

static void printFloatOrDash(float v, uint8_t digits = 1){
  if (isnan(v)) Serial.print("—");
  else Serial.print(v, digits);
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

bool plausiblePackV(float v){ return (v > 5 && v < 100); }

// ----- Parsing -----

bool readPackVI(uint8_t BMS_ADDR, PackVI &o){
  CAN_message_t rx;
  if (!requestAndWait(DID_PACK_SOC_V_I, BMS_ADDR, rx)) return false;

  float v_cum = be16(&rx.buf[0]) * 0.1f;
  float v_gat = be16(&rx.buf[2]) * 0.1f;
  float i_raw = (int32_t(be16(&rx.buf[4])) - 30000) * 0.1f;
  float s_raw = be16(&rx.buf[6]) * 0.1f;

  o.cumulativeVoltage_V = v_cum;
  o.totalVoltage_V = plausiblePackV(v_gat) ? v_gat : v_cum;
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

// ---- MAIN ----

void setup(){
  Serial.begin(115200);
  delay(700);

  Serial.println("\nDaly BMS (Dual-BMS) CAN Reader - Teensy 4.1 (CAN3)");
  Serial.printf("Baud: 250k\n");

  if (!canInit()){
    Serial.println("CAN init FAILED");
    while(1);
  }
  Serial.println("CAN started.");
}

void loop(){
  for (int i = 0; i < 2; i++){
    uint8_t BMS = BMS_ADDR_LIST[i];

    Serial.printf("\n--- BMS #%d  (ADDR=0x%02X) ---\n", i+1, BMS);

    Status1 s1;
    if (!readStatus1(BMS, s1)){
      Serial.println("STATUS1 timeout.");
      continue;
    }

    Serial.printf("Cells:%u Temps:%u Charger:%u Load:%u IO:0x%02X\n",
                  s1.nStrings, s1.nTemps, s1.chargerConnected,
                  s1.loadConnected, s1.ioBits);

    PackVI p;
    if (readPackVI(BMS, p)){
      Serial.print("Pack: V(cum)="); printFloatOrDash(p.cumulativeVoltage_V);
      Serial.print(" V  Vtot=");     printFloatOrDash(p.totalVoltage_V);
      Serial.print(" V  I=");        printFloatOrDash(p.current_A);
      Serial.print(" A  SOC=");      printFloatOrDash(p.soc_pct);
      Serial.println(" %");
    }
  }

  delay(1000);  // one full scan per second
}
