#ifndef BMS_H
#define BMS_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <cmath>

// Data structures
struct PackVI {
  float cumulativeVoltage_V = NAN;
  float totalVoltage_V      = NAN;
  float current_A           = NAN;
  float soc_pct             = NAN;
};

struct Status1 {
  uint8_t nStrings          = 0;
  uint8_t nTemps            = 0;
  uint8_t chargerConnected  = 0;
  uint8_t loadConnected     = 0;
  uint8_t ioBits            = 0;
};

struct TempData {
  float maxTemp_C = NAN;
  float minTemp_C = NAN;
  float avgTemp_C = NAN;
};

enum DalyDataId : uint8_t {
  DID_PACK_SOC_V_I  = 0x90,
  DID_STATUS1       = 0x94,
  DID_TEMP_MOD      = 0x92
};

class BMS {
public:
  BMS(uint8_t address);

  bool init(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus);
  
  bool readPackVI(PackVI &o);
  bool readStatus1(Status1 &o);
  bool readTempData(TempData &o);
  
  PackVI getPackVI() const { return _packVI; }
  Status1 getStatus1() const { return _status1; }
  TempData getTempData() const { return _tempData; }

private:
  uint8_t _address;
  FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>* _bus;
  
  PackVI _packVI;
  Status1 _status1;
  TempData _tempData;
  
  static const uint8_t PC_ADDR = 0x40;
  static const uint32_t CAN_TIMEOUT_MS = 120;
  
  uint16_t be16(const uint8_t* p) const;
  uint32_t makeCanId(uint8_t DID, uint8_t DST, uint8_t SRC) const;
  
  bool txRequest(uint8_t DID);
  bool rxResponse(uint8_t DID, CAN_message_t &rx, uint32_t timeout_ms = 120);
  bool requestAndWait(uint8_t DID, CAN_message_t &rx);
};

#endif
