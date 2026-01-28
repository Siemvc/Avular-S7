#include "BMS.h"

BMS::BMS(uint8_t address) 
  : _address(address), _bus(nullptr) {}

bool BMS::init(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& bus) {
  _bus = &bus;
  return true;
}

uint16_t BMS::be16(const uint8_t* p) const {
  return (uint16_t(p[0]) << 8) | p[1];
}

uint32_t BMS::makeCanId(uint8_t DID, uint8_t DST, uint8_t SRC) const {
  return ((uint32_t)0x18 << 24) | ((uint32_t)DID << 16) | ((uint32_t)DST << 8) | SRC;
}

bool BMS::txRequest(uint8_t DID) {
  if (!_bus) return false;
  
  CAN_message_t tx = {};
  tx.id = makeCanId(DID, _address, PC_ADDR);
  tx.flags.extended = 1;
  tx.len = 8;
  memset(tx.buf, 0, 8);
  return _bus->write(tx);
}

bool BMS::rxResponse(uint8_t DID, CAN_message_t &rx, uint32_t timeout_ms) {
  if (!_bus) return false;
  
  uint32_t expect = makeCanId(DID, PC_ADDR, _address);
  elapsedMillis t = 0;

  while (t < timeout_ms) {
    if (_bus->read(rx)) {
      if (rx.id == expect && rx.flags.extended && rx.len == 8)
        return true;
    }
  }
  return false;
}

bool BMS::requestAndWait(uint8_t DID, CAN_message_t &rx) {
  if (!txRequest(DID)) return false;
  return rxResponse(DID, rx);
}

bool BMS::readPackVI(PackVI &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_PACK_SOC_V_I, rx)) return false;

  float v_cum = be16(&rx.buf[0]) * 0.1f;
  float v_gat = be16(&rx.buf[2]) * 0.1f;
  float i_raw = (int32_t(be16(&rx.buf[4])) - 30000) * 0.1f;
  float s_raw = be16(&rx.buf[6]) * 0.1f;

  o.cumulativeVoltage_V = v_cum;
  o.totalVoltage_V = v_gat;
  o.current_A = i_raw;
  o.soc_pct = s_raw;
  
  _packVI = o;
  return true;
}

bool BMS::readStatus1(Status1 &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_STATUS1, rx)) return false;

  o.nStrings = rx.buf[0];
  o.nTemps   = rx.buf[1];
  o.chargerConnected = rx.buf[2];
  o.loadConnected    = rx.buf[3];
  o.ioBits = rx.buf[4];
  
  _status1 = o;
  return true;
}

bool BMS::readTempData(TempData &o) {
  CAN_message_t rx;
  if (!requestAndWait(DID_TEMP_MOD, rx)) return false;

  float maxTemp = (int8_t)rx.buf[0];
  float minTemp = (int8_t)rx.buf[1];
  float avgTemp = (int8_t)rx.buf[2];

  o.maxTemp_C = maxTemp;
  o.minTemp_C = minTemp;
  o.avgTemp_C = avgTemp;
  
  _tempData = o;
  return true;
}
