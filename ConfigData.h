#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "Hardware.h"
#include "Debug.h"

// Configuration data ------------------------------------
// Config record in the EEPROM has the following format:
struct cfg
{
  uint32_t recID;     // the record id 

  uint16_t tempMin;   // minimum control units temperature
  uint16_t tempMax;   // maximum control units temperature
  uint16_t tempSP;    // iron temp set point

  uint8_t  chk;       // the checksum after the record 
};

class ConfigData
{
private:
  const uint8_t RECORD_SIZE = sizeof(_cfgData); // size of one record in bytes

  struct cfg _cfgData;
  bool _isValid;          // data for a valid config was located
  bool _dataChanged;      // flag that data has changed
  uint16_t _wAddr;        // EEPROM address for new record write
  uint16_t _rAddr;        // EPPROM adress where we found the valid config

  bool readRecord(uint16_t addr, struct cfg &rec);
  bool dumpEEPROM(uint16_t addr, uint8_t size);   // for debugging

public:
  ConfigData() : 
    _isValid(false), _dataChanged(false), _wAddr(0), _rAddr(0)
  {
    _cfgData.recID = 0;
    _cfgData.tempSP = TEMP_CTL_DEFAULT;
    _cfgData.tempMin = TEMP_CTL_MIN;
    _cfgData.tempMax = TEMP_CTL_MAX;
  }

  void begin();

  bool load(void);
  bool save(void);

  inline uint16_t  getTempSP(void) { return(_cfgData.tempSP); }
  void setTempSP(uint16_t t);

  void getCalibrationRange(uint16_t &max, uint16_t &min);
  void setCalibrationRange(uint16_t max, uint16_t min);
};
