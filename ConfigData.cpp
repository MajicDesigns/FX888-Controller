#include "ConfigData.h"

// Configuration data ------------------------------------

void ConfigData::begin(void)
// Read all valid records and note the one with the highest record ID.
//  If a valid config is found, _isValid set to true and _rAddr is set
// to the start address. Point _wAddr (write address) after the last 
// record, with wraparound, ready for the next save.
{
  cfg myCfg;    // temporary location
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;

  PRINTS("\nStart EEPROM search");
  // Scan all the EEPROM memory all the records in the EEPROM 
  // to find min and max record ID
  for (uint16_t addr = 0; addr < EEPROM.length(); addr += RECORD_SIZE)
  {
    if (readRecord(addr, myCfg))
    {
      _isValid = true;    // we have at least one available
      if (maxRecID < myCfg.recID)
      {
        maxRecID = myCfg.recID;
        maxRecAddr = addr;
      }
    }
    else
      break;    // no more valid configs found
  }

  // if we found no records then this is the first 
  // and we can write to the start of the EEPROM
  if (!_isValid)
  {
    PRINTS("\n--> no records found");
    _wAddr = 0;
    _rAddr = 0;
  }
  else
  {
    PRINTS("\n--> found");
    // There were records, so we now know the addresses to use
    _rAddr = maxRecAddr;
    _wAddr = _rAddr + RECORD_SIZE;
    if (_wAddr + RECORD_SIZE > EEPROM.length()) // won't fit, go back to EEPROM start
    {
      PRINTS(" & write address wraparound");
      _wAddr = 0;
    }
  }
}

void ConfigData::setTempSP(uint16_t t)
// Save the new temperature setpoint value
{
  _dataChanged |= (t != _cfgData.tempSP);
  _cfgData.tempSP = t;
  PRINT("\nCFG: Set SP=", t);
  PRINT(" data changed=", _dataChanged);
}

void ConfigData::setCalibrationRange(uint16_t max, uint16_t min)
// Set the new control calibration range
{
  _dataChanged |= (_cfgData.tempMax != max) || (_cfgData.tempMin = min);
  _cfgData.tempMax = max;
  _cfgData.tempMin = min;
  PRINT("\nCFG: Set max=", max);
  PRINT(" min=", min)
  PRINT(" data changed=", _dataChanged);
}

void ConfigData::getCalibrationRange(uint16_t &max, uint16_t &min)
// Get the current control calibrated range
{
  PRINT("\nCFG: Get Range max=", _cfgData.tempMax);
  PRINT(" min=", _cfgData.tempMin)
  max = _cfgData.tempMax;
  min = _cfgData.tempMin;
}

bool ConfigData::save(void)
// Save the current parameters at the next EEPROM location if values have changed.
// Return true if the config needed saving, false if it did not need to be saved.
{
  uint8_t summ = 0;     // running checksum
  uint8_t *p = (uint8_t *)&_cfgData;
  uint16_t wAddr = _wAddr;

  PRINT("\nCFG: Saving config, datachanged=", _dataChanged);
  PRINT(" wAddr=", _wAddr);
  if (_dataChanged)
  {
    _cfgData.recID++;    // use the next record ID

    PRINT(" new recID=", _cfgData.recID);

    // write out the record except, calculate the 
    // checksum as we go
    for (uint8_t i = 0; i < RECORD_SIZE - sizeof(_cfgData.chk); i++)
    {
      summ <<= 2;
      summ += *p;
      EEPROM[wAddr++] = *p++;
    }
    // now write the checksum at the end
    summ++;   // add one to the chk
    PRINT(" chk=", summ);
    EEPROM[wAddr] = summ;

    // adjust our global addresses for the next record
    _rAddr = _wAddr;
    _wAddr += RECORD_SIZE;
    if (_wAddr + RECORD_SIZE > EEPROM.length())
    {
      PRINTS(" & write address wraparound");
      _wAddr = 0;
    }

    dumpEEPROM(_rAddr, RECORD_SIZE);
  }

  return(_dataChanged);
}

bool ConfigData::load(void)
// Load the current config item from EEPROM if we have previously found
// a valid config record. Return true if the record was loaded, false otherwise.
// 
{
  PRINT("\nCFG: Loading, isValid=", _isValid);

  if (_isValid)
  {
    readRecord(_rAddr, _cfgData);

    // if the read is valid, check the sanity of the data retrieved and reset
    // to default if they are out of range
    if (_cfgData.tempMin >= _cfgData.tempMax)
    {
      _cfgData.tempMin = TEMP_CTL_MIN;
      _cfgData.tempMax = TEMP_CTL_MAX;
    }

    if ((_cfgData.tempSP > _cfgData.tempMax) || (_cfgData.tempSP < _cfgData.tempMin))
      _cfgData.tempSP = TEMP_CTL_DEFAULT;

    PRINT("\nCFG: loaded data min=", _cfgData.tempMin);
    PRINT(" max=", _cfgData.tempMax);
    PRINT(" SP=", _cfgData.tempSP);
    _dataChanged = false;
  }

  return(_isValid);
}

bool ConfigData::readRecord(uint16_t rAddr, struct cfg &rec)
{
  struct cfg myCfg;   // temporary config record
  uint8_t *p = (uint8_t *)&myCfg;
  uint8_t summ = 0;   // running checksum
  uint8_t chk;
  bool  isGood;

  dumpEEPROM(rAddr, RECORD_SIZE);

  PRINT("\nReading record addr=", rAddr);
  // load the record data, calculating the checksum as we go
  for (uint8_t i = 0; i < RECORD_SIZE - sizeof(_cfgData.chk); i++)
  {
    *p = EEPROM[rAddr++];
    summ <<= 2;
    summ += *p++;
  }
  summ++; // chk is always incremented
  PRINT(" chk=", summ);

  // is chk correct?
  isGood = (summ == EEPROM[rAddr]);

  if (isGood)
  {
    PRINTS(" - matches EEPROM!");
    myCfg.chk = chk;    // might as well save this for debugging
    rec = myCfg;        // copy back to caller
  }
  else
    PRINTS(" - no match!");

  return(isGood);
}

bool ConfigData::dumpEEPROM(uint16_t addr, uint8_t size)
// Debug routine to dump EEPROM
{
  PRINT("\nEEPROM DUMP @", addr);
  PRINT(" bytes:", size);
  for (uint8_t i = 0; i < size; i++)
  {
    if (i % 16 == 0) { PRINT("\n", addr); PRINTS(":"); }
    PRINT(" ", (uint8_t)EEPROM[addr+i]);
  }
}
