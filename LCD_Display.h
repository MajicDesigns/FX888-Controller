#pragma once

#include <LiquidCrystal_SR.h>
#include "Hardware.h"
#include "BarGraph.h"
#include "BigNum.h"
#include "Debug.h"

// LCD Display -------------------------------------------
// Manage the LCD display for the different modes of operation.
class LCD_Display : protected LiquidCrystal_SR
{
private:
  // Assumes a 2 line 16 character LCD module display.
  static const uint8_t LCD_ROWS = 2;
  static const uint8_t LCD_COLS = 16;

  // There are different types of displays, defined by the fields
  struct fieldDef_t { uint8_t row, col, len; };

  //
  // * MAIN DISPLAY *
  //  
  // +----------------+   D Double height current value
  // |DDDo M SSS XXXXX|   S SP for current value
  // |DDDU GGGGGGGGGGG|   M Control mode (Power or Temperature)
  // +----------------+   X Mode of Operation
  //  0123456789012345    G Bar graph of power applied
  //                      o degree symbol
  //                      C units of temperature
  //
  const fieldDef_t MDF_CV   = { 0, 0, 3 };
  const fieldDef_t MDF_DEG  = { 0, 3, 1 };
  const fieldDef_t MDF_UOM  = { 1, 3, 1 };
  const fieldDef_t MDF_PT   = { 0, 5, 1 };
  const fieldDef_t MDF_SP   = { 0, 7, 3 };
  const fieldDef_t MDF_MODE = { 0, 11, 5 };
  const fieldDef_t MDF_BG   = { 1, 5, 11 };

  //
  // * CALIBRATE *
  // +----------------+   
  // |DDDo LLLLLLLLLLL|   L Label for the parameter
  // |DDDU SSSSSSSSSSS|   S value for setpoint/value
  // +----------------+
  //  0123456789012345 
  //
  const fieldDef_t CDF_CV  = { 0, 0, 3 };
  const fieldDef_t CDF_DEG = { 0, 3, 1 };
  const fieldDef_t CDF_UOM = { 1, 3, 1 };
  const fieldDef_t CDF_LBL = { 0, 5, LCD_COLS };
  const fieldDef_t CDF_DTA = { 1, 5, LCD_COLS };

  //
  // * ERROR *
  // +----------------+   
  // |MMMMMMMMMMMMMMMM|   M Message Line 1 (Label)
  // |NNNNNNNNNNNNNNNN|   N Message LIne 2 (Error)
  // +----------------+
  //  0123456789012345 
  // 
  const fieldDef_t EDF_1 = { 0, 0, LCD_COLS };
  const fieldDef_t EDF_2 = { 1, 0, LCD_COLS };

  // Variables
  uint16_t   _pctGauge;
  uint16_t  _CV, _SP;
  char      _szLabel[LCD_COLS + 1];
  char      _szError[LCD_COLS + 1];
  BarGraph  *_bg;
  BigNum    *_bn;

  enum dispMode { DM_MAIN, DM_CONFIG, DM_ERROR } _dispMode;
  enum opMode   { OM_READY, OM_HEAT, OM_COOL } _opMode;
  enum tempMode { TM_DEGC, TM_DEGF, TM_RAW } _tempMode;
  enum ctlMode  { CM_TEMP, CM_PWR, CM_NONE } _ctlMode;

public:
  LCD_Display(uint8_t DAT, uint8_t CLK) : LiquidCrystal_SR(DAT, CLK)
  {
    _bg = new BarGraph(this, MDF_BG.row, MDF_BG.col, MDF_BG.len);
    _bn = new BigNum(this);

    setMainDisplay();
    setModeReady();
    setCV(0);
    setSP(0);
    setGaugePct(0);
    setTempCel();
    setLabel("");
  }

  ~LCD_Display()
  {
    delete _bn;
    delete _bg;
  }

  inline void clear(void) { LiquidCrystal_SR::clear(); }

  inline void setCV(uint16_t data) { _CV = data; }   // Set the current value for the measured variable
  inline void setSP(uint16_t data) { _SP = data; }   // Set the Set Point for the measured variable
  inline void setGaugePct(uint8_t data) { if (data < 100) _pctGauge = data; }  // Set the gauge as a %age

  // Set various display types
  inline void setMainDisplay(void) { _dispMode = DM_MAIN; }
  inline void setCfgDisplay(void)  { _dispMode = DM_CONFIG; }
  inline void setErrDisplay(void)  { _dispMode = DM_ERROR; }

  // Set various operating modes
  inline void setModeHeating(void) { _opMode = OM_HEAT; }
  inline void setModeCooling(void) { _opMode = OM_COOL; }
  inline void setModeReady(void)   { _opMode = OM_READY; }
  
  // Set temperature display
  inline void setTempCel(void)   { _tempMode = TM_DEGC; }
  inline void setTempFar(void)   { _tempMode = TM_DEGF; }
  inline void setTempRaw(void)   { _tempMode = TM_RAW; }

  // Set the control mode display
  inline void setCtlPower(void)  { _ctlMode = CM_PWR; }
  inline void setCtlTemp(void)   { _ctlMode = CM_TEMP; }
  inline void setCtlNone(void)   { _ctlMode = CM_NONE; }

  void begin(void)
  {
    // initialise LCD LCD_Display
    LiquidCrystal_SR::begin(LCD_COLS, LCD_ROWS);
    clear();
    noAutoscroll();
    noCursor();

    // initialise dependent objects
    _bn->begin();
    _bg->begin();
    _bg->setChar(7);  // 0-6 are used by bigNum
  }

  void setLabel(const char *szMesg)
  {
    strncpy(_szLabel, szMesg, sizeof(_szLabel) - 2);
    _szLabel[sizeof(_szLabel) - 1] = '\0';
  }

  void setError(const char *sz1, const char *sz2)
  {
    strncpy(_szLabel, sz1, sizeof(_szLabel) - 2);
    _szLabel[sizeof(_szLabel) - 1] = '\0';
    strncpy(_szError, sz2, sizeof(_szError) - 2);
    _szLabel[sizeof(_szError) - 1] = '\0';
  }

  void update(void)
  {
    char szLine[LCD_COLS + 1];

    this->noDisplay();    // temporarily disable the LCD display

    this->clear();
    switch (_dispMode)
    {
    case DM_MAIN:
      {
        // write the CV as double height digits
        _bn->writeNumber(MDF_CV.row, MDF_CV.col, _CV, MDF_CV.len);

        // units of measure
        this->setCursor(MDF_DEG.col, MDF_DEG.row);
        if (_tempMode == TM_DEGC || _tempMode == TM_DEGF)
          this->write(0xdf);
        else
          this->write(' ');
        this->setCursor(MDF_UOM.col, MDF_UOM.row);
        this->print(strMode(_tempMode));

        // Control mode
        this->setCursor(MDF_PT.col, MDF_PT.row);
        this->print(strMode(_ctlMode));

        // Set point
        this->setCursor(MDF_SP.col, MDF_SP.row);
        sprintf(szLine, "%%0%dd", MDF_SP.len);    // format string (* parameter not working?)
        sprintf(szLine, szLine, _SP);
        this->print(szLine);

        // Status message
        this->setCursor(MDF_MODE.col, MDF_MODE.row);
        this->print(strMode(_opMode));

        _bg->show(_pctGauge);
      }
      break;

    case DM_CONFIG:
      {
        // write the CV as double height digits
        _bn->writeNumber(CDF_CV.row, CDF_CV.col, _CV, CDF_CV.len);

        // Units of measure
        this->setCursor(CDF_DEG.col, CDF_DEG.row);
        if (_tempMode == TM_DEGC || _tempMode == TM_DEGF)
          this->write(0xdf);
        else
          this->write(' ');
        this->setCursor(CDF_UOM.col, CDF_UOM.row);
        this->print(strMode(_tempMode));

        // Label and Set point value
        this->setCursor(CDF_LBL.col, CDF_LBL.row);
        this->print(_szLabel);
        this->setCursor(CDF_DTA.col, CDF_DTA.row);
        this->print(_SP);
      }
      break;

    case DM_ERROR:
      {
        // Two line message
        this->setCursor(EDF_1.col, EDF_1.row);
        this->print(_szLabel);
        this->setCursor(EDF_2.col, EDF_2.row);
        this->print(_szError);
      }
      break;

    default:
      this->setCursor(0, 0);
      this->print("?Display mode");
      break;
    }

    this->display();    // re-enable the LCD display
  }

private:
  const char *strMode(enum opMode op)
  {
    switch (op)
    {
    case OM_READY: return("Ready");
    case OM_HEAT:  return(" Heat");
    case OM_COOL:  return(" Cool");
    }

    return("?????");
  }

  const char *strMode(enum tempMode op)
  {
    switch (op)
    {
    case TM_DEGC: return("C");
    case TM_DEGF: return("F");
    case TM_RAW:  return(" ");
    }

    return("?");
  }

  const char *strMode(enum ctlMode op)
  {
    switch (op)
    {
    case CM_TEMP: return("T");
    case CM_PWR:  return("P");
    case CM_NONE: return(" ");
    }

    return("?");
  }
};
