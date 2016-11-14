#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal_SR.h>
#include <MD_KeySwitch.h>
#include <MD_REncoder.h>
#include <ResponsiveAnalogRead.h>
#include "Debug.h"
#include "Hardware.h"
#include "ConfigData.h"
#include "Bargraph.h"
#include "LCD_Display.h"

// version 1.0 August 2016
// First implementation from hardware and software design at 
// https://create.arduino.cc/projecthub/sfrwmaker/soldering-iron-controller-for-hakko-907-8c5866
//
// External library dependencies
// LiquidCrystal_SR     https://bitbucket.org/fmalpartida/new-liquidcrystal/overview
// MD_REncoder          https://github.com/MajicDesigns/REncoder
// MD_KeySwitch         https://github.com/MajicDesigns/KeySwitch
// ResponsiveAnalogRead https://github.com/dxinteractive/ResponsiveAnalogRead

// Miscellaneous
void(*resetHW) (void) = 0; //declare reset function @ address 0

// Soldering IRON ----------------------------------------
// Main control class for the soldering iron.
// Within this class, all temperature measurements are in internal units.
//
// Control is carried out using the PID algorithm 
// Un = Kp*(Xs - Xn) + Ki*sum{j=0; j<=n}(Xs - Xj) + Kd(Xn-1 - Xn)
// We use the iterative formulae:
// Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(2*Xn-1 - Xn - Xn-2)
// With the first step:
// U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;

class IRON
{
private:
  const uint16_t PID_RUN_PERIOD = 600;      // time in milliseconds for running PID
  const uint16_t TEMP_READ_PERIOD = 200;    // update the curent value every PERIOD ms

  // heating safety check parameters
  const uint16_t  SAFETY_CHECK_TIME = 10000; // iron heating up safety check time, in ms
  const uint16_t  SAFETY_DELTA_T = 1;        // minimum temp difference at SAFETY_CHECK_TIME

  // PID tuning parameters
  const uint8_t DENOMINATOR_p = 10; // common coefficient denominator power of 2 (ie, 10 means 2^10 = 1024)
  const int32_t PID_Kp = 5120;  // Kp multiplied by denominator
  const int32_t PID_Ki = 512;   // Ki multiplied by denominator
  const int32_t PID_Kd = 768;   // Kd multiplied by denominator
  const int16_t PID_SWITCH_DELTA_T = 15;  // temp difference that triggers switch from initial to iterate PID function

  void applyPower(void);  // sety power after PID and apply power to the heater
  void resetPID(void);    // reset PID algorithm

  uint32_t _timeLastCVUpdate;       // time the temperature was last read
  int32_t  _timeLastSafetyCheck;    // time last safety check was held in ms
  int32_t  _timeLastPIDRun;         // time the last PID was run
  uint8_t _pinHeater, _pinSensor;   // the heater PWM and the sensor analog pin numbers

  bool _iteratePID;       // true when switch to PID iterative formula
  bool _isOn;             // true when the iron is on
  bool _isFixPowerMode;   // true when conttrol mode is for direct power
  bool _isUsed;           // iron has been on at some stage

  int16_t _powerSP;       // current soldering station power curent value
  ResponsiveAnalogRead *_sensor;
  uint8_t _error;         // current error code
                          // 0 - no error
                          // 1 - heating rate out of spec shutdown
                          // 2 - temperature range out of spec

  int16_t  _tempSP;       // temperature set point
  int16_t  _tempPrev;     // temperature when the iron was last safety checked
  uint16_t _tempMin;      // minimum temperature of control range
  uint16_t _tempMax;      // maximum temperature of control range
  int16_t  _tempHist[2];  // temperature history

public:
  IRON(uint8_t pinHeater, uint8_t pinSensor) :
    _pinHeater(pinHeater), _pinSensor(pinSensor),
    _isOn(false), _isFixPowerMode(false), _isUsed(false),
    _error(0)
  {
    _sensor = new ResponsiveAnalogRead(pinSensor, true, 0.001);
  }

  ~IRON()
  {
    delete _sensor;
  }

  void begin(uint16_t t_max, uint16_t t_min);

  // Status values requests
  inline bool isCold(void)    { return(getTempCV() < TEMP_COLD); }
  inline bool isOn(void)      { return(_isOn); }
  inline bool isWorking(void) { return(_error == 0); }
  inline bool isUsed(void)    { return(_isUsed); }

  inline void clearError(void)  { _error = 0; }
  inline uint8_t getError(void) { return(_error); }

  void switchPower(bool On);    // switch power on or off
  bool fixPower(uint8_t Power); // fixed power setting 

  void setTempSP(uint16_t t);       // set the temperature control setpoint
  inline uint16_t getTempSP(void);  // set the current temperature setpoint

  uint16_t getTempCV(void);               // current value for the temperature
  inline uint16_t getTempCVDegrees(void); // get the current temperature in degrees

  void setTempSPDegrees(int t);                 // set the temperature in degrees (celsius or farenheit)
  inline uint16_t getTempSPDegrees(void);       // get the temperature settpoint in degrees
  inline uint16_t temp2degrees(uint16_t temp);  // convert internal temp to degrees

  inline uint8_t getPowerSP(void);    // power that is applied to the soldering iron [MIN_POWER - MAX_POWER]
  inline uint8_t getPowerSPPct(void); // power applied to the iron as a percentage of full range [0-100%]
  inline uint8_t getPowerAvg(void);   // average applied power
  inline uint8_t getHotPercent(void); // how hot the iron is, as a % of full temperature setpoint range
    
  void controlTemp(void); // run the PID control
};

void IRON::begin(uint16_t t_max, uint16_t t_min)
{
  // initialise the hardware
  pinMode(_pinHeater, OUTPUT);
  
  // initialise variables
  _powerSP = 0;
  _timeLastCVUpdate = 0;
  _timeLastSafetyCheck = 0;
  _timeLastPIDRun = 0;
  _tempPrev = getTempCV();
  _tempMax = t_max;
  _tempMin = t_min;

  // set up the IRON class
  for (uint8_t i = 0; i < 10; i++)    // set up some history
    _sensor->update();  
  switchPower(false);
  resetPID();
}

void IRON::setTempSP(uint16_t t) 
// Set the temperature setpoint in internal units
{
  if (_isOn) resetPID();
  _tempSP = t;
  PRINT("\nIRON: Set temp SP=", t);
}

void IRON::setTempSPDegrees(int16_t t) 
// set the temperature setpoint in degrees
{
  uint16_t temp;

  PRINT("\nIRON: Set ", t);
  if (t < (int16_t)TEMP_MIN_DEG) t = TEMP_MIN_DEG;
  if (t > (int16_t)TEMP_MAX_DEG) t = TEMP_MAX_DEG;
  PRINT(" deg, now ", t);
  PRINT(" map to (", _tempMin);
  PRINT(",", _tempMax);
  PRINTS(")");
  temp = map(t+1, TEMP_MIN_DEG, TEMP_MAX_DEG, _tempMin, _tempMax);

  setTempSP(temp);
}

uint16_t IRON::getTempSP(void) 
{ 
  return(_tempSP);
}

uint16_t IRON::getTempSPDegrees(void)
// set the temperature setpoint in degrees
{
  uint16_t temp = getTempSP();

  temp = temp2degrees(temp);

  return(temp);
}

uint16_t IRON::temp2degrees(uint16_t t)
// Convert the internal temperature into degree units
{
  int16_t  temp;

  //PRINT("\nT2DEG: Temp ", t);
  //PRINT(" range (", _tempMin);
  //PRINT(",", _tempMax);
  //PRINTS(") map to (");

  //PRINT("", TEMP_MIN_DEG);
  //PRINT(",", TEMP_MAX_DEG);
  temp = map(t, _tempMin, _tempMax, TEMP_MIN_DEG, TEMP_MAX_DEG);

  //PRINT(") return ", temp);

  return(temp);
}

uint16_t IRON::getTempCV(void)
{
  uint16_t r;

  if (millis() - _timeLastCVUpdate >= TEMP_READ_PERIOD)
  {
    _sensor->update();
    _timeLastCVUpdate = millis();

    // check value in range
    r = _sensor->getValue();
    PRINT("\nIRON: Temp new CV read ", r);
    if (r < TEMP_CTL_MIN || r > TEMP_CTL_MAX)
    {
      PRINTS("\nIRON: CV range error");
      _error = 2;
      switchPower(false);
    }
  }
  else
    r = _sensor->getValue();

  return(r);
}

uint16_t IRON::getTempCVDegrees(void)
{
  uint16_t temp = getTempCV();

  temp = temp2degrees(temp);

  return(temp);
}

uint8_t IRON::getPowerSPPct(void)
// return the currently applied power as a percentage
{
  return(map(getPowerSP(), MIN_POWER, MAX_POWER, 0, 100));  
}

uint8_t IRON::getPowerSP(void) 
// Return the power setypoint value [MIN_POWER, MAX_POWER]
{
  return(_powerSP);  
}

uint8_t IRON::getHotPercent(void) 
// return the temperature percentage of the current setpoint
{
  uint8_t r = map(getTempCV(), TEMP_COLD, _tempSP, 0, 100);
  
  if (r < 0) r = 0;

  return(r);
}

void IRON::resetPID(void)
// reset the PID control
{
  PRINTS("\nIRON: Reset PID");
  _iteratePID = false;
  _tempHist[0] = _tempHist[1] = 0;
}

void IRON::controlTemp(void)
// Main method used to control the iron temperature using PID algorithm
// Does the following:
// 1. Update dampened CV. This is called every interation do that 
//    the update every TEMP_READ_PERIOD can occur.
// 2. Perform a safety check every SAFETY_CHECK_PERIOD.
// 3. Perform a PID calculation every PID_RUN_PERIOD.
{
  uint16_t tempCurr = getTempCV();    // need to call this often for timed updates

  if (!_isOn && !_isFixPowerMode)
  {
    // ensure iron is powered off
    switchPower(false);
    return;
  }

  // Safety check - is the iron heating?
  if (millis() - _timeLastSafetyCheck >= SAFETY_CHECK_TIME) 
  {
    // check if not changing much ..
    bool b = (!_isFixPowerMode && !_iteratePID && abs(_tempPrev - tempCurr) < SAFETY_DELTA_T);
    PRINT(" Chk0 (", _iteratePID); PRINT(",", _tempPrev); PRINT(",", tempCurr); PRINT(") = ", b);

    // ... or we are exceeding the 
    b = b || (tempCurr > TEMP_CTL_MAX);
    PRINT(" Chk1 (", tempCurr); PRINT(",", TEMP_CTL_MAX); PRINT(") = ", b);

    // reset checking values
    _timeLastSafetyCheck = millis();
    _tempPrev = tempCurr;

    // Prevent heater damage if not working
    if (b)
    {
      _error = 1;
      switchPower(false);
      return;
    }
  }

  // now do the PID if it is time
  if (!_isFixPowerMode && (millis() - _timeLastPIDRun >= PID_RUN_PERIOD))
  {
    _timeLastPIDRun = millis();

    PRINTS("\nIRON: PID Control - ")

      if (!_iteratePID) // use the direct PI formulae
      {
        int32_t p = PID_Kp*(_tempSP - tempCurr);
        p += PID_Ki * (_tempSP - tempCurr);
        p += (1 << (DENOMINATOR_p - 1));
        p >>= DENOMINATOR_p;

        _tempHist[1] = tempCurr;
        if (abs(_tempSP - tempCurr) < PID_SWITCH_DELTA_T) // If the temperature is close, prepare PID iteration process
        {
          _tempHist[0] = _tempHist[1]; // now ready to use iterate algorithm
          _iteratePID = true;
        }
        _powerSP = p;

        PRINTS("Direct");
      }
      else
      {
        int32_t delta_p = PID_Kp * (_tempHist[1] - tempCurr);
        delta_p += PID_Ki * (_tempSP - tempCurr);
        delta_p += PID_Kd * (2 * _tempHist[1] - _tempHist[0] - tempCurr);
        delta_p += (1 << (DENOMINATOR_p - 1));
        delta_p >>= DENOMINATOR_p;
        _powerSP += delta_p;
        _tempHist[0] = _tempHist[1];
        _tempHist[1] = tempCurr;

        PRINTS("Iterate");
      }

    PID_TRACE(millis(), getTempSP(), getPowerSP(), getTempCV());
    applyPower();
  }
}

void IRON::switchPower(bool setOn)
// Switch the iron on (true) or off (false)
// This should be the only place this happens so we can
// the different status values.
{
  _isOn = setOn;
  if (!_isOn)  // turn off
  {
    //PRINTS("\nIRON: Switching off");
    digitalWrite(_pinHeater, LOW);
    _isFixPowerMode = false;
    _isUsed = true;
  }
  else
  {
    PRINTS("\nIRON: Switching on");
    resetPID();
    _tempHist[1] = getTempCV();
    PID_TRACE_HEADER;
  }
}

void IRON::applyPower(void)
// Apply some power to the heater based on current control values
// this is used to control using the PID control loop
{
  uint8_t p = getPowerSP();

  if (_tempHist[1] > (_tempSP + 1))
  {
    PRINT("\nIRON: Hist[1]=", _tempHist[1]);
    PRINT(" tempSP=", _tempSP);
    p = 0;
  }
  
  PRINT("\nIRON: Apply Power=", p);
  if (p == 0)
    digitalWrite(_pinHeater, LOW);
  else if (_isOn)
    analogWrite(_pinHeater, p);
}

bool IRON::fixPower(uint8_t setPower)
// Switch the iron power setting directly
// To switch off, set the power to 0
{
  PRINT("\nIRON: FixPower ", setPower);

  _isFixPowerMode = (setPower != 0);
  if (!_isFixPowerMode)
  {
    PRINTS(" turning power off");
    switchPower(false);
  }
  else     // definitely set the power to something!
  {
    _isUsed = true;
    if (setPower > MAX_POWER) setPower = MAX_POWER;
    if (setPower < MIN_POWER) setPower = MIN_POWER;
    PRINT(" set to ", setPower);
    _powerSP = setPower;
    analogWrite(_pinHeater, _powerSP);
  }

  return(true);
}

// cRunState ---------------------------------------------
// Base class for device running state (finite state machine)
class cRunState
{
protected:
  static const uint16_t UPDATE_INTERVAL = 1000;  // in ms

  bool _forceRedraw;        // we need to draw the display
  uint32_t    _timeLastUpdate; // last display time (millis())
  LCD_Display *_pD;         // pointer to the Display instance
  ConfigData  *_pCfg;       // pointer to the Configuration instance
  IRON        *_pIron;      // pointer to the Iron instance

public:
  cRunState *_next;   // pointer to the next cRunState
  cRunState *_nextL;  // pointer to the next cRunState
  cRunState *_setup;  // pointer to the setup cRunState

  cRunState() 
  {
    _next = _nextL = _setup = nullptr;
    _forceRedraw = true;
    _pCfg = nullptr;
    _pD = nullptr;
    _timeLastUpdate = 0;
  }

  virtual void begin(void) { PRINTS("\nCRS: Base class begin()"); }
  virtual void handleEncoder(int8_t value) { PRINTS("\nCRS: Base class handleEncoder()"); }
  virtual void update(void) { PRINTS("\nCRS: Base class update()"); }

  virtual void show(void)
  { 
    // check if we should be doing an update
    if ((!_forceRedraw) && (millis() - _timeLastUpdate < UPDATE_INTERVAL)) 
      return; 

    // update display and reset triggers
    //PRINT("\nUpdate forced=", _forceRedraw);
    //PRINT(" time=", millis() - _timeLastUpdate);
    update();
    _forceRedraw = false;
    _timeLastUpdate = millis();
  }

  virtual cRunState *next(void)     { return((_next  != nullptr) ? _next  : this); }
  virtual cRunState *nextLong(void) { return((_nextL != nullptr) ? _nextL : this); }
  virtual cRunState *nextSetup(void){ return((_setup != nullptr) ? _setup : this); }

  void forceRedraw(void) { _forceRedraw = true; }
};

// idleRunState [soldering iron OFF] ---------------------
// Starting state for the device in off mode
class idleRunState : public cRunState 
{
public:
  idleRunState(IRON *pIron, LCD_Display *pLCD_Display, ConfigData *pCfg)
  {
    _pIron = pIron;
    _pD = pLCD_Display;
    _pCfg = pCfg;
  }

  virtual void begin(void)
  {
    PRINTS("\nIDLE ======");
    // reset the iron the devices to the idle state
    _pIron->switchPower(false);

    // set up display static data and then force dynamic data update
    _pD->setMainDisplay();
    _pD->setCtlNone();
#if USE_CELCIUS
    _pD->setTempCel();
#else
    _pD->setTempFar();
#endif
    forceRedraw();

    if (_pIron->isUsed()) // the iron was used, should save new temp setpoint to EEPROM
    {
      _pCfg->setTempSP(_pIron->getTempSP());
      _pCfg->save();
    }
  }

  virtual void update(void)
  {
    _pD->setSP(_pIron->getTempSPDegrees());
    _pD->setCV(_pIron->getTempCVDegrees());
    _pD->setGaugePct(_pIron->getHotPercent());

    // set the message to what we think is going on
    if (_pIron->isUsed() && !_pIron->isCold())   // not yet cold
      _pD->setModeCooling();
    else
      _pD->setModeReady();

    _pD->update();
  }

  virtual void handleEncoder(int8_t value)
  {
    _pIron->setTempSPDegrees(_pIron->getTempSPDegrees() + value);
    _pD->setSP(value);

    forceRedraw();
  }
};

// ctlTempRunState [soldering iron ON, Temp control] -----
// Normal run mode with device in temperature control mode
class ctlTempRunState : public cRunState 
{
public:
  ctlTempRunState(IRON *pIron, LCD_Display *pLCD_Display) 
  {
	  _pIron = pIron;
	  _pD = pLCD_Display;
  }

  virtual void begin(void)
  {
    PRINTS("\nTEMP ======");

    // start the control sequence
    _pIron->switchPower(true);

    // set up display static data and then force dynamic data update
    _pD->setMainDisplay();
    _pD->setCtlTemp();
#if USE_CELCIUS
    _pD->setTempCel();
#else
    _pD->setTempFar();
#endif
    forceRedraw();
  }

  void update(void)
  {
    _pD->setSP(_pIron->getTempSPDegrees());
    _pD->setCV(_pIron->temp2degrees(_pIron->getTempCV()));
    _pD->setGaugePct((uint8_t)_pIron->getPowerSPPct());

    if (abs(_pIron->getTempSP() - _pIron->getTempCV()) < 4)
      _pD->setModeReady();
    else if (_pIron->getTempCV() < _pIron->getTempSP())
      _pD->setModeHeating();

    _pD->update();
  }

  virtual void handleEncoder(int8_t value)
  {
    _pIron->setTempSPDegrees(_pIron->getTempSPDegrees() + value);
    _pD->setSP(value);

    forceRedraw();
  }
};

// ctlPowerRunState [soldering iron ON, fixed power] -----
// Fixed power running mode
class ctlPowerRunState : public cRunState
{
public:
  ctlPowerRunState(IRON *pIron, LCD_Display *pLCD_Display)
  {
    _pIron = pIron;
    _pD = pLCD_Display;
  }

  void begin(void)
  {
    PRINTS("\nPOWER =====");

    // start the control sequence
    _pIron->switchPower(false);
    _pIron->fixPower(_pIron->getPowerSP());

    // set up display static data and then force dynamic data update
    _pD->setMainDisplay();
    _pD->setCtlPower();
#if USE_CELCIUS
    _pD->setTempCel();
#else
    _pD->setTempFar();
#endif
    forceRedraw();
  }

  void update(void)
  {
    _pD->setSP(_pIron->getPowerSP());
    _pD->setCV(_pIron->temp2degrees(_pIron->getTempCV()));
    _pD->setGaugePct(_pIron->getPowerSPPct());

    if (abs(_pIron->getTempSP() - _pIron->getTempCV()) < 4) 
      _pD->setModeReady();
    else if (_pIron->getTempCV() < _pIron->getTempSP())
      _pD->setModeHeating();

    _pD->update();
  }

  void handleEncoder(int8_t value)
  {
    int16_t p = _pIron->getPowerSP() + value;

    PRINT("\nTUNE: Encoder ", value);

    // make it fit into a uint8_t
    if (p < 0) p = 0;
    if (p > 255) p = 255;
    _pIron->fixPower(p);

    forceRedraw();
  }
};

// tuneRunState ------------------------------------------
// Tuning parameter setup for the device
class tuneRunState : public cRunState
{
private:
  // parameter labels and values
  static const uint8_t MAX_PARAM = 2;
  static const uint8_t P_RANGE_HI = 0;
  static const uint8_t P_RANGE_LO = 1;

  static const uint8_t LABEL_SIZE = 11;

  uint8_t _curItem;
  char _label[LABEL_SIZE+1];
  uint16_t _param[MAX_PARAM];

public:
  tuneRunState(IRON *pIron, LCD_Display *pLCD_Display, ConfigData *pCfg)
  {
    _pIron = pIron;
    _pD = pLCD_Display;
    _pCfg = pCfg;
  }

  void begin(void)
  {
    PRINTS("\nTUNE ======");

    // set all parameters to zero and then ...
    for (uint8_t i = 0; i < MAX_PARAM; i++)
      _param[i] = 0;

    // .. load pre-existing config data
    _pCfg->getCalibrationRange(_param[P_RANGE_HI], _param[P_RANGE_LO]);
    _curItem = 0;

    // turn on fixed power
    _pIron->fixPower((MAX_POWER + MIN_POWER) / 2);

    // set up the display
    _pD->setCfgDisplay();
    _pD->setTempRaw();
    forceRedraw();
  }

  void saveParam(void)
  {
    _param[_curItem] = _pIron->getTempCV();
  }

  void update(void)
  {
    uint16_t temp;
    
    temp = (_curItem == 0 ? TEMP_MAX_DEG : TEMP_MIN_DEG);

    sprintf(_label, "Set %03d deg", temp);

    _pD->setCV(_pIron->getPowerSP());
    _pD->setLabel(_label);
    _pD->setSP(_pIron->getTempCV());

    _pD->update();
  }

  void handleEncoder(int8_t value)
  {
    int16_t p = _pIron->getPowerSP() + value;

    PRINT("\nTUNE: Encoder ", value);

    // make it fit into a uint8_t
    if (p < 0) p = 0;
    if (p > 255) p = 255;
    _pIron->fixPower(p);

    forceRedraw();
  }

  cRunState *next(void)
  // 'next display' is the same display with different item
  {
    saveParam();

    _curItem++;

    if (_curItem < MAX_PARAM)
      forceRedraw();    // do the next parameter
    else 
    {
      // // all parameters processed - turn the iron off ...
      _pIron->fixPower(0);    // turn it off!

      // ... save config, and ...
      _pCfg->setCalibrationRange(_param[P_RANGE_HI], _param[P_RANGE_LO]);
      _pCfg->save();

      // ... reset the hardware to reinitialise all
      resetHW();
    }

    return(this);
  }
};

// errorRunState ------------------------------------------
// Display an error message
class errorRunState : public cRunState
{
private:
  static const uint8_t LABEL_SIZE = 16;
  static const uint8_t MAX_ERROR = 4;     // includes heading
  static const PROGMEM char _labels[MAX_ERROR][LABEL_SIZE+1];

  char _label[2][LABEL_SIZE+1];

  char *getLabel(char *szBuf, uint8_t idx, uint8_t len)
  {
    strncpy_P(szBuf, _labels[idx], len);
    szBuf[len] = '\0';

    return(szBuf);
  }

public:
  errorRunState(IRON *pIron, LCD_Display *pLCD_Display)
  {
    _pIron = pIron;
    _pD = pLCD_Display;
  }

  void begin(void)
  {
    PRINTS("\nERROR ======");

    // turn the iron off
    _pIron->switchPower(false);

    // set up the error message and heading
    if (_pIron->getError() < MAX_ERROR-1) 
      getLabel(_label[1], _pIron->getError() + 1, LABEL_SIZE);  // error mesg, skip heading text
    else
      sprintf(_label[1], getLabel(_label[0], 0, LABEL_SIZE), _pIron->getError()); // unknown, format mesg
    getLabel(_label[0], 1, LABEL_SIZE);   // heading

    // set up the display
    _pD->setErrDisplay();
    forceRedraw();

    _pIron->clearError();
  }

  void update(void)
  {
    _pD->setError(_label[0], _label[1]);
    _pD->update();
  }
};

const PROGMEM char errorRunState::_labels[MAX_ERROR][LABEL_SIZE+1] =
{ 
// 0123456789012345   <- character position
  "?Error %02d",      // Unknown error
  "Check iron",       // Display heading
  "Safety shutdown",  // Real error messages start here (err no + 1)
  "T sensor range",
};

// End of class declarations =============================

ConfigData    ironCfg;
LCD_Display   disp(LCD_DAT_PIN, LCD_CLK_PIN);
IRON          iron(heaterPIN, sensorPIN);
MD_REncoder   REncoder(RE_MAIN_PIN, RE_SECD_PIN);
MD_KeySwitch  RESwitch(RE_SWITCH_PIN);

// Run State Machine states
// Finite State Diagram
// --------------------
//   stTune <---- stIdle <--- stError
//                 ^           ^ ^
//                 |           | |
//                 +-> stCtlT -+ |
//                      ^        +--+
//                      |           |
//                      +-> stCtlP -+
//
idleRunState      stIdle(&iron, &disp, &ironCfg); // main state at power on
ctlTempRunState   stCtlT(&iron, &disp);           // temperature control runing (main run mode)
ctlPowerRunState  stCtlP(&iron, &disp);           // fixed power running
tuneRunState      stTune(&iron, &disp, &ironCfg); // temperatrure tuning
errorRunState     stError(&iron, &disp);          // error message displayed

cRunState *pCurrentRunState = &stIdle;

void setup() 
{
#if DEBUG || PID_PROFILE
  Serial.begin(57600);
  PRINTS("\n[Soldering Controller]");
#endif // DEBUG
  // Start the display
  disp.begin();

  // Initialize rotary encoder and its switch
  RESwitch.begin();
  RESwitch.enableRepeat(false);
  REncoder.begin();
  REncoder.setPeriod(500);

  // Load Configuration parameters and initialise the iron
  ironCfg.begin();
  ironCfg.load();
  {
    uint16_t tempMin, tempMax;
    ironCfg.getCalibrationRange(tempMax, tempMin);
    iron.begin(tempMax, tempMin);
    iron.setTempSP(ironCfg.getTempSP());
  }

  // Initialize cRunState hierarchy to reflect the 
  // state machine transitions when the user moves between state
  // - next by a click
  // - nextL by a long press
  // - setup by a double click
  //
  // cRunState  next      nextL     setup
  // -----------------------------------
  // stIdle     ctlTemp             stTune
  // stCtlT     stIdle      stCtlP
  // stCtlP     stIdle      stCtlT
  // stTune     <next cfg item, then reset>
  // stError    stIdle
  //
  stIdle._next = &stCtlT;
  stIdle._setup = &stTune;

  stCtlT._next = &stIdle;
  stCtlT._nextL = &stCtlP;

  stCtlP._next = &stIdle;
  stCtlP._nextL = &stCtlT;  
  
  stTune._setup = &stIdle;

  stError._next = &stIdle;

  pCurrentRunState->begin();

  PRINTS("\nsetup() completed");
}

void loop() 
{
  cRunState *nxt = nullptr;

  // process rotary encoder
  uint8_t pos = REncoder.read();
  if (pos != DIR_NONE)
  {
    int8_t v = (REncoder.speed() < 5) ? 1 : 5;
    v *= (pos == DIR_CW ? 1 : -1);   // add in the sign
    PRINT("\n-> Encoder n=", v);
    pCurrentRunState->handleEncoder(v);
  }

  // process key switch to navigate the states
  switch (RESwitch.read()) 
  {
  case MD_KeySwitch::KS_PRESS:
    PRINTS("\n-> Press");
    nxt = pCurrentRunState->next();
    break;

  case MD_KeySwitch::KS_LONGPRESS:
    PRINTS("\n--> Long Press");
    nxt = pCurrentRunState->nextLong();
    break;

  case MD_KeySwitch::KS_DPRESS:
    PRINTS("\n=> DPress");
    nxt = pCurrentRunState->nextSetup();
    break;

  case MD_KeySwitch::KS_NULL:
    // do nothing
    break;
  }

  // check if an error has occurred and override nxt if required
  if (!iron.isWorking())
    nxt = &stError;

  // if we changed run state, process the change
  if (nxt != nullptr && nxt != pCurrentRunState)
  {
    pCurrentRunState = nxt;
    pCurrentRunState->begin();
  }
  pCurrentRunState->show();

  // finally, run the iron control
  iron.controlTemp();
}