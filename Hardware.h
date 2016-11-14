#pragma once

// Hardware definition file
// All user settings and configuration items are set in this file.

// Define the units for the user temperature display
// 1 = celcius, 0 = farenheit
#define USE_CELCIUS 1

// LCD with SR Display
const uint8_t LCD_CLK_PIN = 9;  // SR clock pin
const uint8_t LCD_DAT_PIN = 8;  // SR data pin

// Rotary Encoder definitions
const uint8_t RE_MAIN_PIN = 4;  // RE main pin (right)
const uint8_t RE_SECD_PIN = 3;  // RE second pin (left)
const uint8_t RE_SWITCH_PIN = 2;// RE push switch pin

// Soldering iron control and temperature sensor
const uint8_t sensorPIN = A0; // Temperature sensor in soldering iron
const uint8_t heaterPIN = 10; // Power control PWM pin for soldering iron heater

// Temperature Parameters for control and user interface limits (in degrees)
#define C2F(t) ((t*9 + 32*5 + 2)/5) // convert degC to degF

const uint16_t TEMP_MIN_C = 180;    // Minimum temperature calibrated in celsius
const uint16_t TEMP_MAX_C = 400;    // Maximum temperature calibrated in celsius
#if USE_CELCIUS
const uint16_t TEMP_MIN_DEG = TEMP_MIN_C;
const uint16_t TEMP_MAX_DEG = TEMP_MAX_C;
#else
const uint16_t TEMP_MIN_DEG = C2F(TEMP_MIN_C);
const uint16_t TEMP_MAX_DEG = C2F(TEMP_MAX_C);
#endif

// Control parameters maximum range for sanity checking (internal units)
const uint16_t TEMP_CTL_MIN = 100;
const uint16_t TEMP_CTL_MAX = 800;
const uint16_t TEMP_CTL_DEFAULT = 470;

const uint16_t TEMP_COLD = 280; // threshold for cold 

// Power setting parameters
const uint8_t MAX_POWER = 250;  // maximum power allowed to the iron
const uint8_t MIN_POWER = 10;   // minimum power allowed to the iron


