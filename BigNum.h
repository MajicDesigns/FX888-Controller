#pragma once

#include <LiquidCrystal_SR.h>
#include "Debug.h"

#define ORIGINAL_FONT 0

// BigNum -----------------------------------------------
// Draw a large character over 2 lines on the LCD display

class BigNum
{
private:
#if ORIGINAL_FONT
  // Definitions for the number of characters and their size
  static const uint8_t NUM_ELEMENTS = 8; // elements required
  static const uint8_t ELEMENT_SIZE = 8; // bytes per element

  static const PROGMEM uint8_t fontSingle[NUM_ELEMENTS][ELEMENT_SIZE];
  static const PROGMEM uint8_t fontDouble[NUM_ELEMENTS][ELEMENT_SIZE];
#else
  // Definitions for the number of characters and their size
  static const uint8_t NUM_ELEMENTS = 7; // elements required
  static const uint8_t ELEMENT_SIZE = 8; // bytes per element

  static const PROGMEM uint8_t fontSingle[NUM_ELEMENTS][ELEMENT_SIZE];
#endif


protected:
  LiquidCrystal_SR  *_lcd;

public:
  BigNum(LiquidCrystal_SR *lcd) : _lcd(lcd)
  { };

  void begin(void)
  {
    setSingleFont();
  }

  void setSingleFont(void)
  {
    uint8_t c[NUM_ELEMENTS];

    for (uint8_t i = 0; i < NUM_ELEMENTS; i++)
    {
      memcpy_P(c, &fontSingle[i][0], ELEMENT_SIZE*sizeof(fontSingle[0][0]));
      _lcd->createChar(i, c);
    }
  };

  void setDoubleFont(void)
  {
    uint8_t c[NUM_ELEMENTS];

    for (uint8_t i = 0; i < NUM_ELEMENTS; i++)
    {
#if ORIGINAL_FONT
      memcpy_P(c, &fontDouble[i][0], ELEMENT_SIZE*sizeof(fontDouble[0][0]));
#else
      memcpy_P(c, &fontSingle[i][0], ELEMENT_SIZE*sizeof(fontSingle[0][0]));
#endif
      _lcd->createChar(i, c);
    }
  };

  void writeNumber(uint8_t row, uint8_t col, uint16_t num, uint8_t digits, bool leadZero = false)
    // write number num at top left coordinate (row, col) in a space 3 digits in size 
    // with leading zero if requested
  {
    for (int8_t i = digits; i > 0; i--)
    {
      uint8_t d = num % 10;

      // draw the digit depeding on zero or not and, if zero, position in the number or not
      if ((d != 0) || leadZero || (d == 0 && i == digits) || (d == 0 && num != 0))
        writeDigit(row, col + i - 1, d);
      else
      {
        _lcd->setCursor(col, row);
        _lcd->write(' ');
        _lcd->setCursor(col, row + 1);
        _lcd->write(' ');
      }

      num /= 10;
    }
  }

  void writeDigit(uint8_t row, uint8_t col, uint8_t digit)
  {
    // Define the elements for the top and bottom for each digit
#if ORIGINAL_FONT
    static const uint8_t top[10] = { 1, 0, 7, 7, 3, 4, 4, 7, 1, 1 };
    static const uint8_t bot[10] = { 3, 0, 4, 2, 0, 6, 3, 0, 5, 2 };
#else
    static const uint8_t top[10] = { 0, 5, 2, 2, 1, 3, 3, 6, 4, 4 };
    static const uint8_t bot[10] = { 1, 5, 3, 2, 6, 2, 4, 5, 4, 2 };
#endif // ORIGINAL_FONT

    if (digit > 9) return;

    _lcd->setCursor(col, row);
    _lcd->write((char)top[digit]);
    _lcd->setCursor(col, row + 1);
    _lcd->write((char)bot[digit]);
  };
};

// PROGMEM Font data for big numbers
//
#if ORIGINAL_FONT
// Character elements defined in the user chars in numerical order
// 0:    1: _  2: _  3:    4: _  5: _  6:    7: _
//     |   | |    _|   |_|   |_    |_|    _|     |

// Two types of fonts are defined - single and double stroke widths
// Single width strokes
const PROGMEM uint8_t BigNum::fontSingle[NUM_ELEMENTS][ELEMENT_SIZE] =
{
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }, // 0
  { 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }, // 1
  { 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1f }, // 2
  { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f }, // 3
  { 0x1f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f }, // 4
  { 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f }, // 5
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1f }, // 6
  { 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }  // 7
};

const PROGMEM uint8_t BigNum::fontDouble[NUM_ELEMENTS][ELEMENT_SIZE] =
{
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }, // 0
  { 0x1f, 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }, // 1
  { 0x1f, 0x1f, 0x01, 0x01, 0x01, 0x01, 0x1f, 0x1f }, // 2
  { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f, 0x1f }, // 3
  { 0x1f, 0x1f, 0x10, 0x10, 0x10, 0x10, 0x1f, 0x1f }, // 4
  { 0x1f, 0x1f, 0x11, 0x11, 0x11, 0x11, 0x1f, 0x1f }, // 5
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0f, 0x1f }, // 6
  { 0x1f, 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }  // 7
};
#else
// Character elements defined in the user chars in numerical order
// 0: _  1:    2: _  3: _  4: _  5:    6: _  7: not used
//   | |   |_|    _|   |_    |_|     |     |    

// Only one font is defined but it takes on less user defined character
// Single width strokes
const PROGMEM uint8_t BigNum::fontSingle[NUM_ELEMENTS][ELEMENT_SIZE] =
{
  { 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }, // 0
  { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f }, // 1
  { 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1f }, // 2
  { 0x1f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f }, // 3
  { 0x1f, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1f }, // 4
  { 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }, // 5
  { 0x1f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01 }, // 6
};
#endif
