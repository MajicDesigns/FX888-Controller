#pragma once

#include <LiquidCrystal_SR.h>
#include "Debug.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

// BarGraph ----------------------------------------------
// Draw a bargraph on the LCD display
class BarGraph
{
protected:
  const uint8_t COL_PER_CHAR = 5;
  const uint8_t ROW_PER_CHAR = 8;
  const uint8_t LCD_BLOCK_CHAR = 0xff;
  const uint8_t LCD_BLANK_CHAR = ' ';

  LiquidCrystal_SR *_lcd;
  uint8_t   _row, _colStart;
  int8_t    _colLen;
  uint32_t  _valueMin, _valueMax;
  uint8_t   _charLCD;

public:
  BarGraph(LiquidCrystal_SR *lcd, uint8_t row, uint8_t colStart, int8_t colLen) :
    _lcd(lcd), _row(row), _colStart(colStart), _colLen(colLen), _charLCD(0)
  {
    setRange(0, 100);
  };

  void begin(void) {};

  inline void setChar(uint8_t charLCD) { if (_charLCD < 8) _charLCD = charLCD; };
  inline void setRange(uint32_t valueMin, uint32_t valueMax) { _valueMin = valueMin; _valueMax = valueMax; };

  void show(uint32_t value)
    // Build a string with the graph characters and then display the string.
    // Assume the graph is being display L to R and then reverse the string
    // if it is to be displyed R to L. Just need to ensure that the end 
    // character of the bar graph is built in the correct direction.
  {
    uint8_t charMap[ROW_PER_CHAR];  // LCD user defined character
    char *szGraph = (char *)malloc((abs(_colLen) + 1) * sizeof(char));
    uint8_t lenGraph;               // size of the graph in pixel columns
    uint8_t barValue;
    uint8_t c;

    // Can't do much if we couldn't get RAM
    if (szGraph == NULL) return;

    // work out what value display means
    lenGraph = abs(_colLen) * COL_PER_CHAR;
    if (value > _valueMax) barValue = lenGraph;
    else if (value < _valueMin) barValue = 0;
    else barValue = map(value, _valueMin, _valueMax, 0, lenGraph);

    // create the correct orientation for the user defined character
    // create the first row, then copy it to the others
    c = barValue % COL_PER_CHAR;    // number of extra columns
    charMap[0] = 0;
    for (uint8_t i = 0; i<c; i++)
      bitSet(charMap[0], (_colLen < 0) ? i : COL_PER_CHAR - i);
    memset(&charMap[1], charMap[0], sizeof(charMap) - sizeof(charMap[0]));
    _lcd->createChar(_charLCD, charMap);

    // create the graph string as if it was being displayed L to R
    {
      uint8_t i;  // retain loop index within the code block

      c = barValue / COL_PER_CHAR;    // number of whole characters
      for (i = 0; i<c; i++)       // full block characters
        szGraph[i] = LCD_BLOCK_CHAR;
      szGraph[i++] = _charLCD;  // end of graph special character
      for (; i<abs(_colLen); i++)
        szGraph[i] = LCD_BLANK_CHAR;  // blanks where no blocks
      szGraph[abs(_colLen)] = '\0';   // end the string properly
    }

    // prepare to display
    if (_colLen > 0)
    {
      // just set the cursor at specified position - string is already correct
      _lcd->setCursor(_colStart, _row);
    }
    else
    {
      // need to reverse the string and set a different starting cursor position
      strrev(szGraph);
      _lcd->setCursor(_colStart + _colLen + 1, _row);
    }

    // display it and release the string memory
    _lcd->print(szGraph);
    free(szGraph);
  }
};
