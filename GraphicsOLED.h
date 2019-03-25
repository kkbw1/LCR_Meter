/*
 * GraphicsOLED.h
 *
 *  Created on: Oct 27, 2018
 *      Author: Kibum Kwon
 */

#ifndef GRAPHICSOLED_H_
#define GRAPHICSOLED_H_

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

// Set pins
#define A0           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))  // PB6
#define CS_NOT       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4)))  // PB3
#define RES          (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))  // PB2

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void sendGraphicsLcdCommand(uint8_t command);
void sendGraphicsLcdData(uint8_t data);
void setGraphicsLcdPage(uint8_t page);
void setGraphicsLcdColumn(uint8_t x);
void setGraphicsLcdColumnAndPage(uint8_t x, uint8_t page);
void refreshGraphicsLcd();
void clearGraphicsLcd();
void drawGraphicsLcdPixel(uint8_t x, uint8_t y, uint8_t op);
void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, uint8_t op);
void setGraphicsLcdTextPosition(uint8_t x, uint8_t page);
void putcGraphicsLcd(char c);
void putsGraphicsLcd(char str[]);
void writeCharGraphicsLcd(uint8_t x, uint8_t page, char ch);
void writeStrGraphicsLcd(uint8_t x, uint8_t page, char str[]);
void writeUNum1000GraphicsLcd(uint8_t x, uint8_t page, uint16_t num);
void writeSNum1000GraphicsLcd(uint8_t x, uint8_t page, int16_t snum);
void initGraphicsLcd();

#endif /* GRAPHICSOLED_H_ */
