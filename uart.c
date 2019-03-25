/*
 * uart.c
 *
 *  Created on: Nov 14, 2018
 *      Author: Kibum Kwon
 */
#include <stdint.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);  // waiting UART Receive FIFO Empty bit is set, which means waiting the FIFO empty.
    return UART0_DR_R & 0xFF;           // bit masking to get only 16bits data cuz 17~32th bits are garbage values.
}


