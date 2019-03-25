/*
 * uart.h
 *
 *  Created on: Nov 14, 2018
 *      Author: Kibum Kwon
 */

#ifndef UART_H_
#define UART_H_

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c);
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str);
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0();

#endif /* UART_H_ */
