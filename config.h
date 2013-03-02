#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <avr/io.h>

// Debug pins
#define DEBUG_PORT PORTB
#define DEBUG_PIN1 0
#define DEBUG_PIN2 1
#define DEBUG_PIN3 2

// Matrix defines
#define MATRIX_ROWS 4
#define MATRIX_COLS 4
#define MATRIX_NPLANES 14
#define MATRIX_FAST_SLOTS 8
#define PIN_DEFAULT (0x00)


//c uart.[ch] defines
#define UART_INTERRUPT 1
#define UART_BAUD_RATE 4800
#define UART_RXBUFSIZE 16
#define UART_TXBUFSIZE 16
#define UART_LINE_BUFFER_SIZE 40
//#define UART_LEDS 1

#define ROTARY_A     (PINC & 0x02)
#define ROTARY_B     (PINC & 0x01)

#endif
