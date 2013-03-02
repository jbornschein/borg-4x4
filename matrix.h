

#include <avr/io.h>
#include <stdint.h>
#ifndef MATRIX_H
#define MATRIX_H

#include "config.h"
#include "util.h"

extern uint8_t bitplane[MATRIX_NPLANES][MATRIX_ROWS];  // valid for COLS <= 8

/*****************************************************************************
 * Set individual pixel
 */
void set_pixel16(uint8_t x, uint8_t y, uint16_t raw_val);
void set_pixel8(uint8_t x, uint8_t y, uint8_t value);

/*****************************************************************************
 * Constant bitplane filling
 */

void matrix_fill16(uint16_t raw_val);
void matrix_fill8(uint8_t value);

/*****************************************************************************
 * Bit-blitting
 */

void matrix_blit8(uint8_t y, uint8_t x, uint8_t rows, uint8_t cols, 
                    uint8_t brighness, uint8_t *buffer);

/*****************************************************************************
 * Turning it on and off again
 */

void matrix_timer1_on();
void matrix_off();

void matrix_waitsync();


#endif // MATRIX_H
