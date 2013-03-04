#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#include "matrix.h"
#include "util.h"
#include "gamma_table.h"
#include "rotary.h"

uint8_t shl8_table[] = {
    0x01, 0x02, 0x04, 0x08, 
    0x10, 0x20, 0x40, 0x80,
};

uint16_t shl16_table[] = {
    0x0001, 0x0002, 0x0004, 0x0008, 
    0x0010, 0x0020, 0x0040, 0x0080,
    0x0100, 0x0200, 0x0400, 0x0800,
    0x1000, 0x2000, 0x4000, 0x8000
};

/*****************************************************************************
 * PWM Variables
 */
volatile int8_t matrix_syncbit = 0;

uint8_t pwm_phase = 0;        // current PWM phase
uint8_t pwm_row = 0;          // current row

uint8_t bitplane[MATRIX_NPLANES][MATRIX_ROWS];  // valid for COLS <= 8


#define ISR_LATENCY 40

/*****************************************************************************
 * Set individual pixel
 */
void set_pixel16(uint8_t x, uint8_t y, uint16_t raw_val)
{
    uint8_t bit_mask = 0x01 << x;

    uint16_t test_mask = 0x0001;
    for(uint8_t plane=0; plane < MATRIX_NPLANES; plane++, test_mask<<=1) {
        if (raw_val & test_mask) {
            bitplane[plane][y] |= bit_mask;
        } else {
            bitplane[plane][y] &= ~bit_mask;
        }
    }
}

void set_pixel8(uint8_t x, uint8_t y, uint8_t value)
{
    uint16_t raw_value = gamma_table[value];

    set_pixel16(x, y, raw_value);
}

/*****************************************************************************
 * Constant bitplane filling
 */

/**
 * Fill whole screen with raw value
 */
void matrix_fill16(uint16_t raw_val)
{
    uint16_t test_mask = 0x0001;
    for(uint8_t plane=0; plane < MATRIX_NPLANES; plane++, test_mask<<=1) {
        if (raw_val & test_mask) {
            for(uint8_t row=0; row < MATRIX_ROWS; row++)
                bitplane[plane][row] = 0x0f;
        } else {
            for(uint8_t row=0; row < MATRIX_ROWS; row++)
                bitplane[plane][row] = 0x00;
        }
    }
}

/**
 * Fill whole screen with 8 bit gamma corrected value
 */
void matrix_fill8(uint8_t value)
{
    uint16_t raw_value = gamma_table[value];
    matrix_fill16(raw_value);
}

/*****************************************************************************
 * Bit-blit
 */

void matrix_blit8(uint8_t y, uint8_t x, uint8_t rows, uint8_t cols, 
                    uint8_t brighness, uint8_t *buffer)
{
    uint8_t cur_x = x;
    uint8_t cur_y = y;
    uint8_t *ptr = buffer;

    for(uint8_t r=0; r<rows; r++, cur_y++) {
        for(uint8_t c=0; c<rows; c++, cur_x++) {
            set_pixel8(cur_y, cur_x, *ptr);
            ptr++;
        }
    }
}

/*****************************************************************************
 * Turning it on and off again
 */

void matrix_timer1_on()
{
    // Configure I/Os
	DDRD = 0xff;
    PORTD = PIN_DEFAULT;

    // COnfigure timer interrupt
    TCCR1A = 0x00;    // CTC
    TCCR1B = 0x09;    // CTC; clk/1

    TCNT1 = 0;        // set counter
    OCR1A = 0x00ff;   // compare A

    TIMSK |= (1 << OCIE1A);

    wdt_enable(0x00); // 17ms watchdog
}

void matrix_waitsync()
{
    int8_t old_sync = matrix_syncbit;

    while(old_sync == matrix_syncbit);
}

/*****************************************************************************
 * PWM Interrupt
 */

ISR(TIMER1_COMPA_vect)
{
    PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN1);
    wdt_reset();

    if (pwm_phase < MATRIX_FAST_SLOTS) {
        PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN2);
        PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN2);

        // Do this for all the rows
        for (pwm_row=0; pwm_row < MATRIX_ROWS; pwm_row++) {
            // Activate proper row
            uint8_t port = shl8_table[pwm_row+4];
            PORTD = port;

            // Precalc first phases
            uint8_t port_cyc[MATRIX_FAST_SLOTS];   

            for(uint8_t i=0; i<MATRIX_FAST_SLOTS; i++) {
                port_cyc[i] = port | bitplane[i][pwm_row];
            }

            uint8_t port_cyc0 = port_cyc[0];
            uint8_t port_cyc1 = port_cyc[1];
            uint8_t port_cyc2 = port_cyc[2];

            // phase == 0
            PORTD = port_cyc0;
    
            // phase == 1         (1 cycle)
            PORTD = port_cyc1;

            // phase == 2         (2 cycles)
            __builtin_avr_delay_cycles(1);
            PORTD = port_cyc2;

            // phase == 3         (4 cycles)
            __builtin_avr_delay_cycles(4-3);
            PORTD = port_cyc[3];

            // phase == 4
            __builtin_avr_delay_cycles(8-3);
            PORTD = port_cyc[4];

            // phase == 5
            __builtin_avr_delay_cycles(16-3);
            PORTD = port_cyc[5];

            // phase == 6
            __builtin_avr_delay_cycles(32-3);

#if MATRIX_FAST_SLOTS > 6
            PORTD = port_cyc[6];

            // phase == 7
            __builtin_avr_delay_cycles(64-3);
#endif

#if MATRIX_FAST_SLOTS > 7
            PORTD = port_cyc[7];

            // phase == 8
            __builtin_avr_delay_cycles(128-3);
#endif

            PORTD = 0x00;    // deactivate everything
        }

        // Prepare final output for FAST_SLOTS
        pwm_phase = MATRIX_FAST_SLOTS;
        pwm_row = 0;

        // Test whether to activate columns
        uint8_t port = shl8_table[pwm_row+4];
        port |= bitplane[pwm_phase][pwm_row];

        // Apply and Activate!
        PORTD = port;
        TCNT1 = ISR_LATENCY;
        OCR1A = shl16_table[pwm_phase];
            
        // prepare for next slot
        pwm_row++;
    } else {
        // Disable columns and actually switch to next row
        uint8_t port = shl8_table[pwm_row+4];
        //__builtin_avr_delay_cycles(10);
        PORTD = port;
        //__builtin_avr_delay_cycles(10);

        port |= bitplane[pwm_phase][pwm_row];

        // Apply and Activate!
        PORTD = port;
        TCNT1 = ISR_LATENCY;
        OCR1A = shl16_table[pwm_phase];

        // cycle current row
        pwm_row++;

        // Are we entering a new pwm_phase?
        if (pwm_row >=  MATRIX_ROWS) {
            pwm_row = 0;
            pwm_phase++;

            if (pwm_phase == MATRIX_NPLANES) {
                pwm_phase = 0;
                //pwm_phase = MATRIX_FAST_SLOTS;
                //PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN2);
                //PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN2);
                matrix_syncbit = 1 - matrix_syncbit;
            }
        }
    }

    // Read rotary encoder
    rotary_tick();

    PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN1);
}


