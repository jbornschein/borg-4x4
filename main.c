#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <avr/interrupt.h>

#include "util.h"
#include "rotary.h"
#include "gamma_table.h"

#define ROWS 4
#define COLS 4

#define PWM_FAST_SLOTS (8)

#define PIN_DEBUG1 0
#define PIN_DEBUG2 1
#define PIN_DEBUG3 2

#define PIN_ON(var, pin) ((var) &= ~(1 << (pin)))
#define PIN_TOGGLE(var, pin) ((var) ^= (1 << (pin)))

#define PIN_DEFAULT (0x00)

// Brightness to be displayed
uint8_t leds[ROWS][COLS];

/**
 * PWM stuff
 */
uint8_t pwm_phase = 15;       // current PWM phase
uint8_t pwm_row = 0;          // current row

uint8_t bitplane[PWM_SLOTS][ROWS];  // valid for COLS <= 8

void bitplanes_update()
{
    for(uint8_t plane=0; plane<PWM_SLOTS; plane++) {
        uint16_t test_mask = 0x0001 << plane;
        for(uint8_t row=0; row<ROWS; row++) {
            uint8_t val = 0x00;
            for(uint8_t col=0; col<COLS; col++) {
                val <<= 1;
                if (gamma_table[leds[row][col]] & test_mask)
                    val |= 0x01;
            }
            bitplane[plane][row] = val;
        }
    }
}



SIGNAL(SIG_OUTPUT_COMPARE1A)
{
    PIN_TOGGLE(PORTB, PIN_DEBUG1);

    if (pwm_phase >= PWM_SLOTS) {
        PIN_TOGGLE(PORTB, PIN_DEBUG2);
        PIN_TOGGLE(PORTB, PIN_DEBUG2);

        // Do this for all the rows
        for (pwm_row=0; pwm_row<ROWS; pwm_row++) {
            // Activate proper row
            uint8_t port = 0x10 << pwm_row; 
            PORTD = port;

            // Precalc first phases
            uint8_t port_cyc[PWM_FAST_SLOTS];   

            for(uint8_t i=0; i<PWM_FAST_SLOTS; i++) {
                port_cyc[i] = port | bitplane[i][pwm_row];
            }

            uint8_t port_cyc0 = port_cyc[0];
            uint8_t port_cyc1 = port_cyc[1];

            // phase == 0
            PORTD = port_cyc0;
    
            // phase == 1
            PORTD = port_cyc1;

            // phase == 2
            PORTD = port_cyc[2];

            // phase == 3
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
            PORTD = port_cyc[6];

            // phase == 7
            __builtin_avr_delay_cycles(64-3);
            PORTD = port_cyc[7];

            // phase == 8
            __builtin_avr_delay_cycles(128-3);
            PORTD = 0x00;    // deactivate everything
        }

        // Prepare final output for FAST_SLOTS
        pwm_phase = PWM_FAST_SLOTS;
        pwm_row = 0;

        // Test whether to activate columns
        uint8_t port = 0x10 << pwm_row;
        port |= bitplane[pwm_phase][pwm_row];

        // Apply and Activate!
        PORTD = port;
        TCNT1 = 4;
        OCR1A = 0x01 << pwm_phase;
            
        // prepare for next slot
        pwm_row++;

        // Read out rotary encoder
    } else {
        // Disable columns and actually switch to next row
        uint8_t port = 0x10 << pwm_row;
        PORTD = port;

        port |= bitplane[pwm_phase][pwm_row];

        // Apply and Activate!
        PORTD = port;
        TCNT1 = 4;
        OCR1A = 0x01 << pwm_phase;

        // cycle current row
        pwm_row++;

        // Are we entering a new pwm_phase?
        if (pwm_row >=  ROWS) {
            pwm_row = 0;
            pwm_phase++;
        }
    }

    // Read rotary encoder
    rotary_tick();

    PIN_TOGGLE(PORTB, PIN_DEBUG1);
}

/**
 * Lenarize a logarithmic poti
 */
/*
uint16_t linearize_poti(uint16_t adc)
{
    static float p2 = 8.70075617e-06;
    static float p1 = 4.56448472e-01;

    float v = (p2*adc*adc + p1*adc);
    v = fminf(v, (float)0xffff);
    v = fmaxf(v, (float)0x0000);

    return (uint16_t)v;
}
*/

void timer1_on()
{
    TCCR1A = 0x00;    // CTC
    TCCR1B = 0x09;    // CTC; clk/1

    TCNT1 = 0;        // set counter
    OCR1A = 0x00ff;   // compare A

    TIMSK |= (1 << OCIE1A);
}


/**
 * Main
 */
int main()
{
    // Digital I/O
    PORTD = PIN_DEFAULT;
    PORTB = 0x00;

	DDRB = 0xff;
	DDRD = 0xff;

    PORTC = 0x03;
    DDRC = 0x00;

    rotary_init();
    
    // Setup ADC
    /* ADMUX Register:
     *   REFS1 REFS0 ADLAR MUX4-MUX0
     * 
     *  REFS:
     *  0 0     Aref Pin
     *  0 1     AVcc
         *  1 0     Reserved
     *  1 1     Internal 2.56V Reference
     * 
     * MUX = 0-7: Single ended Input on ADC0-ADC7
     */
    //ADMUX = 0x20;
  
    /* ADCSRA Register:
     *   ADEN ADSC ADATE ADIF ADIE ADPS2-ADPS0
     */
    //ADCSRA = (1<<ADEN) | 7; //ADC aktivieren, Takt/128

    // Initial brightness
    for(uint8_t row=0; row<ROWS; row++) {
        for(uint8_t col=0; col<COLS; col++) {
            leds[row][col] = 255;
        }
    }
    bitplanes_update();

    // Activate timer
    timer1_on();

	sei();

    //wait(500);
    //ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
    //ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
    //wait(500);

    uint16_t i = 0;
	while(1){
        i++;

        int8_t change = rotary_read2();
        if (change) {
            for(uint8_t row=0; row<ROWS; row++) {
                for(uint8_t col=0; col<COLS; col++) {
                        leds[row][col] += change;
                }
            }
            bitplanes_update();
        }

        for(uint8_t j=0; j<16; j++) {
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
            PIN_TOGGLE(PORTB, PIN_DEBUG3);
        }



        /*
        rotary_tick();
        
        // Read rotary 
        int16_t new_brightness = (int16_t)brightness + 2*(int16_t)rotary_read2();
        if (new_brightness < 0) new_brightness = 0;
        if (new_brightness > 0xff) new_brightness = 0xff;

        brightness = new_brightness;

        //brightness += 2*rotary_read2();

        //PORTD = (rotary_delta &0x0f) << 4;
        if ( (ADCSRA & (1<<ADIF)) ) {
            val = 1.0*linearize_poti(ADC) / 0xffff;

            ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
            ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
        }
        */

	}
}
