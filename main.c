#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <avr/interrupt.h>

#include "util.h"
#include "rotary.h"
#include "gamma_table.h"


uint16_t linearize_poti(uint16_t adc)
{
    static float p2 = 8.70075617e-06;
    static float p1 = 4.56448472e-01;

    float v = (p2*adc*adc + p1*adc);
    v = fminf(v, (float)0xffff);
    v = fmaxf(v, (float)0x0000);

    return (uint16_t)v;
}

void timer1_on()
{
    TCCR1A = 0x00;    // CTC
    TCCR1B = 0x09;    // CTC; clk/1

    TCNT1 = 0;        // set counter
    OCR1A = 0x00ff;   // compare A

    TIMSK |= (1 << OCIE1A);
}

// Curret PWM slot
uint8_t pwm_phase = 15;
uint8_t pwm_row = 0;

// Color to be displayed
volatile uint8_t brightness;

#define PWM_FAST_SLOTS (8)
#define PWM_ROWS (4)

#define PIN_DEBUG1 0
#define PIN_DEBUG2 1
#define PIN_DEBUG3 2

#define PIN_ON(var, pin) ((var) &= ~(1 << (pin)))
#define PIN_TOGGLE(var, pin) ((var) ^= (1 << (pin)))

#define PIN_DEFAULT (0x00)

SIGNAL(SIG_OUTPUT_COMPARE1A)
{
    PIN_TOGGLE(PORTB, PIN_DEBUG1);

    // Anti volatile and gamma correct!
    uint16_t brightness_ = gamma_table[brightness/2];

    if (pwm_phase >= pwm_slots) {
        PIN_TOGGLE(PORTB, PIN_DEBUG2);
        PIN_TOGGLE(PORTB, PIN_DEBUG2);

        // Do this for all the rows
        for (pwm_row=0; pwm_row<PWM_ROWS; pwm_row++) {
            // Activate proper row
            uint8_t port = 0x10 << pwm_row; 
            PORTD = port;

            // Precalc first phases
            uint8_t port_cyc[PWM_FAST_SLOTS];   

            uint8_t test_bit = 0x01;           // 8 bit are sufficient because PWM_FAST_SLOTS < 8!
            for(uint8_t i=0; i<PWM_FAST_SLOTS; i++) {
                port_cyc[i] = port;
                if (brightness_ & test_bit) port_cyc[i] |= 0x0f;
                test_bit = test_bit << 1;
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
        uint8_t  port = 0x10 << pwm_row;
        uint16_t test_bit = 0x01 << pwm_phase; 
        if (brightness_ & test_bit) port |= 0x0f;

        // Apply and Activate!
        PORTD = port;
        TCNT1 = 4;
        OCR1A = 0x01 << pwm_phase;
            
        // prepare for next slot
        pwm_row = 1;

        // Read out rotary encoder
        rotary_tick();
    } else {
        // Disable columns and actually switch to next row
        uint8_t port = 0x10 << pwm_row;
        PORTD = port;

        // Test whether to activate columns
        uint16_t test_bit = 0x01 << pwm_phase;
        if (brightness_ & test_bit) port |= 0x0f;

        // Apply and Activate!
        PORTD = port;
        TCNT1 = 4;
        OCR1A = 0x01 << pwm_phase;

        // cycle current row
        pwm_row++;

        // Are we entering a new pwm_phase?
        if (pwm_row >=  PWM_ROWS) {
            pwm_row = 0;
            pwm_phase++;
        }
    }

    // Read rotary encoder
    // rotary_tick();

    PIN_TOGGLE(PORTB, PIN_DEBUG1);
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
    DDRC = 0xFF;

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
    brightness = 2;

    // Activate timer
    timer1_on();

	sei();

    //wait(500);
    //ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
    //ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
    //wait(500);

	while(1){
        rotary_tick();
        
        // Read rotary 
        int16_t new_brightness = (int16_t)brightness + 2*(int16_t)rotary_read2();
        if (new_brightness < 0) new_brightness = 0;
        if (new_brightness > 0xff) new_brightness = 0xff;

        brightness = new_brightness;

        //brightness += 2*rotary_read2();

        //PORTD = (rotary_delta &0x0f) << 4;
        /*
        if ( (ADCSRA & (1<<ADIF)) ) {
            val = 1.0*linearize_poti(ADC) / 0xffff;

            ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
            ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
        }
        */

	}
}
