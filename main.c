#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <avr/interrupt.h>

#include "util.h"
#include "matrix.h"
#include "rotary.h"

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


/**
 * Main
 */
int main()
{
    PORTB = 0x00;
	DDRB = 0xff;

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

    matrix_fill8(0);

    // Activate timer
    matrix_on();
	sei();

    wait(500);
    //ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
    //ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
    //wait(500);

    const uint16_t idle_timeout = 6000;
    uint16_t idle = idle_timeout - 200;

    int8_t  fade = 1;
    int16_t brightness = 0;
    int16_t new_brightness = brightness;
    matrix_fill8(brightness);

	while(1){
        wait(10);

        int8_t change = rotary_read2();
        if (change) {
            idle = 0;
            new_brightness += change;
        } else {
            if (idle > idle_timeout) {
                new_brightness += fade;
            } else {
                idle++;
            }
        }

        if (new_brightness < 0) {
            new_brightness = 0;
            fade = +1;
        };
        if (new_brightness > 0xff) {
            new_brightness = 0xff;
            fade = -1;
        }
        if (new_brightness != brightness) {
            brightness = new_brightness;

            matrix_waitsync();
            matrix_fill8((uint8_t)brightness);
        }
        /*
        for(uint8_t j=0; j<1000; j++) {
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
            PIN_TOGGLE(DEBUG_PORT, DEBUG_PIN3);
        }
        */


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
