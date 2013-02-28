#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "util.h"

uint16_t linearize_poti(uint16_t adc)
{
    static float p2 = 8.70075617e-06;
    static float p1 = 4.56448472e-01;

    float v = (p2*adc*adc + p1*adc);
    v = fminf(v, (float)0xffff);
    v = fmaxf(v, (float)0x0000);

    return (uint16_t)v;
}

const uint16_t dR[256] = {
         0,   1,   3,   4,   6,   8,  10,  12,  14,  16,  18,  20,  22,
        24,  27,  29,  31,  34,  36,  38,  41,  43,  46,  48,  51,  53,
        56,  58,  61,  63,  66,  69,  71,  74,  77,  79,  82,  85,  87,
        90,  93,  96,  98, 101, 104, 107, 110, 113, 115, 118, 121, 124,
       127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 163,
       166, 169, 172, 175, 178, 181, 184, 187, 190, 194, 197, 200, 203,
       206, 209, 212, 216, 219, 222, 225, 228, 232, 235, 238, 241, 245,
       248, 251, 254, 258, 261, 264, 268, 271, 274, 277, 281, 284, 287,
       291, 294, 297, 301, 304, 308, 311, 314, 318, 321, 325, 328, 331,
       335, 338, 342, 345, 349, 352, 355, 359, 362, 366, 369, 373, 376,
       380, 383, 387, 390, 394, 397, 401, 404, 408, 411, 415, 419, 422,
       426, 429, 433, 436, 440, 444, 447, 451, 454, 458, 462, 465, 469,
       472, 476, 480, 483, 487, 491, 494, 498, 502, 505, 509, 513, 516,
       520, 524, 527, 531, 535, 538, 542, 546, 550, 553, 557, 561, 564,
       568, 572, 576, 579, 583, 587, 591, 595, 598, 602, 606, 610, 613,
       617, 621, 625, 629, 632, 636, 640, 644, 648, 651, 655, 659, 663,
       667, 671, 674, 678, 682, 686, 690, 694, 698, 702, 705, 709, 713,
       717, 721, 725, 729, 733, 737, 740, 744, 748, 752, 756, 760, 764,
       768, 772, 776, 780, 784, 788, 792, 795, 799, 803, 807, 811, 815,
       819, 823, 827, 831, 835, 839, 843, 847
};


void timer1_on()
{
    TCCR1A = 0x00;    // CTC
    TCCR1B = 0x09;    // CTC; clk/1

    TCNT1 = 0;        // set counter
    OCR1A = 0x1fff;   // compare A

    TIMSK |= (1 << OCIE1A);
}

// Curret PWM slot
uint8_t phase = 0;
uint8_t row = 0;

// Color to be displayed
volatile uint8_t brightness;


#define PIN_DEBUG1 0
#define PIN_DEBUG2 1

#define PIN_ON(var, pin) ((var) &= ~(1 << (pin)))
#define PIN_TOGGLE(var, pin) ((var) ^= (1 << (pin)))

#define PIN_DEFAULT (0xff)

SIGNAL(SIG_OUTPUT_COMPARE1A)
{
    PIN_TOGGLE(PORTB, PIN_DEBUG1);

    // Anti volatile
    uint8_t brightness_   = brightness;

    // Disable everyting
    PORTD = 0x00;

    // Current row
    row = (row + 1) % 4;

    uint8_t port = 0x10 << row;

    // 
    if (phase < 16) {
        if (row == 0) {
            PIN_TOGGLE(PORTB, PIN_DEBUG2);
            PIN_TOGGLE(PORTB, PIN_DEBUG2);
        }

        // Precalc first phases
        uint8_t i;
        uint8_t port_cyc[16];   

        for(i=0; i<16; i++) {
            port_cyc[i] = port;
            if (brightness_ > i) port_cyc[i] |= 0x0f;
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
        __builtin_avr_delay_cycles(dR[2]-1);
        PORTD = port_cyc[3];

        // phase == 4
        __builtin_avr_delay_cycles(dR[3]-1);
        PORTD = port_cyc[4];

        // phase == 5
        __builtin_avr_delay_cycles(dR[4]-1);
        PORTD = port_cyc[5];

        // phase == 6
        __builtin_avr_delay_cycles(dR[5]-1);
        PORTD = port_cyc[6];

        // phase == 7
        __builtin_avr_delay_cycles(dR[6]-1);
        PORTD = port_cyc[7];

        // phase == 8
        __builtin_avr_delay_cycles(dR[7]-1);
        PORTD = port_cyc[8];

        // phase == 9
        __builtin_avr_delay_cycles(dR[8]-1);
        PORTD = port_cyc[9];

        // phase == 10
        __builtin_avr_delay_cycles(dR[9]-1);
        PORTD = port_cyc[10];

        // phase == 11
        __builtin_avr_delay_cycles(dR[10]-1);
        PORTD = port_cyc[11];

        // phase == 12
        __builtin_avr_delay_cycles(dR[11]-1);
        PORTD = port_cyc[12];

        // phase == 13
        __builtin_avr_delay_cycles(dR[12]-1);
        PORTD = port_cyc[13];

        // phase == 14
        __builtin_avr_delay_cycles(dR[13]-1);
        PORTD = port_cyc[14];

        // phase == 15
        __builtin_avr_delay_cycles(dR[14]-1);
        PORTD = port_cyc[15];

        phase = 15;

        TCNT1 = 4;
    } else {
        if (brightness_ > phase) port |= 0x0f;

        PORTD = port;
    }

    OCR1A = dR[phase];

    if (row == 3) {
        phase++;
        if (phase == 255)
            phase = 0;
    }

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
    ADMUX = 0x20;
  
    /* ADCSRA Register:
     *   ADEN ADSC ADATE ADIF ADIE ADPS2-ADPS0
     */
    ADCSRA = (1<<ADEN) | 7; //ADC aktivieren, Takt/128

    // Initialize color channels
    brightness = 128;

    // Activate timer
    timer1_on();

	sei();

    //wait(500);
    //ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
    //ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
    //wait(500);

	while(1){
        /*
        if ( (ADCSRA & (1<<ADIF)) ) {
            val = 1.0*linearize_poti(ADC) / 0xffff;

            ADCSRA |= (1<<ADIF);//Interrupt Flag zurücksetzen
            ADCSRA |= (1<<ADSC);//Einen Konvertierungsvorgang starten
        }
        */

	}
}
