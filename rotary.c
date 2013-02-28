#include <avr/io.h>
#include <stdint.h>

#include "rotary.h"
#include "config.h"

volatile int8_t rotary_lock = 0;
volatile int8_t rotary_delta;          // -128 ... 127

volatile int8_t rotary_last;

void rotary_init()
{
    int8_t new;

    new = 0;
    if( ROTARY_A )
        new = 3;
    if( ROTARY_B )
        new ^= 1;                   // convert gray to binary
    rotary_last = new;                   // power on state
    rotary_delta = 0;
}


void rotary_tick()
{
    int8_t new, diff;
      
    if (rotary_lock)
        return;

    new = 0;
    if( ROTARY_A )
        new = 3;
    if( ROTARY_B )
        new ^= 1;                       // convert gray to binary
    diff = rotary_last - new;           // difference last - new
    rotary_last = new;                  // store new as next last
    if( diff & 1 ) {                    // bit 0 = value (1)
        rotary_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
        // rotary_delta += 1;
    }
}

int8_t rotary_read1()
{
    int8_t val;
       
    rotary_lock = 1;
    val = rotary_delta;
    rotary_delta = 0;
    rotary_lock = 0;
    return val;
}

int8_t rotary_read2()
{
    int8_t val;
       
    rotary_lock = 1;
    val = rotary_delta;
    rotary_delta = val & 1;
    rotary_lock = 0;
    return val >> 1;
}

