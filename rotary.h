#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>

extern volatile int8_t rotary_delta;

/**
 * Call this regulary, e.. in a timer ISR
 */
void rotary_tick(); 

/**
 * Get delta values since last call
 */
int8_t rotary_read1();
int8_t rotary_read2();

#endif
