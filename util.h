#ifndef UTIL_H
#define UTIL_H

#include "config.h"

/**
 * Utility Library
 *
 * #define F_CPU 16000000
 */

#ifndef F_CPU
  #define F_CPU 16000000
#endif

void wait(int ms);

#define PIN_ON(var, pin)  ((var) |=  (1 << (pin)))
#define PIN_OFF(var, pin) ((var) &= ~(1 << (pin)))
#define PIN_TOGGLE(var, pin) ((var) ^= (1 << (pin)))

#endif
