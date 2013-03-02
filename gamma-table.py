#!/usr/bin/env python 

from __future__ import division

import numpy as np

levels = 256
pwm_slots = 14
gamma = 3.3
cpu_freq = 8e6


max_brightness = 2**(pwm_slots)-1
min_brightness = 1

x = np.linspace(0, 1., levels)

y = x**gamma    # gamma correct

# Rescale gamma curve 
scale = (max_brightness - min_brightness) / (y[-1] - y[1])
shift = min_brightness - y[1]*scale
y[1:] = scale*y[1:] + shift

update_freq = cpu_freq / (2**(pwm_slots+1)-1)

print "#include <avr/io.h>"
print "#include <stdlib.h>"
print
print "#include \"config.h\""
print
print "#if MATRIX_NPLANES != %d" % pwm_slots
print "#  error PWM_SLOTS and MATRIX_NPLANES do not agree:"
print "#endif"
print
print "/***"
print " * Gamma table  (%d levels, gamma %.1f)" % (levels, gamma)
print " *"
print " * Framerate when using this table (assuming no row muxing and %d MHz CPU clock): %3.1f" % (cpu_freq // 1e6, update_freq)
print " */"
print "uint16_t gamma_table[%d] = {" % levels,
for i in xrange(levels):
    if i % 8 == 0:
        print "\n    ",
    print "%5d," % int(y[i]),
print "\n};"

