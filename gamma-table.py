#!/usr/bin/env python 


import numpy as np

steps = 128
gamma = 2.0
cpu_hz = 8e6

x = np.linspace(0, 1., steps)

y = x**gamma    # gamma correct
y = y / y[1]    # renormalize in terms of CPU cycles

highest_bit = np.ceil(np.log2(y[-1]))
update_freq = cpu_hz / (2**(highest_bit+1)-1)

print "#include <avr/io.h>"
print "#include <stdlib.h>"
print
print "/***"
print " * Gamma table  (%d levels, gamma %.1f)" % (steps, gamma)
print " *"
print " * Framerate when using this table (assuming no row muxing and 8MHz CPU clock): %3.1f" % update_freq
print " */"
print "uint8_t  pwm_slots = %d;" % highest_bit
print "uint16_t gamma_table[%d] = {" % steps,
for i in xrange(steps):
    if i % 8 == 0:
        print "\n    ",
    print "%5d," % int(y[i]),
print "\n};"

