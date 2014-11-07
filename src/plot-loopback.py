#!/usr/bin/env python

import scipy as sp
import matplotlib.pyplot as plt
from cmath import phase, sqrt
from math import pi

data = sp.fromfile(open("/tmp/sink"), dtype=sp.complex64)[1000000:1100000]

real = [y.real for y in data]
imag = [y.imag for y in data]
X = [float(n)/1e6 for n in range(len(real))]

plt.figure(1)
plt.title("Plot of received waveform")
plt.plot(X, real, 'r', label="Real")
plt.plot(X, imag, 'g', label="Imag")
plt.xlabel("Time")
plt.legend(loc=4)
plt.grid(True, which="both")
plt.show()
