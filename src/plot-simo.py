#!/usr/bin/env python

import scipy as sp
import matplotlib.pyplot as plt
from cmath import phase, sqrt
from math import pi

data1 = sp.fromfile(open("/tmp/sink1"), dtype=sp.complex64)[1000000:1010000]
data2 = sp.fromfile(open("/tmp/sink2"), dtype=sp.complex64)[1000000:1010000]


real1 = [y.real for y in data1]
imag1 = [y.imag for y in data1]
real2 = [y.real for y in data2]
imag2 = [y.imag for y in data2]
phase1 = [phase(y) for y in data1]
phase2 = [phase(y) for y in data2]
X = [float(n)/1e6 for n in range(len(real1))]
phase_diff = [phase1[n] - phase2[n] for n in range(len(data1))]
for n in range(len(phase_diff)):
    while(phase_diff[n] < -1*pi):
        phase_diff[n] += 2*pi
    while(phase_diff[n] > pi):
        phase_diff[n] -= 2*pi

plt.figure(1)
plt.title("Plot of received waveform")
plt.plot(X,real1, 'r', label="USRP 1-Real")
plt.plot(X, real2, 'b', label="USRP 2-Real")
plt.plot(X, imag1, 'g', label="USRP 1-Imag")
plt.plot(X, imag2, 'y', label="USRP 2-Imag")
plt.xlabel("Time")
plt.legend(loc=4)
plt.grid(True, which="both")

plt.figure(3)
plt.subplot(211)
plt.title("Plot of of phase")
plt.plot(X, phase1, 'r', label="USRP 1")
plt.plot(X, phase2, 'b', label="USRP 2")
plt.legend(loc=4)
plt.grid(True, which="both")
plt.subplot(212)
plt.title("Plot of phase difference")
plt.plot(X, phase_diff, 'y')
plt.grid(True, which="both")

plt.show()

