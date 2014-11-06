#!/usr/bin/env python

import scipy as sp
import matplotlib.pyplot as plt
from cmath import phase, sqrt
from math import pi
import time

#def compute_channel(f1, f2, gain1, gain2):

freq = 10e3
amp = 0.25

f = open("logfile.csv", 'a')
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
_sum = 0.0
for n in range(len(data1)):
    _sum += pow(abs(data1[n]), 2)
_mean = _sum/len(data1)
x1 = sqrt(_mean)

_sum = 0.0
for n in range(len(data2)):
    _sum += pow(abs(data2[n]), 2)
_mean = _sum/len(data2)
x2 = sqrt(_mean)
string = time.asctime() + ", "
string = string + str(freq) + ",\t"
string = string + str("%.3f" % amp) + ", "
string = string + str("%.5f" % x1.real) + ", "
string = string + str("%.5f" % x2.real) + ", "
avg_phase_diff = sum(phase_diff)/len(phase_diff)
string = string + str("%.5f" % avg_phase_diff) + "\n"
f.write(string)
print string
f.close()
