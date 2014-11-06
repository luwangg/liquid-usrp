#!/usr/bin/env python
##################################################
# Gnuradio Python Flow Graph
# Title: Measureh
# Generated: Mon Oct 27 17:14:27 2014
##################################################

from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import time
import scipy as sp
import matplotlib.pyplot as plt
from cmath import phase, sqrt
from math import pi

def compute_channel(f1, f2, gain1, gain2, txdelay, rxdelay):
    f = open("logfile.csv", 'a')
    data1 = sp.fromfile(open("/tmp/sink1"), dtype=sp.complex64,
            count=10000)[3000:7000]
    data2 = sp.fromfile(open("/tmp/sink2"), dtype=sp.complex64,
            count=10000)[3000:7000]
    
    real1 = [x.real for x in data1]
    imag1 = [x.imag for x in data1]
    real2 = [x.real for x in data2]
    imag2 = [x.imag for x in data2]
    phase1 = [phase(x) for x in data1]
    phase2 = [phase(x) for x in data2]
    phase_diff = [phase1[n] - phase2[n] for n in range(len(data1))]
    for n in range(len(phase_diff)):
        while(phase_diff[n] < 0):
            phase_diff[n] += 2*pi
        while(phase_diff[n] > 2*pi):
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
    string = string + str(f1) + ",\t" + str(f2) + ",\t"
    string = string + str("%.3f" % gain1) + ", " + str("%.3f" % gain2) + ", "
    string = string + str("%.5f" % x1.real) + ", "
    string = string + str("%.5f" % x2.real) + ", "
    avg_phase_diff = sum(phase_diff)/len(phase_diff)
    string = string + str("%.5f" % avg_phase_diff) + "\n"
    f.write(string)
    print string
    f.close()

class MeasureH(gr.top_block):

    def __init__(self, fc, f1, f2, gain1, gain2, txdelay, rxdelay):
        gr.top_block.__init__(self, "Measureh")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 1e6
        self.gain2 = gain2
        self.gain1 = gain1
        self.f1 = f1
        self.f2 = f2
        self.fc = fc
        self.txdelay = txdelay
        self.rxdelay = rxdelay

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("addr0=134.147.118.213, addr1=134.147.118.214", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(2),
        	),
        )
        self.uhd_usrp_source_0.set_clock_source("mimo", 1)
        self.uhd_usrp_source_0.set_time_source("mimo", 1)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(self.fc, 0)
        self.uhd_usrp_source_0.set_gain(20, 0)
        self.uhd_usrp_source_0.set_center_freq(self.fc, 1)
        self.uhd_usrp_source_0.set_gain(20, 1)
        self.uhd_usrp_source_0.set_antenna("TX/RX", 0)
        self.uhd_usrp_source_0.set_antenna("TX/RX", 1)
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(("addr0=134.147.118.211", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(self.fc, 0)
        self.uhd_usrp_sink_0.set_gain(20, 0)
        self.uhd_usrp_sink_0.set_antenna("TX/RX", 0)
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, "/tmp/sink2", False)
        self.blocks_file_sink_1.set_unbuffered(False)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, "/tmp/sink1", False)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate,
            analog.GR_COS_WAVE, self.f2, self.gain2, 0)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.blocks_file_sink_1, 0))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.analog_sig_source_x_1.set_sampling_freq(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

if __name__ == '__main__':
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    (options, args) = parser.parse_args()

    ##################################
    # Set operating parameters
    ##################################
    gain1=0.25
    gain2=0.25
    fc = 900e6
    f = 1000
    f1 = f
    f2 = f
    txdelay = 0
    rxdelay = 0

    ##################################
    # Intiate and run the flowgraph
    ##################################
    tb = MeasureH(fc, f1, f2, gain1, gain2, txdelay, rxdelay)
    tb.start()
    time.sleep(5)
    tb.stop()
    tb.wait()
    time.sleep(1)
#    compute_channel(f1, f2, gain1, gain2, txdelay, rxdelay)
