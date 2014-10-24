/*
 * Copyright (c) 2014 Manu T S
 *
 * This software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>

#include <uhd/usrp/multi_usrp.hpp>

void usage() {
    printf("tone_tx -- transmit a tone\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  c     : center frequency [Hz], default: 900 MHz\n");
    printf("  r     : usrp sampling rage [Hz], default: 1 MHz\n");
    printf("  f     : tone frequency [Hz], default: 250 kHz\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
    printf("  t     : number of seconds, default: 100\n");
}

int main (int argc, char **argv)
{
    // defining PI
    double PI = std::acos(-1);

    double center_freq = 900.0e6;
    double samp_rate = 1.0e6;
    double tone_freq = 100e3f;
    unsigned int seconds = 100;         // number of seconds to transmit
    float txgain = 0.2f;               // software tx gain
    double uhd_txgain = 20.0;           // uhd (hardware) tx gain

    //
    int d;
    while ((d = getopt(argc,argv,"uhc:r:f:g:G:t:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'c':   center_freq = atof(optarg);     break;
        case 'f':   tone_freq   = atof(optarg);     break;
        case 'r':   samp_rate   = atof(optarg);     break;
        case 'g':   txgain      = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 't':   seconds     = atoi(optarg);     break;
        default:
            usage();
            return 0;
        }
    }
    
    std::cout << "Requested Parameters  :\n--\n";
    std::cout << "Center Frequency [Hz] : " << center_freq << std::endl;
    std::cout << "Tone Frequency [Hz]   : " << tone_freq << std::endl;
    std::cout << "Sampling Rate [Hz]    : " << samp_rate << std::endl;
    std::cout << "Transmit duration [S] : " << seconds << std::endl;

    
    uhd::device_addr_t dev_addr;
    dev_addr["addr0"] = "134.147.118.212";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);


    usrp->set_tx_rate(samp_rate);
    usrp->set_tx_freq(center_freq);
    usrp->set_tx_gain(uhd_txgain);

    // Creating usrp transmit streamer
    uhd::stream_args_t stream_args("fc32", "sc16");
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);


    // number of points in a cycle
    unsigned int len_cycle = 10;
    // one cycle the tone
    std::complex<float> samples[len_cycle];
    // Length of usrp_buffer
    size_t buffer_len = 2046;
    // usrp buffer
    std::vector<std::complex<float> > usrp_buffer(buffer_len);
    // Index to keep track of the cycle.
    unsigned int index = 0;

    // FIXME Samples corresponds to 100kHz sinusoid at 1MHz sampling frequency
    // Make more general sampling routine to accept the tone frequency and
    // the sampling freqeuncy.
    for(; index < len_cycle; index++) {
        samples[index].real(txgain*cos(index*((2*PI)/len_cycle)));
        samples[index].imag(txgain*sin(index*((2*PI)/len_cycle)));
    }
    
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    index = 0;
    while(true) {       // FIXME Loop based on time elapsed
        // Filling the usrp buffer
        for(unsigned int sample_counter = 0; sample_counter < buffer_len; sample_counter++) {
            usrp_buffer[sample_counter] = samples[index++];
            if (index == len_cycle)
                index = 0;
        }
        tx_stream->send(&usrp_buffer.front(),
                        usrp_buffer.size(),
                        md);
    }
    return 0;
}

