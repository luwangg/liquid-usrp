/*
 * Copyright (c) 2014 Manu T S
 *
 * This is free software: you can redistribute it and/or modify
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
    printf("miso_tone -- transmit tones in MISO channel\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  f     : center frequency [Hz], default: 900 MHz\n");
    printf("  s     : usrp sampling rate [Hz] default: 1 MHz\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
}

int main (int argc, char **argv)
{
    // defining PI
    double PI = std::acos(-1);

    double frequency = 900.0e6;
    double samp_rate = 1.0e6;
    double txgain = 0.25;               // software tx gain [dB]
    double uhd_txgain = 20.0;           // uhd (hardware) tx gain

    int d;
    while ((d = getopt(argc,argv,"uhf:s:g:G:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'f':   frequency   = atof(optarg);     break;
        case 's':   samp_rate   = atof(optarg);     break;
        case 'g':   txgain      = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        default:
            usage();
            return 0;
        }
    }


    uhd::device_addr_t dev_addr;
    // device address in dks network
    dev_addr["addr0"] = "134.147.118.213";
    dev_addr["addr1"] = "134.147.118.214";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // declaring the MIMO config
    usrp->set_clock_source("mimo", 1);
    usrp->set_time_source("mimo", 1);
    usrp->set_time_now(uhd::time_spec_t(0.0), 0);

    // waiting for a while
    std::cout << "Entering sleep\n";
    sleep(2);
    std::cout << "Exiting sleep\n";

    // try to set tx rate
    usrp->set_tx_rate(samp_rate);
    // get actual tx rate
    std::cout << "Requested sampling rate\t:" << samp_rate << std::endl;
    std::cout << "Actual sampling rate\t:" << usrp->get_tx_rate() << std::endl;

    usrp->set_tx_freq(frequency, 0);
    usrp->set_tx_freq(frequency, 1);
    usrp->set_tx_gain(uhd_txgain, 0);
    usrp->set_tx_gain(uhd_txgain, 1);

    // creating the streamer
    std::vector<size_t> channels(2);
    channels[0] = (size_t)0;
    channels[1] = (size_t)1;
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channels;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // Length of usrp_buffer
    const size_t max_buff_len = tx_stream->get_max_num_samps();
    size_t buff_len = 4096;         // TODO add option to enter the buffer length
    while(buff_len > max_buff_len) {
        buff_len = buff_len/2;
    }
    std::cout << "Maximum possible buffer length\t:" << max_buff_len << std::endl;
    std::cout << "Allocated buffer length\t\t:" << buff_len << std::endl;
    // usrp buffer
    std::vector<std::vector<std::complex<float> > > buffs(
            usrp->get_tx_num_channels(),
            std::vector<std::complex<float> >(buff_len)
            );

    //create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float> *> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++)
        buff_ptrs.push_back(&buffs[i].front());

    // number of points in a cycle
    unsigned int len_cycle = 10;
    // one cycle the tone
    std::complex<float> samples[len_cycle];
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

    // copy the tone samples to the usrp tx buffer.
    index = 0;
    while(true) {       // FIXME Loop based on time elapsed
        // Filling the usrp buffer
        for(unsigned int sample_counter = 0; sample_counter < buff_len; sample_counter++) {
            buffs[0][sample_counter] = samples[index];
            buffs[1][sample_counter] = samples[index++];
            if (index == len_cycle)
                index = 0;
        }
        tx_stream->send(buff_ptrs,
                        buff_len,
                        md);
    }
    return 0;
}

