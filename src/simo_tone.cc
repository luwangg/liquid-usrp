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
#include <fstream>
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
    printf("  g     : software rx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd rx gain [dB] (default: 40dB)\n");
    printf("  S     : number of samples to capture (default: 2M)\n");
}

int main (int argc, char **argv)
{
    // defining PI
    double PI = std::acos(-1);

    double frequency = 900.0e6;
    double samp_rate = 2.0e5;
    double rxgain = 0.25;               // software rx gain [dB]
    double uhd_rxgain = 20.0;           // uhd (hardware) rx gain
    double num_samples = 2000;           // uhd (hardware) rx gain

    int d;
    while ((d = getopt(argc,argv,"uhf:s:g:G:S:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'f':   frequency   = atof(optarg);     break;
        case 's':   samp_rate   = atof(optarg);     break;
        case 'g':   rxgain      = atof(optarg);     break;
        case 'G':   uhd_rxgain  = atof(optarg);     break;
        case 'S':   num_samples = atof(optarg);     break;
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

    // try to set rx rate
    usrp->set_rx_rate(samp_rate);
    // get actual rx rate
    std::cout << "Requested sampling rate\t:" << samp_rate << std::endl;
    std::cout << "Actual sampling rate\t:" << usrp->get_rx_rate() << std::endl;

    usrp->set_rx_freq(frequency, 0);
    usrp->set_rx_freq(frequency, 1);
    usrp->set_rx_gain(uhd_rxgain, 0);
    usrp->set_rx_gain(uhd_rxgain, 1);

    // creating the streamer
    std::vector<size_t> channels(2);
    channels[0] = (size_t)0;
    channels[1] = (size_t)1;
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channels;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Length of usrp_buffer
    const size_t max_buff_len = rx_stream->get_max_num_samps();
    size_t buff_len = 256;         // TODO add option to enter the buffer length
    while(buff_len > max_buff_len) {
        buff_len = buff_len/2;
    }
    std::cout << "Maximum possible buffer length\t:" << max_buff_len << std::endl;
    std::cout << "Allocated buffer length\t\t:" << buff_len << std::endl;
    // usrp buffer
    std::vector<std::vector<std::complex<float> > > buffs(
            usrp->get_rx_num_channels(),
            std::vector<std::complex<float> >(buff_len)
            );

    //create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float> *> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++)
        buff_ptrs.push_back(&buffs[i].front());
    
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.num_samps = num_samples;
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    rx_stream->issue_stream_cmd(stream_cmd);
    uhd::rx_metadata_t md;

    size_t num_acc_samps = 0;
    size_t num_samps_read;

    double timeout = 1;

    // files to write data to
    std::ofstream sink1;
    sink1.open("/tmp/sink1", std::ios::binary);
    std::ofstream sink2;
    sink2.open("/tmp/sink2", std::ios::binary);

    while(num_acc_samps < num_samples) {       // FIXME Loop based on time elapsed
        // Filling the usrp buffer
        std::cout << "Inside while\n";
        num_samps_read = rx_stream->recv(buff_ptrs,
                                         buff_len,
                                         md,
                                         3.0,
                                         true);
        //handle the error code
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT){
            std::cout << "Error Timout\n";
            break;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % md.strerror()));
        }
        num_acc_samps += num_samps_read;
        std::cout << "Writing\n";
        sink1.write(reinterpret_cast<const char*>(buff_ptrs[0]), buff_len*sizeof(std::complex<float>));
        sink2.write(reinterpret_cast<const char*>(buff_ptrs[1]), buff_len*sizeof(std::complex<float>));
    }
    sink1.close();
    sink2.close();
    std::cout << "Number of samples read :" << num_acc_samps;
    return 0;
}

