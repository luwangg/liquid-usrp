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
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>

void usage() {
    printf("miso_tone -- transmit tones in MISO channel\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  F     : center frequency [Hz], default: 900 MHz\n");
    printf("  f     : tone frequency [Hz], default: 1 kHz\n");
    printf("  r     : usrp sampling rate [Hz] default: 1 MHz\n");
    printf("  a     : tone amplitude (default: 0.25)\n");
    printf("  G     : uhd rx gain [dB] (default: 20dB)\n");
    printf("  S     : number of samples to capture (default: 2M)\n");
}

int main (int argc, char **argv)
{
  // define temp variables
  FILE * sink;
  sink = fopen("/tmp/sink", "wb");
  if (!(sink)){
    std::cout << "File open failed\n";
    exit(1);
  }
  // defining PI
  double PI = std::acos(-1);
  
  double cent_freq = 900.0e6;         // center frequency of transmission
  double tone_freq = 1000.0;          // tone frequency
  double samp_rate = 1e6;             // usrp samping rate
  double tone_amp = 0.25;             // tone amplitude
  double uhd_rxgain = 20.0;           // rx frontend gain
  double num_samps = 2e6;             // number of samples to capture.
  size_t sig_src_buff_len = 1e6;      // size of the signal source buffer

  int d;
  while ((d = getopt(argc,argv,"uhF:f:r:a:G:S:")) != EOF) {
    switch (d) {
      case 'u':
      case 'h':   usage();                        return 0;
      case 'F':   cent_freq   = atof(optarg);     break;
      case 'f':   tone_freq   = atof(optarg);     break;
      case 'r':   samp_rate   = atof(optarg);     break;
      case 'a':   tone_amp    = atof(optarg);     break;
      case 'G':   uhd_rxgain  = atof(optarg);     break;
      case 'S':   num_samps   = atof(optarg);     break;
      default:
                  usage();                        return 0;
    }
  }

    // define the address for tx and rx
    uhd::device_addr_t tx_addr, rx_addr;
    tx_addr["addr0"] = "134.147.118.211";
    rx_addr["addr0"] = "134.147.118.213";
    uhd::usrp::multi_usrp::sptr tx = uhd::usrp::multi_usrp::make(tx_addr);
    uhd::usrp::multi_usrp::sptr rx = uhd::usrp::multi_usrp::make(rx_addr);

    // define the signal source - liquid's nco object
    // LIQUID_VCO generates complex sinusoids using sinf and cosf
    // from the standard math library.
    nco_crcf sig_src = nco_crcf_create(LIQUID_VCO);
    std::cout << 2*PI*(tone_freq/samp_rate) << std::endl;
    nco_crcf_set_frequency(sig_src, 2*PI*(tone_freq/samp_rate));
    // allocate memory to store the sinusoid samples
    std::complex<float> * sig_src_buff;
    sig_src_buff = (std::complex<float> *)malloc(sig_src_buff_len*sizeof(std::complex<float>));
    // populate the signal source
    for (int i = 0; i < sig_src_buff_len; i++)
    {
      nco_crcf_cexpf(sig_src, sig_src_buff + i);
      nco_crcf_step(sig_src);
    }
    // temp - write samples to file to see in matplotlib
    if (sig_src_buff_len != fwrite((void *)sig_src_buff,
                                   sizeof(std::complex<float>),
                                   sig_src_buff_len,
                                   sink))
    {
      std::cout << "Error in writing to file, exiting\n";
      exit(1);
    }
    fclose(sink);
    free(sig_src_buff);
    return 0;
}

