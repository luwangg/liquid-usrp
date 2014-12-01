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
#include <ctime>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>

#include <boost/thread.hpp>

void usage() {
    printf("simo_tone -- Transmit a tone in SIMO Channel\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  F     : center frequency [Hz], default: 900 MHz\n");
    printf("  f     : tone frequency [Hz], default: 1 kHz\n");
    printf("  r     : usrp sampling rate [Hz] default: 1 MHz\n");
    printf("  a     : tone amplitude (default: 0.25)\n");
    printf("  R     : uhd rx gain [dB] (default: 20dB)\n");
    printf("  T     : uhd tx gain [dB] (default: 20dB)\n");
    printf("  S     : number of seconds to operate (default: 5)\n");
}

int UHD_SAFE_MAIN(int argc, char **argv)
{
  uhd::set_thread_priority_safe();

  const std::complex<float> I(0.0, 1.0);
  double cent_freq = 900.0e6;         // center frequency of transmission
  double samp_rate = 1e6;             // usrp samping rate
  float tone_amp = 0.25;              // tone amplitude
  double txgain = 5.0;                // tx frontend gain
  double rxgain = 1.0;                // rx frontend gain
  double num_secs = 5;                // number of seconds to operate
  std::string cpu = "fc32";           // cpu format for the streamer
  std::string wire = "sc16";          // wire formate for the streamer
  unsigned int seq_len_exp = 6;
  unsigned int seq_len = (unsigned int)(pow(2, seq_len_exp));
  unsigned int k  = 16;               // samples/symbol
  unsigned int m  = 11;               // filter delay (symbols)
  float beta      = 0.35f;            // excess bandwidth factor
  size_t num_rx_chans;
  size_t tx_buff_len;
  size_t rx_buff_len;

  msequence ms1 = msequence_create(seq_len_exp, 0x005b, 1);
  msequence ms2 = msequence_create(seq_len_exp, 0x0043, 1);
  firinterp_crcf interp = firinterp_crcf_create_rnyquist(LIQUID_FIRFILT_ARKAISER,k,m,beta,0);

  std::complex<float> * pn1;
  std::complex<float> * pn2;
  std::complex<float> * smb;
  std::complex<float> * rrc;
  std::complex<float> * tx_buff;
  std::vector<std::complex<float> *> rx_buff;

  pn1 = (std::complex<float> *)malloc(sizeof(std::complex<float>)*seq_len);
  pn2 = (std::complex<float> *)malloc(sizeof(std::complex<float>)*seq_len);
  smb = (std::complex<float> *)malloc(sizeof(std::complex<float>)*3*seq_len);
  rrc = (std::complex<float> *)malloc(sizeof(std::complex<float>)*3*seq_len*k);
  std::vector<size_t> chans;

  for(unsigned int i = 0; i < seq_len; i++)
  {
    pn1[i] = (msequence_advance(ms1) ? 1.0f : -1.0f);
    pn2[i] = (msequence_advance(ms2) ? 1.0f : -1.0f);
  }
  msequence_destroy(ms1);
  msequence_destroy(ms2);

  for(unsigned int i = 0; i < seq_len; i++)
  {
    smb[i] = 0.0f;
    smb[i + seq_len] = pn1[i] + I*pn2[i];
    smb[i + 2*seq_len] = 0.0f;
  }
  for (unsigned int i = 0; i < 3*seq_len + m; i++) {
    if(i < m)
      firinterp_crcf_execute(interp, smb[i], rrc + 0);
    else {
      firinterp_crcf_execute(interp, smb[i%(3*seq_len)], rrc + k*(i - m));
    }
  }
  firinterp_crcf_destroy(interp);

  uhd::device_addr_t tx_addr, rx_addr;
  tx_addr["addr0"] = "134.147.118.211";
  rx_addr["addr0"] = "134.147.118.212";
  uhd::usrp::multi_usrp::sptr tx = uhd::usrp::multi_usrp::make(tx_addr);
  uhd::usrp::multi_usrp::sptr rx = uhd::usrp::multi_usrp::make(rx_addr);

  rx->set_clock_source("mimo", 0);
  rx->set_time_source("mimo", 0);

  // setting tx frequency, rate and gain.
  tx->set_tx_rate(samp_rate);
  std::cout << "TX Rate :";
  std::cout << tx->get_tx_rate(0) << "\n";
  uhd::tune_request_t tx_tune_request(cent_freq);
  uhd::tune_result_t tx_tune_result;
  tx_tune_result = tx->set_tx_freq(tx_tune_request);
  std::cout << "Transmit Tune Result\n";
  std::cout << tx_tune_result.to_pp_string();
  tx->set_tx_gain(txgain);
  std::cout << "Transmit gain :" << tx->get_tx_gain() << "dB\n";
  tx->set_tx_antenna("TX/RX");
  std::cout << "Transmit Antenna :" << tx->get_tx_antenna() << "\n";

  num_rx_chans = rx->get_rx_num_channels();
  // set freq, rate, gain, antenna.
  std::cout << "Setting rate, gain and freq for RX\n";
  for (size_t chan = 0; chan < num_rx_chans; chan++) {
    std::cout << "Parameters for Channel " << chan <<"\n";
    rx->set_rx_rate(samp_rate, chan);
    std::cout << "RX Rate :";
    std::cout << rx->get_rx_rate(chan) << "\n";
    uhd::tune_request_t rx_tune_request(cent_freq);
    uhd::tune_result_t rx_tune_result;
    rx_tune_result = rx->set_rx_freq(rx_tune_request, chan);
    std::cout << "Receive Tune Result" << "\n";
    std::cout << rx_tune_result.to_pp_string();
    rx->set_rx_gain(rxgain, chan);
    std::cout << "Receive Gain :" << rx->get_rx_gain(chan) << "dB\n";
    rx->set_rx_antenna("TX/RX", chan);
    std::cout << "Receive Antenna :" << rx->get_rx_antenna(chan) << "\n";
    chans.push_back(chan);
  }

  // create a tx streamer
  uhd::stream_args_t tx_stream_args(cpu, wire);
  uhd::tx_streamer::sptr tx_stream = tx->get_tx_stream(tx_stream_args);

  // create an rx streamer
  uhd::stream_args_t rx_stream_args(cpu, wire);
  rx_stream_args.channels = chans;
  uhd::rx_streamer::sptr rx_stream = rx->get_rx_stream(rx_stream_args);

  // allocate memory to store the sinusoid samples
  tx_buff_len = tx_stream->get_max_num_samps();
  std::cout << "Transmit Buffer Length :" << tx_buff_len << "\n";
  tx_buff = (std::complex<float> *)malloc(tx_buff_len*sizeof(std::complex<float>));

  // allocate memory to store the received samples
  rx_buff_len = rx_stream->get_max_num_samps();
  std::cout << "Receive Buffer Length :" << rx_buff_len << "\n";
  for (size_t chan = 0; chan < num_rx_chans; chan++)
    rx_buff.push_back((std::complex<float> *)malloc(rx_buff_len*sizeof(std::complex<float>)));

  // free all
  free(rrc)
  free(pn1);
  free(pn2);
  free(smb);
  return EXIT_SUCCESS;
}

  /*
  FILE * f;
  f = fopen("/tmp/rrc", "wb");
  if (!(f)){
    std::cout << "File open failed\n";
    exit(1);
  }
  fwrite((void *)rrc, sizeof(std::complex<float>), seq_len*3*k, f);
  fclose(f);
  */
