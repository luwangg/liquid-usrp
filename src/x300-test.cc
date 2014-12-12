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
#include <cassert>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <gnuradio/filter/fft_filter.h>
#include <gnuradio/gr_complex.h>
#include <volk/volk.h>

#include <boost/thread.hpp>

int make_tx_sig(std::vector<std::complex<float> *> tx_sig,
                 size_t tx_sig_len,
                 nco_crcf sig_src,
                 size_t chan,
                 float tone_amp) {
  std::complex<float> sig_val;
  if(chan > tx_sig.size())
    return 1;
  switch(chan){
    case 0:
      for(unsigned int i = 0; i < tx_sig_len; i++) {
        nco_crcf_cexpf(sig_src, &sig_val);
        *(tx_sig[0] + i) = sig_val*tone_amp;
        *(tx_sig[1] + i) = 0.0f;
        return 0;
      }
    case 1:
      for(unsigned int i = 0; i < tx_sig_len; i++) {
        nco_crcf_cexpf(sig_src, &sig_val);
        *(tx_sig[1] + i) = sig_val*tone_amp;
        *(tx_sig[0] + i) = 0.0f;
        return 0;
      }
    case 2:
      return 1;
    case 3:
      for(unsigned int i = 0; i < tx_sig_len; i++) {
        nco_crcf_cexpf(sig_src, &sig_val);
        *(tx_sig[1] + i) = sig_val*tone_amp;
        *(tx_sig[0] + i) = sig_val*tone_amp;
        return 0;
      }
    default:
      return 1;
  }
}

void usage() {
    printf("x300-test -- MIMO with USRP X300\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  F     : center frequency [Hz], default: 900 MHz\n");
    printf("  f     : tone frequency [Hz], default: 1 kHz\n");
    printf("  r     : usrp sampling rate [Hz] default: 1 MHz\n");
    printf("  a     : tone amplitude (default: 0.25)\n");
    printf("  R     : uhd rx gain [dB] (default: 20dB)\n");
    printf("  T     : uhd tx gain [dB] (default: 20dB)\n");
    printf("  S     : number of samples to Transmit/Receive (default: 500e3)\n");
}

int UHD_SAFE_MAIN(int argc, char **argv)
{
  const double PI = std::acos(-1);

  uhd::set_thread_priority_safe();
  double cent_freq = 2450.0e6;        // center frequency of transmission
  double tone_freq = 1000.0;          // tone frequency
  double samp_rate = 500e3;           // usrp samping rate
  float tone_amp = 0.25;              // tone amplitude
  double txgain = 25.0;               // tx frontend gain
  double rxgain = 25.0;               // rx frontend gain
  std::string cpu = "fc32";           // cpu format for the streamer
  std::string wire = "sc16";          // wire formate for the streamer
  size_t num_rx_chans;
  size_t num_tx_chans;
  size_t tx_buff_len;
  size_t rx_buff_len;
  size_t num_tx_samps;
  size_t num_rx_samps;
  size_t tx_sig_len = size_t(5000e3);
  size_t rx_sig_len = size_t(5000e3);
  size_t rx_sample_point;
  size_t tx_sample_point;
  size_t chan_select;
  bool to_continue;
  float rx_timeout = 0.1;
  float wait_to_rx = 0.1;
  float tx_timeout = 0.1;
  std::vector<size_t> tx_chans;
  std::vector<size_t> rx_chans;
  uhd::usrp::subdev_spec_t tx_subdev_specs;
  uhd::usrp::subdev_spec_t rx_subdev_specs;
  nco_crcf sig_src = nco_crcf_create(LIQUID_VCO);
  uhd::tx_metadata_t txmd;
  uhd::rx_metadata_t rxmd;

  std::vector<std::complex<float> *> tx_buff;
  std::vector<std::complex<float> *> tx_sig;
  std::vector<std::complex<float> *> rx_buff;
  std::vector<std::complex<float> *> rx_sig;
  float * phase_vec;

  int d;
  while ((d = getopt(argc,argv,"uhF:f:r:a:T:R:S:")) != EOF) {
    switch (d) {
      case 'u':
      case 'h':   usage();                        return 0;
      case 'F':   cent_freq   = atof(optarg);     break;
      case 'f':   tone_freq   = atof(optarg);     break;
      case 'r':   samp_rate   = atof(optarg);     break;
      case 'a':   tone_amp    = atof(optarg);     break;
      case 'T':   txgain      = atof(optarg);     break;
      case 'R':   rxgain      = atof(optarg);     break;
      case 'S':   tx_sig_len  = atof(optarg);     
                  rx_sig_len  = atof(optarg);     break;
      default :   usage();                        return 0;
    }
  }

  uhd::device_addr_t tx_addr, rx_addr;
  tx_addr["addr0"] = "134.147.118.216";
  rx_addr["addr0"] = "134.147.118.217";
  uhd::usrp::multi_usrp::sptr tx = uhd::usrp::multi_usrp::make(tx_addr);
  uhd::usrp::multi_usrp::sptr rx = uhd::usrp::multi_usrp::make(rx_addr);

  num_tx_chans = tx->get_tx_num_channels();
  std::cout << "# TX Channels: " << num_tx_chans << std::endl;
  // set freq, rate, gain, antenna.
  std::cout << "Setting rate, gain and freq for TX\n";
  for (size_t chan = 0; chan < num_tx_chans; chan++) {
    std::cout << "Parameters for Channel " << chan <<"\n";
    tx->set_tx_rate(samp_rate, chan);
    std::cout << "TX Rate :";
    std::cout << tx->get_tx_rate(chan) << "\n";
    uhd::tune_request_t tx_tune_request(cent_freq);
    uhd::tune_result_t tx_tune_result;
    tx_tune_result = tx->set_tx_freq(tx_tune_request, chan);
    std::cout << "Transmit Tune Result" << "\n";
    std::cout << tx_tune_result.to_pp_string();
    tx->set_tx_gain(txgain, chan);
    std::cout << "Transmit Gain :" << tx->get_tx_gain(chan) << "dB\n";
    tx->set_tx_antenna("TX/RX", chan);
    std::cout << "Transmit Antenna :" << tx->get_tx_antenna(chan) << "\n";
    tx_chans.push_back(chan);
  }

  num_rx_chans = rx->get_rx_num_channels();
  std::cout << "# RX Channels: " << num_rx_chans << std::endl;
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
    rx_chans.push_back(chan);
  }

  std::cout << "\nSummary of TX Config\n";
  std::cout << tx->get_pp_string();
  std::cout << "\nSummary of RX Config\n";
  std::cout << rx->get_pp_string();
  
  uhd::usrp::subdev_spec_t tx_subdev("B:0");
  tx->set_tx_subdev_spec(tx_subdev, 0);
  tx_subdev_specs = tx->get_tx_subdev_spec();
  rx_subdev_specs = rx->get_tx_subdev_spec();
  std::cout << tx_subdev_specs.to_pp_string();
  std::cout << rx_subdev_specs.to_pp_string();

  // create tx streamer
  uhd::stream_args_t tx_stream_args(cpu, wire);
//  tx_stream_args.channels = tx_chans;
  uhd::tx_streamer::sptr tx_stream = tx->get_tx_stream(tx_stream_args);
  tx_buff_len = tx_stream->get_max_num_samps();

  // create rx streamer
  uhd::stream_args_t rx_stream_args(cpu, wire);
  rx_stream_args.channels = rx_chans;
  uhd::rx_streamer::sptr rx_stream = rx->get_rx_stream(rx_stream_args);
  rx_buff_len = rx_stream->get_max_num_samps();

  for (size_t chan = 0; chan < num_tx_chans; chan++) {
    tx_buff.push_back((std::complex<float> *)malloc(tx_buff_len*sizeof(std::complex<float>)));
    tx_sig.push_back((std::complex<float> *)malloc(tx_sig_len*sizeof(std::complex<float>)));
  }
  for (size_t chan = 0; chan < num_rx_chans; chan++) {
    rx_buff.push_back((std::complex<float> *)malloc(rx_buff_len*sizeof(std::complex<float>)));
    rx_sig.push_back((std::complex<float> *)malloc(rx_sig_len*sizeof(std::complex<float>)));
  }
  txmd.start_of_burst = true;
  txmd.end_of_burst = false;

  chan_select = 1;
  tx_sample_point = 0;
  rx_sample_point = 0;
  nco_crcf_set_frequency(sig_src, 2*PI*(tone_freq/samp_rate));
  make_tx_sig(tx_sig, tx_sig_len, sig_src, chan_select, tone_amp);

  rx->set_time_now(uhd::time_spec_t(0.0), 0);
  tx->set_time_now(uhd::time_spec_t(0.0), 0);
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(wait_to_rx);
  rx_stream->issue_stream_cmd(stream_cmd);
  rx_timeout = wait_to_rx + rx_timeout;

  to_continue =  ((tx_sample_point + tx_buff_len) < tx_sig_len)
              && ((rx_sample_point + rx_buff_len) < rx_sig_len);

  while(to_continue) {
    for(size_t chan = 0; chan < num_tx_chans; chan++) {
      memmove(tx_buff[chan],
              tx_sig[chan] + tx_sample_point,
              sizeof(std::complex<float>)*tx_buff_len);
    }
    num_tx_samps = tx_stream->send(tx_buff,
                                   tx_buff_len,
                                   txmd,
                                   tx_timeout);
    assert(num_tx_samps == tx_buff_len);
    tx_sample_point += num_tx_samps;
    num_rx_samps = rx_stream->recv(rx_buff,
                                   rx_buff_len,
                                   rxmd,
                                   rx_timeout);
    assert(rxmd.error_code == 0x0);
    assert(num_rx_samps == rx_buff_len);
    for(size_t chan = 0; chan < num_rx_chans; chan++) {
      memmove(rx_sig[chan] + rx_sample_point,
              rx_buff[chan],
              sizeof(std::complex<float>)*num_rx_samps);
    }
    rx_sample_point += num_rx_samps;
    to_continue =  ((tx_sample_point + tx_buff_len) < tx_sig_len)
                && ((rx_sample_point + rx_buff_len) < rx_sig_len);
    txmd.start_of_burst = false;
    rx_timeout = 0.1;
  }
  // stop rx streaming
  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  rx_stream->issue_stream_cmd(stream_cmd);
  txmd.end_of_burst = true;
  num_tx_samps = tx_stream->send(tx_buff[0],
                                 0,
                                 txmd,
                                 tx_timeout);
  assert(num_tx_samps == 0);

  std::cout << "tx_sample_point = " << tx_sample_point << "\n";
  std::cout << "rx_sample_point = " << rx_sample_point << "\n";

  phase_vec = (float *)malloc(rx_sample_point*sizeof(float));
  for(unsigned int n = 0; n < rx_sample_point; n++) {
    phase_vec[n] = float(atan2(std::real(rx_sig[0][n]), std::imag(rx_sig[0][n])));
  }

  {
    FILE * f_phase;
    f_phase = fopen("/tmp/phase", "wb");
    if (!(f_phase)){
      std::cout << "File open failed\n";
      exit(1);
    }
    fwrite((void *)phase_vec, sizeof(float), rx_sample_point, f_phase);
    fclose(f_phase);

    FILE * f_rx_sig0;
    f_rx_sig0 = fopen("/tmp/chan0", "wb");
    if (!(f_rx_sig0)){
      std::cout << "File open failed\n";
      exit(1);
    }
    fwrite((void *)rx_sig[0], sizeof(std::complex<float>), rx_sample_point, f_rx_sig0);
    fclose(f_rx_sig0);

    FILE * f_rx_sig1;
    f_rx_sig1 = fopen("/tmp/chan1", "wb");
    if (!(f_rx_sig1)){
      std::cout << "File open failed\n";
      exit(1);
    }
    fwrite((void *)rx_sig[1], sizeof(std::complex<float>), rx_sample_point, f_rx_sig1);
    fclose(f_rx_sig1);
  }

  for (size_t chan = 0; chan < num_tx_chans; chan++) {
    free(tx_buff[chan]);
    free(tx_sig[chan]);
  }
  for (size_t chan = 0; chan < num_rx_chans; chan++) {
    free(rx_buff[chan]);
    free(rx_sig[chan]);
  }
  free(phase_vec);
  return EXIT_SUCCESS;
}
