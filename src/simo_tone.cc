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

  // define variables
  // File to save data to.
  FILE * sink1;
  sink1 = fopen("/tmp/sink1", "wb");
  if (!(sink1)){
    std::cout << "File open failed\n";
    exit(1);
  }
  FILE * sink2;
  sink2 = fopen("/tmp/sink2", "wb");
  if (!(sink2)){
    std::cout << "File open failed\n";
    exit(1);
  }

  // defining standard params
  double cent_freq = 900.0e6;         // center frequency of transmission
  double tone_freq = 1000.0;          // tone frequency
  double samp_rate = 1e6;             // usrp samping rate
  float tone_amp = 0.25;              // tone amplitude
  double txgain = 20.0;               // tx frontend gain
  double rxgain = 20.0;               // rx frontend gain
  double num_secs = 5;                // number of seconds to operate
  std::string cpu = "fc32";           // cpu format for the streamer
  std::string wire = "sc16";          // wire formate for the streamer

  // definging constants
  const double PI = std::acos(-1);
  const double wait_to_rx = 2.0;

  // commandline options to modify the standard params
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
      case 'S':   num_secs    = atof(optarg);     break;
      default :   usage();                        return 0;
    }
  }

  // define the address for tx and rx
  uhd::device_addr_t tx_addr, rx_addr;
  tx_addr["addr0"] = "134.147.118.211";
  rx_addr["addr0"] = "134.147.118.213";
  rx_addr["addr1"] = "134.147.118.214";
  uhd::usrp::multi_usrp::sptr tx = uhd::usrp::multi_usrp::make(tx_addr);
  uhd::usrp::multi_usrp::sptr rx = uhd::usrp::multi_usrp::make(rx_addr);

  // setting sync for RX
  rx->set_clock_source("mimo", 1);
  rx->set_time_source("mimo", 1);
  rx->set_time_now(uhd::time_spec_t(0.0), 0);
  boost::this_thread::sleep(boost::posix_time::milliseconds(100));

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

  size_t num_rx_chans = rx->get_rx_num_channels();
  std::vector<size_t> chans;
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
  size_t sig_src_buff_len = tx_stream->get_max_num_samps();
  std::cout << "Transmit Buffer Length :" << sig_src_buff_len << "\n";
  std::complex<float> * sig_src_buff;
  sig_src_buff = (std::complex<float> *)malloc(sig_src_buff_len*sizeof(std::complex<float>));

  // allocate memory to store the received samples
  size_t rec_buff_len = rx_stream->get_max_num_samps();
  std::cout << "Receive Buffer Length :" << rec_buff_len << "\n";
  std::vector<std::complex<float> *> rec_buff;
  for (size_t chan = 0; chan < num_rx_chans; chan++)
    rec_buff.push_back((std::complex<float> *)malloc(rec_buff_len*sizeof(std::complex<float>)));

  // setup streaming
  uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.stream_now = false;
  stream_cmd.time_spec = uhd::time_spec_t(wait_to_rx);
  rx_stream->issue_stream_cmd(stream_cmd);

  // define the signal source - liquid's nco object
  nco_crcf sig_src = nco_crcf_create(LIQUID_VCO);
  nco_crcf_set_frequency(sig_src, 2*PI*(tone_freq/samp_rate));

  uhd::tx_metadata_t txmd;
  txmd.start_of_burst = true;
  txmd.end_of_burst = false;
  uhd::rx_metadata_t rxmd;

  // the starting time
  time_t begin = time(NULL);
  size_t num_rx_samps, num_saved_samps;
  num_saved_samps = 0;

  float timeout = wait_to_rx + 0.1;
  while(time(NULL) - begin < num_secs) {    // run for num_secs
    // populate the signal buffer
    for (unsigned int i = 0; i < sig_src_buff_len; i++)
    {
      nco_crcf_cexpf(sig_src, sig_src_buff + i);
      (*(sig_src_buff + i))*=tone_amp;
      nco_crcf_step(sig_src);
    }
    // transmit the signal buffer
    tx_stream->send(sig_src_buff,
                    sig_src_buff_len,
                    txmd);
    txmd.start_of_burst = false;
    // receive samples to the receive-buffer.
    num_rx_samps = rx_stream->recv(rec_buff,
                                   rec_buff_len,
                                   rxmd,
                                   timeout);
    if (num_rx_samps != rec_buff_len) {
      std::cout << "\nRequested " << rec_buff_len << " samples\n";
      std::cout << "recv returned with " << num_rx_samps << " samples\n";
    }
    if(rxmd.error_code) {
      std::cerr << "Receive Stream Error Code :" << rxmd.error_code << "\n";
      break;
    }
    timeout = 0.1;
    fwrite((void *)rec_buff[0], sizeof(std::complex<float>), num_rx_samps, sink1);
    fwrite((void *)rec_buff[1], sizeof(std::complex<float>), num_rx_samps, sink2);
    num_saved_samps += num_rx_samps;
  }
  // send the last packet
  txmd.end_of_burst = true;
  for (unsigned int i = 0; i < sig_src_buff_len; i++)
  {
    nco_crcf_cexpf(sig_src, sig_src_buff + i);
    (*(sig_src_buff + i))*=tone_amp;
    nco_crcf_step(sig_src);
  }
  tx_stream->send(sig_src_buff,
                  sig_src_buff_len,
                  txmd);
  // stop rx streaming
  stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  rx_stream->issue_stream_cmd(stream_cmd);

  std::cout << "\nNumber of captured samples :" << num_saved_samps << std::endl;
  // free dynamically allocated memory
  free(sig_src_buff);
  for (size_t chan = 0; chan < num_rx_chans; chan++)
    free(rec_buff[chan]);
  // wait a moment for everything to RIP
  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  return EXIT_SUCCESS;
}

