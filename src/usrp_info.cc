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
 
#include <iostream>
#include <stdio.h>
#include <getopt.h>
#include <vector>

#include <uhd/usrp/multi_usrp.hpp>

void usage() {
    printf("usrp_info -- Print the usrp information\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
}

int main (int argc, char **argv)
{

  int d;
  while ((d = getopt(argc,argv,"uhF:f:r:a:T:R:S:")) != EOF) {
    switch (d) {
      case 'u':
      case 'h':   usage();                        return 0;
      default :   usage();                        return 0;
    }
  }

  // define the address for tx and rx
  uhd::device_addr_t tx_addr, rx_addr;
  tx_addr["addr0"] = "134.147.118.211"; 
  tx_addr["addr1"] = "134.147.118.212"; 
  rx_addr["addr0"] = "134.147.118.213";
  rx_addr["addr1"] = "134.147.118.214";
  uhd::usrp::multi_usrp::sptr tx = uhd::usrp::multi_usrp::make(tx_addr);
  uhd::usrp::multi_usrp::sptr rx = uhd::usrp::multi_usrp::make(rx_addr);

  rx->set_rx_antenna("TX/RX");
  uhd::dict<std::string, std::string> rx_info_0 = rx->get_usrp_rx_info(0);
  std::vector<std::string> rx_keys_0(rx_info_0.keys());
  std::vector<std::string> rx_vals_0(rx_info_0.vals());
  std::cout << "Receive Channel 0\n----------\n";
  for(size_t n = 0; n < rx_info_0.size(); n++)
  {
    if (n == 3)
      std::cout << rx_keys_0[n] << "\t\t: " << rx_vals_0[n] << std::endl;
    else
      std::cout << rx_keys_0[n] << "\t: " << rx_vals_0[n] << std::endl;
  }
  std::cout << "----------\n";
  uhd::dict<std::string, std::string> rx_info_1 = rx->get_usrp_rx_info(1);
  std::vector<std::string> rx_keys_1(rx_info_1.keys());
  std::vector<std::string> rx_vals_1(rx_info_1.vals());
  std::cout << "Receive Channel 1\n----------\n";
  for(size_t n = 0; n < rx_info_1.size(); n++)
  {
    if (n == 3)
      std::cout << rx_keys_1[n] << "\t\t: " << rx_vals_1[n] << std::endl;
    else
      std::cout << rx_keys_1[n] << "\t: " << rx_vals_1[n] << std::endl;
  }
  std::cout << "----------\n";

  tx->set_tx_antenna("TX/RX");
  uhd::dict<std::string, std::string> tx_info_0 = tx->get_usrp_tx_info(0);
  std::vector<std::string> tx_keys_0(tx_info_0.keys());
  std::vector<std::string> tx_vals_0(tx_info_0.vals());
  std::cout << "Transmit Channel 0\n----------\n";
  for(size_t n = 0; n < tx_info_0.size(); n++)
  {
    if (n == 3)
      std::cout << tx_keys_0[n] << "\t\t: " << tx_vals_0[n] << std::endl;
    else
      std::cout << tx_keys_0[n] << "\t: " << tx_vals_0[n] << std::endl;
  }
  std::cout << "----------\n";
  uhd::dict<std::string, std::string> tx_info_1 = tx->get_usrp_tx_info(1);
  std::vector<std::string> tx_keys_1(tx_info_1.keys());
  std::vector<std::string> tx_vals_1(tx_info_1.vals());
  std::cout << "Transmit Channel 1\n----------\n";
  for(size_t n = 0; n < tx_info_1.size(); n++)
  {
    if (n == 3)
      std::cout << tx_keys_1[n] << "\t\t: " << tx_vals_1[n] << std::endl;
    else
      std::cout << tx_keys_1[n] << "\t: " << tx_vals_1[n] << std::endl;
  }
  std::cout << "----------\n";
  return 0;
}

