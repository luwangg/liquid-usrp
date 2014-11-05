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

#include "print_usrp_info.h"

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
  tx->set_tx_antenna("TX/RX");
  print_usrp_config(rx, false);
  print_usrp_config(tx, true);
  return 0;
}

