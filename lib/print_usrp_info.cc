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
#include <vector>

#include "print_usrp_info.h"

int print_usrp_config(uhd::usrp::multi_usrp::sptr usrp, bool istx)
{
  // getting the number of channels
  size_t num_rx_chans, num_tx_chans;
  num_rx_chans = usrp->get_rx_num_channels();
  num_tx_chans = usrp->get_tx_num_channels();

  if(istx) {
    for (size_t m = 0; m < num_tx_chans; m++) {
      uhd::dict<std::string, std::string> info = usrp->get_usrp_tx_info(m);
      std::vector<std::string> keys(info.keys());
      std::vector<std::string> vals(info.vals());
      std::cout << "Transmit Channel "<< m <<"\n----------\n";
      for(size_t n = 0; n < info.size(); n++)
      {
        if (n == 3)
          std::cout << keys[n] << "\t\t: " << vals[n] << std::endl;
        else
          std::cout << keys[n] << "\t: " << vals[n] << std::endl;
      }
      std::cout << "----------\n";
    }
  }
  else {
    for (size_t m = 0; m < num_rx_chans; m++) {
      uhd::dict<std::string, std::string> info = usrp->get_usrp_rx_info(m);
      std::vector<std::string> keys(info.keys());
      std::vector<std::string> vals(info.vals());
      std::cout << "Receive Channel "<< m <<"\n----------\n";
      for(size_t n = 0; n < info.size(); n++)
      {
        if (n == 3)
          std::cout << keys[n] << "\t\t: " << vals[n] << std::endl;
        else
          std::cout << keys[n] << "\t: " << vals[n] << std::endl;
      }
      std::cout << "----------\n";
    }
  }
  return 0;
}

