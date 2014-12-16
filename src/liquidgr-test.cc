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
#include <ctime>
#include <cassert>

#include <liquid/liquid.h>
#include <liquid/test.h>

#include <uhd/utils/thread_priority.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <gnuradio/filter/fft_filter.h>
#include <gnuradio/gr_complex.h>
#include <volk/volk.h>

#include <boost/thread.hpp>

int UHD_SAFE_MAIN(int argc, char **argv)
{
  unsigned char str[] = "abcdefg";
  liquid::test::copy cpy(sizeof(unsigned char));
  unsigned char * out_buff;
  out_buff = (unsigned char *)malloc(sizeof(unsigned char)*7);
  cpy.work(out_buff, str, 7);
  for(unsigned int i = 0; i < 7; i++)
    std::cout << out_buff[i];
  std::cout << std::endl;
  return EXIT_SUCCESS;
}
