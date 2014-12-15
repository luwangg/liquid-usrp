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

#include <cstdlib>
#include <iostream>
#include <complex>

#include <liquid/liquid.h>

int main(int argc, char **argv) {
  std::complex<float> I(0.0f, -1.0f);
  std::complex<float> symb_tab[4] = {1.0f + 1.0f*I,
                                     -1.0f + 1.0f*I,
                                     1.0f - 1.0f*I,
                                     -1.0f - 1.0f*I};
  unsigned char input[256];
  std::complex<float> * exp_output;
  std::complex<float> * output;
  exp_output = (std::complex<float> *)malloc(sizeof(std::complex<float>)*256);
  output = (std::complex<float> *)malloc(sizeof(std::complex<float>)*256);
  for(unsigned int i = 0; i < 256; i++) {
    input[i] = rand()%4;
    exp_output[i] = symb_tab[input[i]];
  }
  chunks_to_symbols qpsk = chunks_to_symbols_create(symb_tab, 4);
  chunks_to_symbols_execute(qpsk, output, input, 256);
  for(unsigned int i = 0; i < 256; i++) {
    std::cout << (exp_output[i] == output[i]);
  }
  chunks_to_symbols_destroy(qpsk);
}

