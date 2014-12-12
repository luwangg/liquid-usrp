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

int main() {
  unsigned int seq_len_exp = 7;
  unsigned int seq_len = (unsigned int)(pow(2, seq_len_exp)) - 1;
  msequence ms1 = msequence_create(seq_len_exp, 0x0089, 1);
  msequence ms2 = msequence_create(seq_len_exp, 0x00FD, 1);
  std::complex<float> * pn1;
  std::complex<float> * pn2;
  pn1 = (std::complex<float> *)malloc(sizeof(std::complex<float>)*seq_len);
  pn2 = (std::complex<float> *)malloc(sizeof(std::complex<float>)*seq_len);
  for(unsigned int i = 0; i < seq_len; i++)
  {
    pn1[i] = (msequence_advance(ms1) ? 1.0f : -1.0f);
    pn2[i] = (msequence_advance(ms2) ? 1.0f : -1.0f);
    std::cout << pn1[i];
  }
  std::cout << std::endl;
  msequence_destroy(ms1);
  msequence_destroy(ms2);
  {
    FILE * f_pn1;
    f_pn1 = fopen("/home/sreedhm/build/liquid-usrp/data/pn1", "wb");
    if (!(f_pn1)){
      std::cout << "File open failed\n";
      exit(1);
    }
    fwrite((void *)pn1, sizeof(std::complex<float>), seq_len, f_pn1);
    fclose(f_pn1);
    FILE * f_pn2;
    f_pn2 = fopen("/home/sreedhm/build/liquid-usrp/data/pn2", "wb");
    if (!(f_pn2)){
      std::cout << "File open failed\n";
      exit(1);
    }
    fwrite((void *)pn2, sizeof(std::complex<float>), seq_len, f_pn2);
    fclose(f_pn2);
  }
  return 0;
}
