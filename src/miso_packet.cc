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
#include <stdio.h>
#include <stdlib.h>
#include <complex>
#include <getopt.h>
#include <liquid/liquid.h>

#include <uhd/usrp/multi_usrp.hpp>

void usage() {
    printf("packet_tx -- transmit simple packets\n");
    printf("\n");
    printf("  u,h   : usage/help\n");
    printf("  q/v   : quiet/verbose\n");
    printf("  f     : center frequency [Hz], default: 900 MHz\n");
    printf("  b     : bandwidth [Hz] (62.5kHz min, 8MHz max), default: 250 kHz\n");
    printf("  g     : software tx gain [dB] (default: -6dB)\n");
    printf("  G     : uhd tx gain [dB] (default: 40dB)\n");
}

int main (int argc, char **argv)
{
    // command line options
    bool verbose = true;

    unsigned long int DAC_RATE = 100e6;
    double min_bandwidth = 0.25*(DAC_RATE / 512.0);
    double max_bandwidth = 0.25*(DAC_RATE /   4.0);

    double frequency = 900.0e6;
    double bandwidth = 250e3f;
    unsigned int num_frames = 2000;     // number of frames to transmit
    double txgain_dB = -12.0f;               // software tx gain [dB]
    double uhd_txgain = 20.0;           // uhd (hardware) tx gain

    // define source file, find it's length
    FILE *source_file;
    source_file = fopen("/home/sreedhm/data-files/text.txt", "r");
    if(source_file == NULL) {
        std::cout << "The file can't be opened\n";
        exit(1);
    }
    //
    int d;
    while ((d = getopt(argc,argv,"uhqvf:b:g:G:N:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                        return 0;
        case 'f':   frequency   = atof(optarg);     break;
        case 'b':   bandwidth   = atof(optarg);     break;
        case 'g':   txgain_dB   = atof(optarg);     break;
        case 'G':   uhd_txgain  = atof(optarg);     break;
        case 'N':   num_frames  = atoi(optarg);     break;
        default:
            usage();
            return 0;
        }
    }

    if (bandwidth > max_bandwidth) {
        fprintf(stderr,"error: %s, maximum bandwidth exceeded (%8.4f MHz)\n", argv[0], max_bandwidth*1e-6);
        return 0;
    } else if (bandwidth < min_bandwidth) {
        fprintf(stderr,"error: %s, minimum bandwidth exceeded (%8.4f kHz)\n", argv[0], min_bandwidth*1e-3);
        exit(1);
    }

    uhd::device_addr_t dev_addr;
    // device address in dks network
    dev_addr["addr0"] = "134.147.118.213";
    dev_addr["addr1"] = "134.147.118.214";
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);

    // declaring the MIMO config
    usrp->set_clock_source("mimo", 1);
    usrp->set_time_source("mimo", 1);
    usrp->set_time_now(uhd::time_spec_t(0.0), 0);

    // waiting for a while
    std::cout << "Entering sleep\n";
    sleep(2);
    std::cout << "Exiting sleep\n";

    // set properties
    double tx_rate = 4.0*bandwidth;

    // NOTE : the sample rate computation MUST be in double precision so
    //        that the UHD can compute its interpolation rate properly
    unsigned int interp_rate = (unsigned int)(DAC_RATE / tx_rate);
    // ensure multiple of 4
    interp_rate = (interp_rate >> 2) << 2;
    // NOTE : there seems to be a bug where if the interp rate is equal to
    //        240 or 244 we get some weird warning saying that
    //        "The hardware does not support the requested TX sample rate"
    while (interp_rate == 240 || interp_rate == 244)
        interp_rate -= 4;
    // compute usrp sampling rate
    double usrp_tx_rate = DAC_RATE / (double)interp_rate;
    
    // try to set tx rate
    usrp->set_tx_rate(DAC_RATE / interp_rate);

    // get actual tx rate
    usrp_tx_rate = usrp->get_tx_rate();

    // compute arbitrary resampling rate
    double tx_resamp_rate = usrp_tx_rate / tx_rate;

    usrp->set_tx_freq(frequency, 0);
    usrp->set_tx_freq(frequency, 1);
    usrp->set_tx_gain(uhd_txgain, 0);
    usrp->set_tx_gain(uhd_txgain, 1);

    // creating the streamer
    std::vector<size_t> channels(2);
    channels[0] = (size_t)0;
    channels[1] = (size_t)1;
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = channels;
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    // Length of usrp_buffer
    const size_t max_buff_len = tx_stream->get_max_num_samps();
    size_t buff_len = 4096;         // TODO add option to enter the buffer length
    while(buff_len > max_buff_len) {
        buff_len = buff_len/2;
    }
    std::cout << "Maximum possible buffer length\t:" << max_buff_len << std::endl;
    std::cout << "Allocated buffer length\t\t:" << buff_len << std::endl;
    // usrp buffer
    std::vector<std::vector<std::complex<float> > > buffs(
            usrp->get_tx_num_channels(),
            std::vector<std::complex<float> >(buff_len)
            );

    //create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float> *> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++)
        buff_ptrs.push_back(&buffs[i].front());

    printf("frequency   :   %12.8f [MHz]\n", frequency*1e-6f);
    printf("bandwidth   :   %12.8f [kHz]\n", bandwidth*1e-3f);
    printf("verbosity   :   %s\n", (verbose?"enabled":"disabled"));

    printf("sample rate :   %12.8f kHz = %12.8f * %8.6f (interp %u)\n",
            tx_rate * 1e-3f,
            usrp_tx_rate * 1e-3f,
            1.0 / tx_resamp_rate,
            interp_rate);

    // set the IF filter bandwidth
    //usrp->set_tx_bandwidth(2.0f*tx_rate);

    // add arbitrary resampling component
    // TODO : check that resampling rate does indeed correspond to proper bandwidth
    msresamp_crcf resamp = msresamp_crcf_create(2.0*tx_resamp_rate, 60.0f);

    // transmitter gain (linear)
    float g = powf(10.0f, txgain_dB/20.0f);

    // data arrays
    unsigned char header[8];
    unsigned char payload[64];
    
    // create frame generator
    framegen64 fg = framegen64_create();
    framegen64_print(fg);

    // allocate array to hold frame generator samples
    unsigned int frame_len = LIQUID_FRAME64_LEN;   // length of frame64 (defined in liquid.h)
    std::complex<float> frame_samples[frame_len];

    // create buffer for arbitrary resamper output
    std::complex<float> buffer_resamp[(int)(2*tx_resamp_rate) + 64];

    unsigned int usrp_sample_counter = 0;

    // set up the metadta flags
    uhd::tx_metadata_t md;
    md.start_of_burst = false;  // never SOB when continuous
    md.end_of_burst   = false;  // 
    md.has_time_spec  = false;  // set to false to send immediately

    unsigned int j;
    unsigned int pid;
    pid = 0;
    // read 64 byte payload from the source file.
    while (fread((void *)payload, 64, 1, source_file) == 1) {

        if (verbose)
            printf("tx packet id: %6u\n", pid);
        
        // write header (first two bytes packet ID, remaining are random)
        header[0] = (pid >> 8) & 0xff;
        header[1] = (pid     ) & 0xff;
        pid++;
        for (j=2; j<8; j++)
            header[j] = rand() & 0xff;


        // generate the entire frame
        framegen64_execute(fg, header, payload, frame_samples);

        // resample the frame and push resulting samples to USRP
        for (j=0; j<frame_len; j++) {
            // resample one sample at a time
            unsigned int nw;    // number of samples output from resampler
            msresamp_crcf_execute(resamp, &frame_samples[j], 1, buffer_resamp, &nw);

            // for each output sample, stuff into USRP buffer
            unsigned int n;
            for (n=0; n<nw; n++) {
                // append to USRP buffer, scaling by software
                buffs[0][usrp_sample_counter] = g*buffer_resamp[n];
                buffs[1][usrp_sample_counter++] = g*buffer_resamp[n];

                // once USRP buffer is full, reset counter and send to device
                if (usrp_sample_counter==buff_len) {
                    // reset counter
                    usrp_sample_counter=0;

                    // send the result to the USRP
                    tx_stream->send(buff_ptrs,
                                    buff_len,
                                    md);
                }
            }
        }


    } // packet loop
 
    // send a mini EOB packet
    md.start_of_burst = false;
    md.end_of_burst   = true;
    tx_stream->send("", 0, md);

    // sleep for a small amount of time to allow USRP buffers
    // to flush
    usleep(100000);

    fclose(source_file);

    //finished
    printf("usrp data transfer complete\n");

    // delete allocated objects
    framegen64_destroy(fg);
    msresamp_crcf_destroy(resamp);
    return 0;
}

