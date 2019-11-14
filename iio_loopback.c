/*
 * iio_loopback.c
 *
 * Modified from AnalogDevices AdalmPluto iiostream example
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <time.h>

#include "phy/phy_ue.h"
#include "phy/phy_bs.h"
#include "platform/pluto.h"

#include "log.h"

static bool stop;

static void handle_sig(int sig)
{
	printf("Waiting for process to finish...\n");
	stop = true;
}

uint8_t payload[1024];
void create_tx_buf(PhyBS phy_bs, uint mcs, float complex* buf_in)
{
	clock_t before;
	float elapsed_tx=0;

    // generate random payload
	uint tbs = get_tbs_size(phy_bs->common,mcs);
    for (int j=0; j<4*tbs/8; j++) {
        payload[j] = 65 + (j % 26);
    }
    LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

    // Phy channel mapping
    chan->userid = 0x01;
    chan->payload_len = tbs/8;
    chan->data = payload;

    before = clock();
    phy_map_dlslot(phy_bs, chan, 0, mcs);
    chan->data += tbs/8;
    phy_map_dlslot(phy_bs, chan, 1, mcs);
    chan->data += tbs/8;
    phy_map_dlslot(phy_bs, chan, 2, mcs);
    chan->data += tbs/8;
    phy_map_dlslot(phy_bs, chan, 3, mcs);

    phy_write_subframe(phy_bs, buf_in);
    elapsed_tx += (clock() - before)*1000.0/CLOCKS_PER_SEC;

}

PhyUE phy_ue;
void mac_callback(LogicalChannel chan)
{
	static uint total_error = 0;
	static uint total_bits = 0;
	uint biterr = count_bit_errors_array(chan->data,payload+chan->payload_len*chan->writepos,chan->payload_len);
	total_error+=biterr;
	total_bits +=8*(chan->payload_len);
	printf("Biterrors: %d rate: %f total bits: %d\n", biterr, total_error*1.0/total_bits, total_bits);
	printf("CFO: %f\n",ofdmframesync_get_cfo(phy_ue->fs)*256000.0/2/M_PI);
	printf("RX: %.10s\n",chan->data);
	printf("TX: %.10s\n",payload+chan->payload_len*chan->writepos);
	free(chan->data);
}

/* simple configuration and streaming */
/* usage:
 * Default context, assuming local IIO devices, i.e., this script is run on ADALM-Pluto for example
 $./a.out
 * URI context, find out the uri by typing `iio_info -s` at the command line of the host PC
 $./a.out usb:x.x.x
 */
int main (int argc, char **argv)
{
	clock_t start;

	// Listen to ctrl+c and ASSERT
	signal(SIGINT, handle_sig);

	// Create PHY instances and initialize them
	printf("* Setting up UE and BS instances\n");

    unsigned int buf_len=(NFFT+CP_LEN)*SUBFRAME_LEN;
    float complex buf_in[buf_len];
    float complex buf_out[buf_len];
    memset(buf_in,0,buf_len*sizeof(float complex));
    memset(buf_out,0,buf_len*sizeof(float complex));

    uint tx_slot = 0;
    uint rx_slot = 0;
    float complex* buf_out_ptr, buf_in_ptr;

	uint mcs = 0;

	PhyBS phy_bs = phy_init_bs();
	phy_ue = phy_init_ue();
    phy_ue->common->userid = 0x01;
    phy_ue_set_mac_cb(phy_ue, mac_callback);
    phy_ue_set_mcs_dl(phy_ue, mcs);


    // Init TX/RX buffers
    init_pluto_platform();

    //flush tx buffer initially
    pluto_transmit();

    start = clock();

    // Start main loop
	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	while (!stop)
	{
    	printf("tx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		// Push buffer
		pluto_transmit();

    	printf("rx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		// Get new buffer and do RX
		pluto_receive(buf_out,buf_len);
		phy_ue_do_rx(phy_ue,buf_out,buf_len);

    	printf("preptx: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		// prepare new TX buffer
		int tx_samps;
		create_tx_buf(phy_bs, mcs, buf_in);
		tx_samps = pluto_prep_tx(buf_in,(NFFT+CP_LEN)*SUBFRAME_LEN);
		printf("Prepared %d samps\n",tx_samps);

		// log
    	//printf("log: %f\n",(clock()-start)*1000.0/CLOCKS_PER_SEC);
		log_bin((uint8_t*)buf_out,buf_len*sizeof(float complex),"test.bin","a");
	}

	shutdown();

	return 0;
}
