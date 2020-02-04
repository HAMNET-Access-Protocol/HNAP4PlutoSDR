/*
 * test_mac.c
 *
 *  Created on: Jan 15, 2020
 *      Author: lukas
 *
 *	First test of MAC+PHY layer
 */

#include "../mac/mac_ue.h"
#include "../mac/mac_bs.h"
#include "../phy/phy_ue.h"
#include "../phy/phy_bs.h"
#include "../platform/platform_simulation.h"

#include "test.h"

#define payload_size 100

// PHY test variables
uint8_t phy_ul[FRAME_LEN][4][MAX_SLOT_DATA];
uint8_t phy_dl[FRAME_LEN][4][MAX_SLOT_DATA];

uint32_t phy_ul_tot_bits=0;
uint32_t phy_ul_biterr=0;
uint32_t phy_dl_tot_bits=0;
uint32_t phy_dl_biterr=0;

// MAC test variables
int mac_dl_timestamps[num_simulated_subframes];
int mac_ul_timestamps[num_simulated_subframes];

uint global_sfn = 0;
uint global_slot = 0;

// UE/BS instances
PhyUE phy_ue;
PhyBS phy_bs;
MacUE mac_ue;
MacBS mac_bs;

uint buflen = NFFT+CP_LEN;

platform bs;
platform client;

int run_simulation(uint num_subframes)
{
	uint subframe_cnt = 0;
	global_sfn = 0;
	global_slot = 0;
	// offset stores the currently compensated TX advance
	// tx_shift stores the shift within the buffer that is caused by the offset
	int offset=0, tx_shift=0, num_samples=buflen;


	// Main simulation thread
	float complex* dl_data = calloc(sizeof(float complex),NFFT+CP_LEN);
	float complex* ul_data_tx = calloc(sizeof(float complex),NFFT+CP_LEN);
	float complex* ul_data_rx = calloc(sizeof(float complex),NFFT+CP_LEN);

	while (subframe_cnt<num_subframes)
	{
		LOG(INFO,"Prepare frame %d\n",subframe_cnt);
		// add some data to send
		MacDataFrame dl_frame = dataframe_create(payload_size);
		for (int i=0; i<payload_size; i++)
			dl_frame->data[i] = rand() & 0xFF;
		memcpy(dl_frame->data,&global_sfn,sizeof(uint));
		mac_bs_add_txdata(mac_bs, 2, dl_frame);
		mac_dl_timestamps[global_sfn] = -global_sfn*SUBFRAME_LEN - global_slot;
		// add some data to send for client
		MacDataFrame ul_frame = dataframe_create(100);
		for (int i=0; i<100; i++)
			ul_frame->data[i] = rand() & 0xFF;
		memcpy(ul_frame->data,&global_sfn,sizeof(uint));
		mac_ul_timestamps[global_sfn] = -global_sfn*SUBFRAME_LEN - global_slot;

		if(!mac_ue_add_txdata(mac_ue, ul_frame)) {
			dataframe_destroy(ul_frame);
		}

		// run BS scheduler
		mac_bs_run_scheduler(mac_bs);

		for (int symbol=0; symbol<SUBFRAME_LEN; symbol++) {
			// BS TX
			phy_bs_write_symbol(phy_bs, dl_data);
			bs->platform_tx_prep(bs, dl_data, 0, buflen);
			bs->platform_tx_push(bs);

			// Client RXTX
			if (!phy_ue->has_synced_once) {
				// Initial sync
				// TODO edge case for offset=68
				client->platform_rx(client, dl_data);
				offset = phy_ue_initial_sync(phy_ue, dl_data, NFFT+CP_LEN);
				if (offset>0) {
					// receive remaining symbols
					phy_ue_do_rx(phy_ue, dl_data, NFFT+CP_LEN-offset);
					phy_ue->rx_offset =  offset;

					offset = -phy_ue->rx_offset;	// TODO use rx offset to align tx
					tx_shift = phy_ue->rx_offset;
					num_samples = buflen-tx_shift;
				}
			} else {
				// ---------- RX ------------
				client->platform_rx(client, dl_data);
				// process samples
				phy_ue_do_rx(phy_ue, dl_data, NFFT+CP_LEN);
				// Run scheduler after DLCTRL slot was received
				if (phy_ue->common->rx_symbol == DLCTRL_LEN) {
					mac_ue_run_scheduler(mac_ue);
				}

				// --------- TX ------------
				// create tx time data
				// first add the last samples from the previous generated symbol
				client->platform_tx_prep(client, ul_data_tx+num_samples, 0, tx_shift);
				// create new symbol
				phy_ue_write_symbol(phy_ue, ul_data_tx);

				// prepare first part of the new symbol
				client->platform_tx_prep(client, ul_data_tx, tx_shift, num_samples);

				// push buffer
				client->platform_tx_push(client);

				// update timing offset for tx. TODO explain chosen tx_Symbol idx
				if (phy_ue->common->tx_symbol == 29 && phy_ue->common->tx_subframe == 0) {
					// check if offset has changed
					int new_offset = phy_ue->mac->timing_advance - phy_ue->rx_offset;
					int diff = new_offset - offset;
					if (abs(diff)>0) {
						LOG(INFO,"[Runtime] adapt tx offset. diff: %d old txshift: %d\n",diff, tx_shift);

						// if offset shift-diff is <0, we have to skip ofdm symbols
						while (tx_shift - diff < 0) {
							phy_ue->common->tx_symbol+=1;
							diff-=buflen;
						}
						while (tx_shift - diff >=buflen) {
							phy_ue->common->tx_symbol-=1;
							diff+=buflen;
						}
						tx_shift = tx_shift - diff;
						offset = new_offset;
						num_samples = buflen - tx_shift;
					}
				}
			} // end(client RXTX)

			// BS RX
			bs->platform_rx(bs, ul_data_rx);
			phy_bs_rx_symbol(phy_bs, ul_data_rx);

			global_slot = (global_slot+1) % SUBFRAME_LEN;
		} // end{for}
		subframe_cnt++;
		global_sfn++;
		// show progress
		if (subframe_cnt%1000==0)
			printf("Processed %d subframes...\n",subframe_cnt);

		// Log stats
		if (subframe_cnt%10000==0) {
			// PHY
			printf("PHY BS: %d bit rx, %d biterrors\n",phy_ul_tot_bits,phy_ul_biterr);
			printf("PHY UE: %d bit rx, %d biterrors\n",phy_dl_tot_bits,phy_dl_biterr);
			// MAC
			printf("MAC BS channels received:fail %d:%d\n",mac_bs->stats.chan_rx_succ,mac_bs->stats.chan_rx_fail);
			printf("       bytes rx: %d bytes tx: %d\n",mac_bs->stats.bytes_rx, mac_bs->stats.bytes_tx);
			printf("MAC UE channels received:fail %d:%d\n",mac_ue->stats.chan_rx_succ,mac_ue->stats.chan_rx_fail);
			printf("       bytes rx: %d bytes tx: %d\n",mac_ue->stats.bytes_rx, mac_ue->stats.bytes_tx);
		}
	}

	printf("PHY BS: %d bit rx, %d biterrors\n",phy_ul_tot_bits,phy_ul_biterr);
	printf("PHY UE: %d bit rx, %d biterrors\n",phy_dl_tot_bits,phy_dl_biterr);
	// MAC
	printf("MAC BS channels received:fail %d:%d\n",mac_bs->stats.chan_rx_succ,mac_bs->stats.chan_rx_fail);
	printf("       bytes rx: %d bytes tx: %d\n",mac_bs->stats.bytes_rx, mac_bs->stats.bytes_tx);
	printf("MAC UE channels received:fail %d:%d\n",mac_ue->stats.chan_rx_succ,mac_ue->stats.chan_rx_fail);
	printf("       bytes rx: %d bytes tx: %d\n",mac_ue->stats.bytes_rx, mac_ue->stats.bytes_tx);

	client->end(client);
	bs->end(bs);

	return 0;
}

int main()
{
	// Setup the hardware
	bs = platform_init_simulation(buflen);
	client = platform_init_simulation(buflen);
	simulation_connect(bs, client);

	// Create PHY and MAC instances
	phy_ue = phy_ue_init();
	phy_bs = phy_bs_init();
	mac_ue = mac_ue_init();
	mac_bs = mac_bs_init();

	phy_ue_set_mac_interface(phy_ue, mac_ue_rx_channel, mac_ue);
	mac_ue_set_phy_interface(mac_ue, phy_ue);
	phy_bs_set_mac_interface(phy_bs, mac_bs);
	mac_bs_set_phy_interface(mac_bs, phy_bs);


	// init mac delay measurements
	for (int i=0; i<num_simulated_subframes; i++) {
		mac_dl_timestamps[i] = -1;
		mac_ul_timestamps[i] = -1;
	}

	run_simulation(num_simulated_subframes);

	FILE* fd = fopen("mac_dl_delays.bin","w");
	if (fd) {
		fwrite(mac_dl_timestamps,sizeof(int),num_simulated_subframes,fd);
		fclose(fd);
	}

	fd = fopen("mac_ul_delays.bin","w");
	if (fd) {
		fwrite(mac_ul_timestamps,sizeof(int),num_simulated_subframes,fd);
		fclose(fd);
	}
}
