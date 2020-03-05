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

// size of a mac data frame in bytes [VoIP data + UDP + IP + Ethernet header]
#define payload_size (20+8+20+14)
// how often mac data frames will be sent in ms
#define PACKETIZATION_TIME 20

#define CLIENT_SEND_ENABLE 1
#define BS_SEND_ENABLE 0

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
uint global_symbol = 0;

// UE/BS instances
PhyUE phy_ue;
PhyBS phy_bs;
MacUE mac_ue;
MacBS mac_bs;

uint buflen = NFFT+CP_LEN;

platform bs;
platform client;

uint get_sim_time_msec()
{
	float time =  1000.0*(global_sfn*SUBFRAME_LEN + global_symbol)*(NFFT+CP_LEN)/SAMPLERATE;
	return time;
}

int run_simulation(uint num_subframes, uint mcs)
{
	uint subframe_cnt = 0;
	global_sfn = 0;
	global_symbol = 0;
	// offset stores the currently compensated TX advance
	// tx_shift stores the shift within the buffer that is caused by the offset
	int offset=0, tx_shift=0, num_samples=buflen;

	// Buffers for simulation
	float complex dl_data[NFFT+CP_LEN];
	float complex ul_data_tx[NFFT+CP_LEN];
	float complex ul_data_rx[NFFT+CP_LEN];

	uint last_tx = get_sim_time_msec();
	uint packet_id = 0;

	while (subframe_cnt<num_subframes)
	{
		// Change MCS at some point
		if (1) {
			//mac_bs_set_mcs(mac_bs,2,mcs,DL);
			//mac_bs_set_mcs(mac_bs,2,mcs,UL);
			// hard set instead of signaling.
			if (mac_bs->UE[2])  {
				mac_bs->UE[2]->dl_mcs = mcs;
				mac_bs->UE[2]->ul_mcs = mcs;
			}
			phy_ue->mcs_dl = mcs;
			mac_ue->dl_mcs = mcs;
			mac_ue->ul_mcs = mcs;
		}

		// Add some data every 20ms
		if (get_sim_time_msec() - last_tx > PACKETIZATION_TIME) {
			last_tx = get_sim_time_msec();
			LOG(INFO,"Prepare frame %d\n",packet_id);
			// add some data to send
#if BS_SEND_ENABLE
            MacDataFrame dl_frame = dataframe_create(payload_size);
			for (int i=0; i<payload_size; i++)
				dl_frame->data[i] = rand() & 0xFF;
			memcpy(dl_frame->data,&packet_id,sizeof(uint));
			if(!mac_bs_add_txdata(mac_bs, 2, dl_frame))
				dataframe_destroy(dl_frame);
			mac_dl_timestamps[packet_id] = -global_sfn*SUBFRAME_LEN - global_symbol;
#endif
#if CLIENT_SEND_ENABLE
			// add some data to send for client
			MacDataFrame ul_frame = dataframe_create(payload_size);
			for (int i=0; i<payload_size; i++)
				ul_frame->data[i] = rand() & 0xFF;
			memcpy(ul_frame->data,&packet_id,sizeof(uint));
			mac_ul_timestamps[packet_id] = -global_sfn*SUBFRAME_LEN - global_symbol;

			if(!mac_ue_add_txdata(mac_ue, ul_frame)) {
				dataframe_destroy(ul_frame);
			}
#endif
			packet_id++;
		}

		// run BS scheduler
		if (phy_bs->common->tx_symbol==0)
			mac_bs_run_scheduler(mac_bs);

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
					LOG(ERR,"[Runtime] adapt tx offset. diff: %d old txshift: %d\n",diff, tx_shift);

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

		global_symbol = (global_symbol+1) % SUBFRAME_LEN;
		if (global_symbol==0) {
			subframe_cnt++;
			global_sfn++;
		}

		// show progress
		if (subframe_cnt%1000==0 && global_symbol==0)
			printf("Processed %d subframes...\n",subframe_cnt);

		// Log stats
		if (subframe_cnt%10000==0 && global_symbol==0) {
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

	return 0;
}

void setup_simulation(float snr)
{
	// Setup the hardware
	bs = platform_init_simulation(buflen, snr);
	client = platform_init_simulation(buflen, snr);
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

	// init phy measurements
	phy_ul_tot_bits=0;
	phy_ul_biterr=0;
	phy_dl_tot_bits=0;
	phy_dl_biterr=0;
}

void clean_simulation()
{
	phy_bs_destroy(phy_bs);
	phy_ue_destroy(phy_ue);
	mac_bs_destroy(mac_bs);
	mac_ue_destroy(mac_ue);
	bs->end(bs);
	client->end(client);
}

int main(int argc, char* argv[])
{
	// Arrays to store biterror rates. First index: MCS, second index: SNR
	double biterr_ul_array[5][40]= {0};
	double biterr_dl_array[5][40]= {0};
	int mcs=0;

	if (argc==2) {
		char * ptr;
		mcs = strtol(argv[1],&ptr, 10);
	}

	for (int snr= 5; snr<40; snr+=1) {
		printf("Starting simulation with SNR %ddB mcs%d\n",snr,mcs);

		setup_simulation(snr);
		run_simulation(num_simulated_subframes, mcs);
		clean_simulation();

		char filename[40];
		sprintf(filename,"sim/mac_dl_delays_mcs%d_snr%d",mcs,snr);
		FILE* fd = fopen(filename,"w");
		if (fd) {
			fwrite(mac_dl_timestamps,sizeof(int),num_simulated_subframes,fd);
			fclose(fd);
		}

		sprintf(filename,"sim/mac_ul_delays_mcs%d_snr%d",mcs,snr);
		fd = fopen(filename,"w");
		if (fd) {
			fwrite(mac_ul_timestamps,sizeof(int),num_simulated_subframes,fd);
			fclose(fd);
		}

		//log biterr
		biterr_ul_array[mcs][snr] = (double)phy_ul_biterr / phy_ul_tot_bits;
		biterr_dl_array[mcs][snr] = (double)phy_dl_biterr / phy_dl_tot_bits;
	}

	printf("biterr_ul(%d,:) = [",mcs+1);
	for (int snr=0; snr<40; snr++) {
		printf("%.12f ",biterr_ul_array[mcs][snr]);
	}
	printf("];\n");

	printf("biterr_dl(%d,:) = [",mcs+1);
	for (int snr=0; snr<40; snr++) {
		printf("%.12f ",biterr_dl_array[mcs][snr]);
	}
	printf("];");
}
