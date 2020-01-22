/*
 * test_mac.c
 *
 *  Created on: Jan 15, 2020
 *      Author: lukas
 *
 *	First test of MAC+PHY layer
 */

#include "mac/mac_ue.h"
#include "mac/mac_bs.h"
#include "phy/phy_ue.h"
#include "phy/phy_bs.h"
#include "platform/platform_simulation.h"

int main()
{
	// Setup the hardware
	uint buflen = NFFT+CP_LEN;
	// offset stores the currently compensated TX advance
	// tx_shift stores the shift within the buffer that is caused by the offset
	int offset=0, tx_shift=0, num_samples=buflen;
	platform bs = platform_init_simulation(buflen);
	platform client = platform_init_simulation(buflen);
	simulation_connect(bs, client);

	// Create PHY and MAC instances
	PhyUE phy_ue = phy_ue_init();
	PhyBS phy_bs = phy_bs_init();
	MacUE mac_ue = mac_ue_init();
	MacBS mac_bs = mac_bs_init();

	phy_ue_set_mac_interface(phy_ue, mac_ue_rx_channel, mac_ue);
	mac_ue_set_phy_interface(mac_ue, phy_ue);
	phy_bs_set_mac_interface(phy_bs, mac_bs);
	mac_bs_set_phy_interface(mac_bs, phy_bs);

	// Main simulation thread
	float complex* dl_data = calloc(sizeof(float complex),NFFT+CP_LEN);
	float complex* ul_data = calloc(sizeof(float complex),NFFT+CP_LEN);

	while (1)
	{
		// add some data to send
		MacDataFrame dl_frame = dataframe_create(100);
		for (int i=0; i<100; i++)
			dl_frame->data[i] = rand() & 0xFF;
		mac_bs_add_txdata(mac_bs, 2, dl_frame);
		// add some data to send for client
		MacDataFrame ul_frame = dataframe_create(100);
		for (int i=0; i<100; i++)
			ul_frame->data[i] = rand() & 0xFF;
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
					offset = 0;
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
				client->platform_tx_prep(client, ul_data, 0, tx_shift);
				// create new symbol
				phy_ue_write_symbol(phy_ue, ul_data);

				// prepare first part of the new symbol
				client->platform_tx_prep(client, ul_data, tx_shift, num_samples);

				// push buffer
				client->platform_tx_push(client);

				// update timing offset for tx. TODO explain chosen tx_Symbol idx
				if (phy_ue->common->tx_symbol == 29 && phy_ue->common->tx_subframe == 0) {
					// check if offset has changed
					int new_offset = phy_ue->mac->timing_advance;
					if (abs(new_offset-offset)>2) {
						// if offset diff is >0, we have to skip ofdm symbols
						while (new_offset-offset > 0) {
							phy_ue->common->tx_symbol++;
							offset+=NFFT+CP_LEN;
						}
						tx_shift = (NFFT+CP_LEN-new_offset) % (NFFT+CP_LEN);
						offset = new_offset;
						num_samples = NFFT+CP_LEN - tx_shift;
					}
				}
			} // end(client RXTX)

			// BS RX
			bs->platform_rx(bs, ul_data);
			phy_bs_rx_symbol(phy_bs, ul_data);

		} // end{for}
	}

	client->end(client);
	bs->end(bs);

	return 0;
}
