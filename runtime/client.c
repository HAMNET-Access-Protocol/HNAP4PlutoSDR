/*
 * client.c
 *
 *  Created on: Jan 24, 2020
 *      Author: lukas
 */

#define _GNU_SOURCE

#include "../mac/mac_ue.h"
#include "../phy/phy_ue.h"
#include "../platform/pluto.h"
#include "../platform/platform_simulation.h"
#include "../util/log.h"

#include <pthread.h>
#include <sched.h>

// Config thread core affinities
#define UE_RX_CPUID 1
#define UE_TX_CPUID 0
#define UE_MAC_CPUID 0

#define SYMBOLS_PER_BUF 2
#define BUFLEN ((NFFT+CP_LEN)*SYMBOLS_PER_BUF)

// Set to 1 in order to use the simulated platform
#define CLIENT_USE_PLATFORM_SIM 0


// struct holds arguments for RX thread
struct rx_th_data_s {
	PhyUE phy;
	platform hw;
	pthread_cond_t* scheduler_signal;
};

// struct holds arguments for TX thread
struct tx_th_data_s {
	PhyUE phy;
	platform hw;
};

// struct holds arguments for MAC thread
struct mac_th_data_s {
	pthread_cond_t* scheduler_signal;
	pthread_mutex_t* scheduler_mutex;
	MacUE mac;
};

// Main Thread for UE receive
void thread_phy_ue_rx(struct rx_th_data_s* arg)
{
	platform hw = arg->hw;
	PhyUE phy = arg->phy;
	pthread_cond_t* scheduler_signal = arg->scheduler_signal;

	float complex* rxbuf_time = calloc(sizeof(float complex),BUFLEN);

	// get initial sync
	/*int offset = -1;
	while (offset == -1) {
		hw->platform_rx(hw, rxbuf_time);
		offset = phy_ue_initial_sync(phy, rxbuf_time, BUFLEN);
	}
	if (offset>0) {
		// receive remaining symbols
		phy_ue_do_rx(phy, rxbuf_time, BUFLEN-offset);
		offset = 0;
	}
	LOG(INFO,"[PHY UE] rx thread got sync!\n");*/
	// start the tx thread
	//pthread_cond_signal(&phy->tx_sync_cond);

	// Main RX loop
	while (1) {
		// fill buffer
		hw->platform_rx(hw, rxbuf_time);
		// process samples
		phy_ue_do_rx(phy, rxbuf_time, BUFLEN);
		//log_bin((uint8_t*)rxbuf_time,BUFLEN*sizeof(float complex), "dl_data.bin","a");
		// Run scheduler after DLCTRL slot was received
		if (phy->common->rx_symbol == DLCTRL_LEN ||
				phy->common->rx_symbol == DLCTRL_LEN+1) {
			pthread_cond_signal(scheduler_signal);
		}
	}
}

// Main Thread for UE transmit
void thread_phy_ue_tx(struct tx_th_data_s* arg)
{
	PhyUE phy = arg->phy;
	platform hw = arg->hw;

	float complex* ul_data_tx = calloc(sizeof(float complex),BUFLEN);
	int offset, num_samples, tx_shift=0;;

	// wait until rx thread has achieved sync
	while (!phy->has_synced_once) {
		hw->platform_tx_push(hw);
		hw->platform_tx_prep(hw, ul_data_tx, 0, BUFLEN);
	}

	LOG(INFO,"[PHY UE] main tx thread started!\n");
	offset = -phy->rx_offset;	// TODO use rx offset to align tx
	tx_shift = phy->rx_offset;
	num_samples = BUFLEN-tx_shift;

	while (1) {
		// create tx time data
		// first add the last samples from the previous generated symbol
		hw->platform_tx_prep(hw, ul_data_tx+num_samples, 0, tx_shift);
		// create new symbol
		phy_ue_write_symbol(phy, ul_data_tx);
		phy_ue_write_symbol(phy, ul_data_tx+(NFFT+CP_LEN));

		// prepare first part of the new symbol
		hw->platform_tx_prep(hw, ul_data_tx, tx_shift, num_samples);

		// push buffer
		hw->platform_tx_push(hw);

		// update timing offset for tx. TODO explain chosen tx_Symbol idx
		if (phy->common->tx_symbol >= 29 && phy->common->tx_subframe == 0) {
			// check if offset has changed
			int new_offset = phy->mac->timing_advance - phy->rx_offset;
			int diff = new_offset - offset;
			if (abs(diff)>0) {
				LOG(INFO,"[Runtime] adapt tx offset. diff: %d old txshift: %d\n",diff, tx_shift);

				// if offset shift-diff is <0, we have to skip ofdm symbols
				while (tx_shift - diff < 0) {
					phy->common->tx_symbol+=SYMBOLS_PER_BUF;
					diff-=BUFLEN;
				}
				while (tx_shift - diff >=BUFLEN) {
					phy->common->tx_symbol-=SYMBOLS_PER_BUF;
					diff+=BUFLEN;
				}
				tx_shift = tx_shift - diff;
				offset = new_offset;
				num_samples = BUFLEN - tx_shift;
			}
		}
	}
	hw->end(hw);
}

void thread_mac_ue_scheduler(struct mac_th_data_s* arg)
{
	MacUE mac = arg->mac;
	pthread_cond_t* cond_signal = arg->scheduler_signal;
	pthread_mutex_t* mutex = arg->scheduler_mutex;
	uint sched_rounds=0;
	while (1) {
		// Wait for signal from UE rx thread
		pthread_mutex_lock(mutex);
		pthread_cond_wait(cond_signal, mutex);

		// add some data to send for client
		MacDataFrame ul_frame = dataframe_create(100);
		for (int i=0; i<100; i++)
			ul_frame->data[i] = rand() & 0xFF;
		if(!mac_ue_add_txdata(mac, ul_frame)) {
			dataframe_destroy(ul_frame);
		}

		mac_ue_run_scheduler(mac);
		pthread_mutex_unlock(mutex);
		sched_rounds++;

		// show some MAC stats
		if (sched_rounds%1000==0) {
			LOG(WARN,"[MAC] channels received:fail %d:%d\n",mac->stats.chan_rx_succ,mac->stats.chan_rx_fail);
			LOG(WARN,"      bytes rx: %d bytes tx: %d\n",mac->stats.bytes_rx, mac->stats.bytes_tx);
		}
	}
}

int main()
{
	pthread_t ue_phy_rx_th, ue_phy_tx_th, ue_mac_th;

	// Init platform
#if CLIENT_USE_PLATFORM_SIM
	platform pluto = platform_init_simulation(NFFT+CP_LEN);
#else
	platform pluto = init_pluto_platform(BUFLEN);
	pluto_set_rxgain(40);
	pluto_set_tx_freq(LO_FREQ_UL);
	pluto_set_rx_freq(LO_FREQ_DL);
#endif

	// Init phy and mac layer
	PhyUE phy = phy_ue_init();
	MacUE mac = mac_ue_init();

	phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
	mac_ue_set_phy_interface(mac, phy);

	// create arguments for MAC scheduler thread
	pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
	struct mac_th_data_s mac_th_data;
	mac_th_data.mac = mac;
	mac_th_data.scheduler_mutex = &mutex;
	mac_th_data.scheduler_signal = &cond;

	// create arguments for RX thread
	struct rx_th_data_s rx_th_data;
	rx_th_data.hw = pluto;
	rx_th_data.phy = phy;
	rx_th_data.scheduler_signal = &cond;

	// create arguments for TX thread
	struct tx_th_data_s tx_th_data;
	tx_th_data.hw = pluto;
	tx_th_data.phy = phy;

	// start RX thread
	if (pthread_create(&ue_phy_rx_th, NULL, thread_phy_ue_rx, &rx_th_data) !=0) {
		LOG(ERR,"could not create RX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created RX thread.\n");
	}
	cpu_set_t cpu_set;
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_RX_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_phy_rx_th,2,&cpu_set);

	// start TX thread
	if (pthread_create(&ue_phy_tx_th, NULL, thread_phy_ue_tx, &tx_th_data) !=0) {
		LOG(ERR,"could not create TX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created TX thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_TX_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_phy_tx_th,2,&cpu_set);

	// start MAC thread
	if (pthread_create(&ue_mac_th, NULL, thread_mac_ue_scheduler, &mac_th_data) !=0) {
		LOG(ERR,"could not create MAC thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created MAC thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_MAC_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_mac_th,2,&cpu_set);

	static int ret[3];
	pthread_join(ue_phy_rx_th, &ret[0]);
	pthread_join(ue_phy_tx_th, &ret[1]);
	pthread_join(ue_mac_th, &ret[2]);
	LOG(INFO,"Thread exit codes: RX: %d, TX: %d, MAC: %d\n",ret[0],ret[1],ret[2]);
}
