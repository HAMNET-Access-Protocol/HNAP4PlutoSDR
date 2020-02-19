/*
 * client.c
 *
 *  Created on: Jan 24, 2020
 *      Author: lukas
 */

#define _GNU_SOURCE

#include "../mac/mac_ue.h"
#include "../phy/phy_ue.h"
#include "../phy/phy_config.h"
#include "../platform/pluto.h"
#include "../platform/platform_simulation.h"
#include "../util/log.h"

#include <pthread.h>
#include <sched.h>
#include <time.h>

// Config thread core affinities
#define UE_RX_CPUID 1
#define UE_TX_CPUID 1
#define UE_MAC_CPUID 0
#define UE_RX_SLOT_CPUID 0
#define UE_TAP_CPUID 0

#define SYMBOLS_PER_BUF 2
#define BUFLEN ((NFFT+CP_LEN)*SYMBOLS_PER_BUF)

// Set to 1 in order to use the simulated platform
#define CLIENT_USE_PLATFORM_SIM 0

#define CLIENT_SEND_ENABLE 1


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

// struct with args for RX slot thread
struct rx_slot_th_data_s {
	PhyUE phy;
	pthread_cond_t* rx_slot_signal;
	pthread_mutex_t* rx_slot_mutex;
};

// Main Thread for UE receive
void thread_phy_ue_rx(struct rx_th_data_s* arg)
{
	platform hw = arg->hw;
	PhyUE phy = arg->phy;
	pthread_cond_t* scheduler_signal = arg->scheduler_signal;
	struct timespec start, end;
	double elapsed;

	float complex* rxbuf_time = calloc(sizeof(float complex),BUFLEN);

	// read some rxbuffer objects in order to empty rxbuffer queue
	for (int i=0; i<10; i++)
		hw->platform_rx(hw, rxbuf_time);

	// Main RX loop
	while (1) {
		// fill buffer
		hw->platform_rx(hw, rxbuf_time);
		// process samples
		clock_gettime(CLOCK_MONOTONIC, &start);
		phy_ue_do_rx(phy, rxbuf_time, BUFLEN);
		//log_bin((uint8_t*)rxbuf_time,BUFLEN*sizeof(float complex), "dl_data.bin","a");
		// Run scheduler after DLCTRL slot was received
		if (phy->common->rx_symbol == DLCTRL_LEN ||
				phy->common->rx_symbol == DLCTRL_LEN+1) {
			pthread_cond_signal(scheduler_signal);
		}
		clock_gettime(CLOCK_MONOTONIC, &end);
		elapsed = (end.tv_sec-start.tv_sec)*1000000.0 +(end.tv_nsec-start.tv_nsec)/1000.0;

		if (elapsed > 2000)
			LOG(WARN,"RX timing warn: Iteration took %.1fus in %d %d\n",elapsed,
					phy->common->rx_subframe, phy->common->rx_symbol);

	}
}

// Main Thread for UE transmit
void thread_phy_ue_tx(struct tx_th_data_s* arg)
{
	PhyUE phy = arg->phy;
	platform hw = arg->hw;
	struct timespec start, end;
	double elapsed;

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
		clock_gettime(CLOCK_MONOTONIC, &start);
		// create tx time data
		// first add the last samples from the previous generated symbol
		hw->platform_tx_prep(hw, ul_data_tx+num_samples, 0, tx_shift);
		// create new symbol
		phy_ue_write_symbol(phy, ul_data_tx);
		phy_ue_write_symbol(phy, ul_data_tx+(NFFT+CP_LEN));

		// prepare first part of the new symbol
		hw->platform_tx_prep(hw, ul_data_tx, tx_shift, num_samples);

		clock_gettime(CLOCK_MONOTONIC, &end);
		elapsed = (end.tv_sec-start.tv_sec)*1000000.0 +(end.tv_nsec-start.tv_nsec)/1000.0;
		if (elapsed > 1000) //TODO calculate based in buf size and kernel buffer amount
			LOG(WARN,"TX timing warn: Iteration took %.1fus. %d %d\n",elapsed,
					phy->common->tx_subframe, phy->common->tx_symbol);
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
	struct timespec start, end;
	double elapsed;
	uint sched_rounds=0;

	while (1) {
		// Wait for signal from UE rx thread
		clock_gettime(CLOCK_MONOTONIC,&start);
		pthread_mutex_lock(mutex);
		pthread_cond_wait(cond_signal, mutex);

		// add some data to send for client
#if CLIENT_SEND_ENABLE
		MacDataFrame ul_frame = dataframe_create(120);
		for (int i=0; i<100; i++)
			ul_frame->data[i] = rand() & 0xFF;
		if(!mac_ue_add_txdata(mac, ul_frame)) {
			dataframe_destroy(ul_frame);
		}
#endif
		mac_ue_run_scheduler(mac);
		pthread_mutex_unlock(mutex);
		sched_rounds++;

		// show some MAC stats
		if (sched_rounds%1000==0) {
			LOG(WARN,"[MAC] channels received:fail %d:%d\n",mac->stats.chan_rx_succ,mac->stats.chan_rx_fail);
			LOG(WARN,"      bytes rx: %d bytes tx: %d\n",mac->stats.bytes_rx, mac->stats.bytes_tx);
		}

		clock_gettime(CLOCK_MONOTONIC, &end);
		elapsed = (end.tv_sec-start.tv_sec)*1000000.0 +(end.tv_nsec-start.tv_nsec)/1000.0;
		LOG(INFO,"Prepared frame %d. Time: %.3f\n",sched_rounds,elapsed);

	}
}

void thread_phy_ue_rx_slot(struct rx_slot_th_data_s* arg)
{
	PhyUE phy = arg->phy;
	pthread_cond_t* cond_signal = arg->rx_slot_signal;
	pthread_mutex_t* mutex = arg->rx_slot_mutex;
	struct timespec start, end;
	double elapsed;

	while (1) {
		// Wait for signal from UE rx thread
		pthread_mutex_lock(mutex);
		pthread_cond_wait(cond_signal, mutex);
		clock_gettime(CLOCK_MONOTONIC,&start);

		phy_ue_proc_slot(phy,phy->rx_slot_nr);

		pthread_mutex_unlock(mutex);
		clock_gettime(CLOCK_MONOTONIC,&end);
		elapsed = (end.tv_sec-start.tv_sec)*1000000.0 +(end.tv_nsec-start.tv_nsec)/1000.0;
		if (elapsed > 3500)
			LOG(WARN,"[PHY UE] Timing warn: RX slot processing took %.3fus\n",elapsed);
	}
}

int main()
{
	pthread_t ue_phy_rx_th, ue_phy_tx_th, ue_mac_th, ue_tap_th, ue_phy_rx_slot_th;

	// Init platform
#if CLIENT_USE_PLATFORM_SIM
	platform pluto = platform_init_simulation(NFFT+CP_LEN);
#else
	platform pluto = init_pluto_platform(BUFLEN);
	pluto_set_rxgain(20);
	pluto_set_tx_freq(LO_FREQ_UL);
	pluto_set_rx_freq(LO_FREQ_DL);
	pluto_start_monitor();
#endif

	// Init phy and mac layer
	PhyUE phy = phy_ue_init();
	MacUE mac = mac_ue_init();

	phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
	mac_ue_set_phy_interface(mac, phy);

	// create arguments for MAC scheduler thread
	pthread_mutex_t mac_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t mac_cond = PTHREAD_COND_INITIALIZER;
	struct mac_th_data_s mac_th_data;
	mac_th_data.mac = mac;
	mac_th_data.scheduler_mutex = &mac_mutex;
	mac_th_data.scheduler_signal = &mac_cond;

	// create arguments for RX thread
	struct rx_th_data_s rx_th_data;
	rx_th_data.hw = pluto;
	rx_th_data.phy = phy;
	rx_th_data.scheduler_signal = &mac_cond;

	// create arguments for TX thread
	struct tx_th_data_s tx_th_data;
	tx_th_data.hw = pluto;
	tx_th_data.phy = phy;

	// create arguments for RX slot thread
	pthread_cond_t rx_slot_cond = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t rx_slot_mutex = PTHREAD_MUTEX_INITIALIZER;
	struct rx_slot_th_data_s rx_slot_th_data;
	rx_slot_th_data.phy = phy;
	rx_slot_th_data.rx_slot_mutex = &rx_slot_mutex;
	rx_slot_th_data.rx_slot_signal = &rx_slot_cond;
	phy_ue_set_rx_slot_th_signal(phy,&rx_slot_cond);

	// start RX slot thread
	if (pthread_create(&ue_phy_rx_slot_th, NULL, thread_phy_ue_rx_slot, &rx_slot_th_data) !=0) {
		LOG(ERR,"could not create RX slot processing thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created RX slot processing thread.\n");
	}
	cpu_set_t cpu_set;
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_RX_SLOT_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_phy_rx_slot_th,sizeof(cpu_set_t),&cpu_set);

	// start RX thread
	if (pthread_create(&ue_phy_rx_th, NULL, thread_phy_ue_rx, &rx_th_data) !=0) {
		LOG(ERR,"could not create RX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created RX thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_RX_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_phy_rx_th,sizeof(cpu_set_t),&cpu_set);

	// start TX thread
	if (pthread_create(&ue_phy_tx_th, NULL, thread_phy_ue_tx, &tx_th_data) !=0) {
		LOG(ERR,"could not create TX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created TX thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_TX_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_phy_tx_th,sizeof(cpu_set_t),&cpu_set);

	// start MAC thread
	if (pthread_create(&ue_mac_th, NULL, thread_mac_ue_scheduler, &mac_th_data) !=0) {
		LOG(ERR,"could not create MAC thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created MAC thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_MAC_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_mac_th,sizeof(cpu_set_t),&cpu_set);

	// start thread that binds to TAP and receives data
	if (pthread_create(&ue_tap_th, NULL, mac_ue_tap_rx_th, mac) != 0) {
		LOG(ERR,"could not create TAP rx thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created TAP thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(UE_TAP_CPUID,&cpu_set);
	pthread_setaffinity_np(ue_tap_th,sizeof(cpu_set_t),&cpu_set);

	// printf affinities
	pthread_getaffinity_np(ue_phy_rx_th,sizeof(cpu_set_t),&cpu_set);
	printf("RX Thread CPU mask: ");
	for (int i=0; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));

	pthread_getaffinity_np(ue_phy_tx_th,sizeof(cpu_set_t),&cpu_set);
	printf("\nTX Thread CPU mask: ");
	for (int i=0; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));

	pthread_getaffinity_np(ue_mac_th,sizeof(cpu_set_t),&cpu_set);
	printf("\nMAC Thread CPU mask: ");
	for (int i=0; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));

	pthread_getaffinity_np(ue_phy_rx_slot_th,sizeof(cpu_set_t),&cpu_set);
	printf("\nRX slot proc Thread CPU mask: ");
	for (int i=0; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));
	printf("\n");

	static int ret[4];
	pthread_join(ue_phy_rx_th, &ret[0]);
	pthread_join(ue_phy_tx_th, &ret[1]);
	pthread_join(ue_mac_th, &ret[2]);
	pthread_join(ue_phy_rx_slot_th, &ret[3]);
	LOG(INFO,"Thread exit codes: RX: %d, TX: %d, MAC: %d\n",ret[0],ret[1],ret[2]);
}
