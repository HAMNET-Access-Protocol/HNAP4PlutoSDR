/*
 * basestation.c
 *
 *  Created on: Jan 24, 2020
 *      Author: lukas
 */

#define _GNU_SOURCE


#include "../mac/mac_bs.h"
#include "../phy/phy_bs.h"
#include "../platform/pluto.h"
#include "../platform/platform_simulation.h"
#include "../util/log.h"

#include <pthread.h>
#include <time.h>
#include <sched.h>

// Set to 1 in order to use the simulated platform
#define BS_USE_PLATFORM_SIM 0

#define BUFLEN ((NFFT+CP_LEN)*2)


// Configure CPU core affinities for the threads
#define BS_RX_CPUID 0
#define BS_TX_CPUID 1
#define BS_MAC_CPUID 0

// struct holds arguments for RX thread
struct rx_th_data_s {
	PhyBS phy;
	platform hw;
};

// struct holds arguments for TX thread
struct tx_th_data_s {
	PhyBS phy;
	platform hw;
	pthread_cond_t* scheduler_signal;
};

// struct holds arguments for MAC thread
struct mac_th_data_s {
	pthread_cond_t* scheduler_signal;
	pthread_mutex_t* scheduler_mutex;
	MacBS mac;
};

// Main Thread for BS receive
void thread_phy_bs_rx(struct rx_th_data_s* arg)
{
	platform hw = arg->hw;
	PhyBS phy = arg->phy;

	float complex* rxbuf_time = calloc(sizeof(float complex),BUFLEN);

	while (1)
	{
		hw->platform_rx(hw, rxbuf_time);
		phy_bs_rx_symbol(phy, rxbuf_time);
		phy_bs_rx_symbol(phy, rxbuf_time+(NFFT+CP_LEN));
	}
}

// Main Thread for BS transmit
void thread_phy_bs_tx(struct tx_th_data_s* arg)
{
	PhyBS phy = arg->phy;
	platform bs = arg->hw;
	pthread_cond_t* scheduler_signal = arg->scheduler_signal;

	float complex* txbuf_time = calloc(sizeof(float complex),BUFLEN);

	uint subframe_cnt = 0;
	while (1)
	{
		for (int symbol=0; symbol<SUBFRAME_LEN/2; symbol++) {
			bs->platform_tx_push(bs);
			phy_bs_write_symbol(phy, txbuf_time);
			phy_bs_write_symbol(phy, txbuf_time+1*(NFFT+CP_LEN));

			bs->platform_tx_prep(bs, txbuf_time, 0, BUFLEN);
			// run scheduler
			if (symbol==30) {
				pthread_cond_signal(scheduler_signal);
			}
		} // end{for}
		subframe_cnt++;
	}

	bs->end(bs);

}

void thread_mac_bs_scheduler(struct mac_th_data_s* arg)
{
	MacBS mac = arg->mac;
	pthread_cond_t* cond_signal = arg->scheduler_signal;
	pthread_mutex_t* mutex = arg->scheduler_mutex;

	uint subframe_cnt = 0;
	while (1) {
		// Wait for signal from BS tx thread
		pthread_mutex_lock(mutex);
		pthread_cond_wait(cond_signal, mutex);

		// add some dummy data
		LOG(INFO,"Prepare frame %d. Time: %.3f\n",subframe_cnt,clock()*1000.0/CLOCKS_PER_SEC);
		// add some data to send
		MacDataFrame dl_frame = dataframe_create(100);
		for (int i=0; i<100; i++)
			dl_frame->data[i] = rand() & 0xFF;
		memcpy(dl_frame->data,&subframe_cnt,sizeof(uint));
		if(!mac_bs_add_txdata(mac, 2, dl_frame)) {
			dataframe_destroy(dl_frame);
		}
		mac_bs_run_scheduler(mac);
		subframe_cnt++;

		// show some MAC stats
		if (subframe_cnt%1000==0) {
			LOG(WARN,"[MAC] channels received:fail %d:%d\n",mac->stats.chan_rx_succ,mac->stats.chan_rx_fail);
			LOG(WARN,"      bytes rx: %d bytes tx: %d\n",mac->stats.bytes_rx, mac->stats.bytes_tx);
		}
		pthread_mutex_unlock(mutex);
	}
}

int main()
{
	pthread_t bs_phy_rx_th, bs_phy_tx_th, bs_mac_th;

	// Init platform
#if BS_USE_PLATFORM_SIM
	platform pluto = platform_init_simulation(BUFLEN);
#else
	platform pluto = init_pluto_platform(BUFLEN);
	pluto_set_rxgain(20);
#endif

	// Init phy and mac layer
	PhyBS phy = phy_bs_init();
	MacBS mac = mac_bs_init();

	phy_bs_set_mac_interface(phy, mac);
	mac_bs_set_phy_interface(mac, phy);

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

	// create arguments for TX thread
	struct tx_th_data_s tx_th_data;
	tx_th_data.hw = pluto;
	tx_th_data.phy = phy;
	tx_th_data.scheduler_signal = &cond;


	// start RX thread
	if (pthread_create(&bs_phy_rx_th, NULL, thread_phy_bs_rx, &rx_th_data) !=0) {
		LOG(ERR,"could not create RX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created RX thread.\n");
	}
	cpu_set_t cpu_set;
	CPU_ZERO(&cpu_set);
	CPU_SET(BS_RX_CPUID,&cpu_set);
	pthread_setaffinity_np(bs_phy_rx_th,sizeof(cpu_set_t),&cpu_set);

	// start TX thread
	if (pthread_create(&bs_phy_tx_th, NULL, thread_phy_bs_tx, &tx_th_data) !=0) {
		LOG(ERR,"could not create TX thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created TX thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(BS_TX_CPUID,&cpu_set);
	pthread_setaffinity_np(bs_phy_tx_th,sizeof(cpu_set_t),&cpu_set);

	// start MAC thread
	if (pthread_create(&bs_mac_th, NULL, thread_mac_bs_scheduler, &mac_th_data) !=0) {
		LOG(ERR,"could not create MAC thread. Abort!\n");
		exit(EXIT_FAILURE);
	} else {
		LOG(INFO,"created MAC thread.\n");
	}
	CPU_ZERO(&cpu_set);
	CPU_SET(BS_MAC_CPUID,&cpu_set);
	pthread_setaffinity_np(bs_mac_th,sizeof(cpu_set_t),&cpu_set);

	// printf affinities
	pthread_getaffinity_np(bs_phy_rx_th,sizeof(cpu_set_t),&cpu_set);
	printf("RX Thread CPU mask: ");
	for (int i=i; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));

	pthread_getaffinity_np(bs_phy_tx_th,sizeof(cpu_set_t),&cpu_set);
	printf("\nTX Thread CPU mask: ");
	for (int i=i; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));

	pthread_getaffinity_np(bs_mac_th,sizeof(cpu_set_t),&cpu_set);
	printf("\nMAC Thread CPU mask: ");
	for (int i=i; i<4; i++)
		printf("%d ",CPU_ISSET(i, &cpu_set));
	printf("\n");

	static int ret[3];
	pthread_join(bs_phy_rx_th, &ret[0]);
	pthread_join(bs_phy_tx_th, &ret[1]);
	pthread_join(bs_mac_th, &ret[2]);
	LOG(INFO,"Thread exit codes: RX: %d, TX: %d, MAC: %d\n",ret[0],ret[1],ret[2]);

}
