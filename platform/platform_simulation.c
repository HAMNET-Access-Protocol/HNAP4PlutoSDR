/*
 * platform_simulation.c
 *
 *  Created on: Jan 17, 2020
 *      Author: lukas
 */

#include "platform_simulation.h"
#include "../phy/phy_config.h"
#include <liquid/liquid.h>
#include <math.h>
#include <string.h>
#include "../util/log.h"

#define TX_ENABLE_FILE_LOG 0

#define USE_GSM_MULTIPATH 1
#define SNR 8

// Struct which stores data necessary for simulation
// i.e. two buffers and a channel object
struct simu_data_s {
	channel_cccf tx_channel; // liquidsdr channel object
	float complex* tx_prep_buf;	// buffer to hold prepared tx data
	float complex* tx_dest;		// pointer to a remote rx buffer
	float complex* rxbuf;		// buffer to hold rx data
	pthread_cond_t rx_cond;		// condition in order to wait for a remote tx thread to fill the buffer
	pthread_mutex_t rx_lock;
	uint buflen;
};

typedef struct simu_data_s* simu_data;

int simulation_receive(platform p, float complex* buf)
{
	simu_data data = ((simu_data)p->data);
	memcpy(buf,data->rxbuf,data->buflen*sizeof(float complex));
	return 1;
}

int simulation_prep_tx(platform p, float complex* buf, uint offset, uint num_samples)
{
	simu_data data = ((simu_data)p->data);
	if (data->buflen<offset+num_samples) {
		LOG(ERR,"[PLATFORM] Buffer boundary violation when writing to %d in buf.\n",offset+num_samples);
		return 0;
	}
	memcpy(data->tx_prep_buf+offset,buf,num_samples*sizeof(float complex));
	return 1;
}

int simulation_tx(platform p)
{
	simu_data data = ((simu_data)p->data);
	if (data->tx_dest) {
		channel_cccf_execute_block(data->tx_channel, data->tx_prep_buf, data->buflen, data->tx_dest);
	}
#if TX_ENABLE_FILE_LOG
	log_bin((uint8_t*)data->tx_prep_buf,sizeof(float complex)*data->buflen, "dldata.bin","a");
#endif
	return 1;
}

void sim_end(platform p)
{
	free(((simu_data)p->data)->rxbuf);
	free(((simu_data)p->data)->tx_prep_buf);
	channel_cccf_destroy(((simu_data)p->data)->tx_channel);
}

// Connect two simulation platform instances
// the tx_push functions of the simulation instances
// will write data to the other's rx bufferr from now on
void simulation_connect(platform p, platform remote)
{
	simu_data p_data = (simu_data)p->data;
	simu_data remote_data = (simu_data)remote->data;

	p_data->tx_dest = remote_data->rxbuf;
	remote_data->tx_dest = p_data->rxbuf;
}

platform platform_init_simulation(uint buflen)
{
	// Generate platform interface
	platform sim = malloc(sizeof(struct platform_s));
	simu_data sim_data = malloc(sizeof(struct simu_data_s));

	// Set the functions
	sim->platform_rx = simulation_receive;
	sim->platform_tx_prep = simulation_prep_tx;
	sim->platform_tx_push = simulation_tx;
	sim->end = sim_end;
	sim->data = sim_data;

	// Generate buffers
	sim_data->rxbuf = malloc(sizeof(float complex)*buflen);
	sim_data->tx_prep_buf = malloc(sizeof(float complex)*buflen);
	sim_data->tx_dest = NULL;
	sim_data->buflen = buflen;

	// Generate simulation channel
    channel_cccf channel = channel_cccf_create();

    // AWGN
    float noise_floor = -60.0;
    float snr = SNR;
    channel_cccf_add_awgn(channel,noise_floor, snr);

    // Multipath
    // GSM typical urban 12 tap scenario 1
	#if USE_GSM_MULTIPATH
    float complex gsmTUx12c1[] = {   0.0010 + 0.0013i,
    		   0.0020 + 0.0079i,
    		  -0.0092 - 0.0218i,
    		   0.0111 + 0.0325i,
    		  -0.0154 - 0.0500i,
    		   0.0226 + 0.0807i,
    		  -0.0373 - 0.1543i,
    		   0.1526 + 0.9226i,
    		   0.8225 + 0.4973i,
    		  -0.0054 - 0.0771i,
    		   0.0166 + 0.0542i,
    		  -0.0130 - 0.0359i,
    		   0.0098 + 0.0238i,
    		  -0.0072 - 0.0154i,
    		   0.0052 + 0.0095i,
    		  -0.0035 - 0.0054i,
    		  -0.0012 - 0.0006i,
    		   0.0000 + 0.0000i
    };
    uint gsmTUx12c1_len = 18;
    channel_cccf_add_multipath(channel, gsmTUx12c1, gsmTUx12c1_len);
	#else
    float complex hc[] = {.1, 1, 0, .1};
    uint hc_len = 4;		// number of channel filter taps
    channel_cccf_add_multipath(channel, hc, hc_len);
	#endif
    // frequency offset
    float cfo = 50;	// frequency offset in Hertz
    float dphi = (2*3.1415*cfo)/256000.0;	//frequency offset in radians/sample
    float phi = 2;		// phase offset in radians
    channel_cccf_add_carrier_offset(channel, dphi, phi);

    // slow-flat fading
    //float sigma = 1;	// standard deviation for log-normal shadowing
    //float fd = 20.0/256000.0;		// relative Doppler frequency
    //channel_cccf_add_shadowing(channel, sigma, fd);

	sim_data->tx_channel = channel;

	return sim;
}
