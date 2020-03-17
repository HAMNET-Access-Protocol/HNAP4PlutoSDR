/*
 * pluto.h
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#ifndef PLATFORM_PLUTO_H_
#define PLATFORM_PLUTO_H_

#include "platform.h"
#include <pthread.h>

// The number of kernel buffers for TX and RX
#define KERNEL_BUF_TX 4
#define KERNEL_BUF_RX 6

// Pluto Platform hardware abstraction
// use init pluto platform, to generate a platform
// abstraction. See platform.h on how to use it
platform init_pluto_platform(unsigned int buf_len);

platform init_pluto_network_platform(unsigned int buf_len);

// ---------------- Configuration --------------------- //
void pluto_set_rxgain(int gain);
void pluto_set_txgain(int gain);

int pluto_set_rx_freq(long long rxfreq);
int pluto_set_tx_freq(long long txfreq);

int pluto_set_tx_off();
int pluto_set_tx_on();

// --------------- Read device config ----------------- //
long long pluto_get_rxgain();

// start a thread that monitors for Buffer over/underflows
pthread_t pluto_start_monitor();

#endif /* PLATFORM_PLUTO_H_ */
