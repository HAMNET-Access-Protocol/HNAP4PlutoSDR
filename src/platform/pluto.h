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

// maximum and minimum rxgain and txgain values
#define RXGAIN_MAX 73
#define RXGAIN_MIN -1
#define TXGAIN_MAX 0
#define TXGAIN_MIN -89

// The number of kernel buffers for TX and RX
#define KERNEL_BUF_TX 4
#define KERNEL_BUF_RX 6

// adjust the delay between generation of the signal in software and
// the hardware toggle. A default delay is calculated from the kernel buffer
// length, this variable can be used for fine tuning
#define DEFAULT_PTT_DELAY_COMP 200 // [usec]

// Pluto Platform hardware abstraction
// use init pluto platform, to generate a platform
// abstraction. See platform.h on how to use it
platform init_pluto_platform(unsigned int buf_len, char* config_file);

platform init_pluto_network_platform(unsigned int buf_len);

// ---------------- Configuration --------------------- //
void pluto_set_rxgain(platform hw, int gain);
void pluto_set_txgain(platform hw, int gain);

int pluto_set_rx_freq(platform hw, long long rxfreq);
int pluto_set_tx_freq(platform hw, long long txfreq);

// Duplex mode config
int pluto_enable_ptt(platform hw);
void pluto_ptt_set_tx(platform hw);
void pluto_ptt_set_rx(platform hw);
void pluto_ptt_set_switch_delay(platform hw, int delay_us);

// --------------- Read device config ----------------- //
long long pluto_get_rxgain(platform hw);

// start a thread that monitors for Buffer over/underflows
pthread_t pluto_start_monitor(platform hw);

#endif /* PLATFORM_PLUTO_H_ */
