/*
 * pluto.h
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#ifndef PLATFORM_PLUTO_H_
#define PLATFORM_PLUTO_H_

#include "platform.h"

// Pluto Platform hardware abstraction
// use init pluto platform, to generate a platform
// abstraction. See platform.h on how to use it
platform init_pluto_platform(unsigned int buf_len);

platform init_pluto_network_platform(unsigned int buf_len);

// ---------------- Configuration --------------------- //
void pluto_set_rxgain(int gain);
int pluto_set_rx_freq(long long rxfreq);
int pluto_set_tx_freq(long long txfreq);


#endif /* PLATFORM_PLUTO_H_ */
