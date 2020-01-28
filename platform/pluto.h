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

// set the rxgain of the platform
void pluto_set_rxgain(int gain);

#endif /* PLATFORM_PLUTO_H_ */
