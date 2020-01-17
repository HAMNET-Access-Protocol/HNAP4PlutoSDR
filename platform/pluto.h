/*
 * pluto.h
 *
 *  Created on: 26.12.2019
 *      Author: lukas
 */

#ifndef PLATFORM_PLUTO_H_
#define PLATFORM_PLUTO_H_

#include <stdio.h>
#include <iio.h>

#include "platform.h"
#include "../phy/phy_config.h"

void shutdown();
void init_pluto_platform(uint buf_len);
void pluto_transmit();
int pluto_prep_tx(float complex* buf_tx, uint offset, uint num_samples);
int pluto_receive(float complex* buf_rx);

#endif /* PLATFORM_PLUTO_H_ */
