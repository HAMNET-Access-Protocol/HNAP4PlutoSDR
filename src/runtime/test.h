/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA
 */

#ifndef RUNTIME_TEST_H_
#define RUNTIME_TEST_H_

#include <stdint.h>
#include "../phy/phy_config.h"

#define MAX_SLOT_DATA 300

// Store binary data that is sent at phy layer to calculate BER
extern uint8_t phy_dl[FRAME_LEN][4][MAX_SLOT_DATA];
extern uint8_t phy_ul[FRAME_LEN][4][MAX_SLOT_DATA];

extern uint32_t phy_ul_tot_bits;
extern uint32_t phy_ul_biterr;

extern uint32_t phy_dl_tot_bits;
extern uint32_t phy_dl_biterr;

// store timestamps when mac payload was sent/received
#define num_simulated_subframes 75000
extern int mac_dl_timestamps[num_simulated_subframes];
extern int mac_ul_timestamps[num_simulated_subframes];

// global subframe and slot counter to calculate MAC delay
extern uint global_sfn;
extern uint global_symbol;

#endif /* RUNTIME_TEST_H_ */
