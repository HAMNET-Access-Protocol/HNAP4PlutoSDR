/*
 * test.h
 *
 *  Created on: Feb 3, 2020
 *      Author: lukas
 */

#ifndef RUNTIME_TEST_H_
#define RUNTIME_TEST_H_

#include <stdint.h>
#include "../phy/phy_config.h"

#define MAX_SLOT_DATA 256

// Store binary data that is sent at phy layer to calculate BER
extern uint8_t phy_dl[FRAME_LEN][4][MAX_SLOT_DATA];
extern uint8_t phy_ul[FRAME_LEN][4][MAX_SLOT_DATA];

extern uint32_t phy_ul_tot_bits;
extern uint32_t phy_ul_biterr;

extern uint32_t phy_dl_tot_bits;
extern uint32_t phy_dl_biterr;

// store timestamps when mac payload was sent/received
#define num_simulated_subframes 3000
extern int mac_dl_timestamps[num_simulated_subframes];
extern int mac_ul_timestamps[num_simulated_subframes];

// global subframe and slot counter to calculate MAC delay
extern uint global_sfn;
extern uint global_symbol;

#endif /* RUNTIME_TEST_H_ */
