/*
 * phy_bs.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_BS_H_
#define PHY_BS_H_

#include "phy_common.h"
#include "../mac/mac_bs.h"
#include "../platform/platform.h"
#include <pthread.h>

// forward declaration of mac struct
struct MacBS_s;

struct PhyBS_s {
	PhyCommon common;			// pointer to common phy objects
	ofdmframegen fg;			// OFDM frame generator object
	ofdmframesync fs_rach; 		// OFDM receiver for rach slot

	// Variables to store the slot assignments
	// 1. array index: 0 for even subframes, 1 for uneven subframe
	// 2. array index: slot index
	uint8_t** ulslot_assignments;
	uint8_t** ulctrl_assignments;

	// store uplink resource allocation on OFDM symbol basis
	// BS has to pick the correct ofdmframesync object depending on the user
	// 1. Index: subframe index: 0 -> even, 1->odd
	// 2. Index ofdm symbol idx
	uint8_t** ul_symbol_alloc;

	dlctrl_alloc_t* dlctrl_buf;	// holds DL ctrl slot data

	// buffer stores data which is sent by users during RACH procedure
	float complex rach_buffer[NFFT];

	// stores timing offset of a received RA message
	int rach_timing;
	// we have to store the remaining samps that have to be received after sync with a new user is achieved
	int rach_remaining_samps;

	struct MacBS_s* mac;
};

typedef struct PhyBS_s* PhyBS;

/*************** Initializer functions *********************/
PhyBS phy_bs_init();
void phy_bs_destroy(PhyBS phy);
void phy_bs_set_mac_interface(PhyBS phy, struct MacBS_s* mac);

/************* TX mapper functions *************************/
int phy_map_dlslot(PhyBS phy, LogicalChannel chan, uint subframe, uint8_t slot_nr, uint userid, uint mcs);
void phy_map_dlctrl(PhyBS phy, uint subframe);
void phy_assign_dlctrl_dd(PhyBS phy, uint8_t* slot_assignment);
void phy_assign_dlctrl_ud(PhyBS phy, uint subframe, uint8_t* slot_assignment);
void phy_assign_dlctrl_uc(PhyBS phy, uint subframe, uint8_t* slot_assignment);

/************** Main RX/TX functions ***********************/
void phy_bs_rx_symbol(PhyBS phy, float complex* rxbuf_time);
void phy_bs_write_symbol(PhyBS phy, float complex* txbuf_time);

#endif /* PHY_BS_H_ */
