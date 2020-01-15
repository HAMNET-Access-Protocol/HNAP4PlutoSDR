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

// forward declaration of mac struct
struct MacBS_s;

struct PhyBS_s {
	PhyCommon common;			// pointer to common phy objects
	ofdmframegen fg;			// OFDM frame generator object
	ofdmframesync fs[MAX_USER];	// OFDM frame receiver objects for each user
	ofdmframesync fs_rach; 		// OFDM receiver for rach slot

	uint8_t ul_slot_alloc[4];	// holds userID allocation for uplink slots
	uint8_t ulctrl_slot_allot[2];

	dlctrl_alloc_t* dlctrl_buf;	// holds DL ctrl slot data

	uint8_t* rach_buffer;		// buffer stores data which is sent by users during RACH procedure

	struct MacBS_s* mac;
};

typedef struct PhyBS_s* PhyBS;

PhyBS phy_init_bs();

void phy_make_syncsig(PhyBS phy, float complex* txbuf_time);
void phy_write_subframe(PhyBS phy, float complex* txbuf_time);
int phy_map_dlslot(PhyBS phy, LogicalChannel chan, uint8_t slot_nr, uint userid, uint mcs);
void phy_map_dlctrl(PhyBS phy);
void phy_assign_dlctrl_ud(PhyBS phy, uint8_t* slot_assignment);
void phy_assign_dlctrl_uc(PhyBS phy, uint8_t* slot_assignment);
int phy_assign_dlctrl_ud_user(PhyBS phy, uint8_t* assigned_slots, uint userid);
int phy_assign_dlctrl_uc_user(PhyBS phy, uint8_t* assigned_slots, uint userid);
int phy_bs_rx_subframe(PhyBS phy, float complex* rxbuf_time);

#endif /* PHY_BS_H_ */
