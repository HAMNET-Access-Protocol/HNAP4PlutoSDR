/*
 * phy_ue.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_UE_H_
#define PHY_UE_H_

#include "phy_common.h"
#include "../platform/platform.h"

typedef enum {NO_SYNC, HAS_SYNC} phy_states;

// forward declaration of Mac struct
struct MacUE_s;

struct PhyUE_s {
	PhyCommon common;	// pointer to common phy objects
	ofdmframegen fg;	// OFDM frame generator object
	ofdmframesync fs;	// OFDM frame receiver object

	// Variables to store the decoded slot assignments.
	// Set to 1 if the corresponding slot is allocated for this client, 0 otherwise
	// 1. array index: 0 for even subframes, 1 for uneven subframe
	// 2. array index: slot index
	uint8_t** dlslot_assignments;
	uint8_t** ulslot_assignments;
	uint8_t** ulctrl_assignments;

	// store resource allocation on OFDM symbol basis
	// UE has to refrain from sending if no data is allocated
	// 1. Index: subframe index: 0 -> even, 1->odd
	// 2. Index ofdm symbol idx
	uint8_t** ul_symbol_alloc;

	// Currently used modulation scheme for RX
	uint mcs_dl;

	// MAC layer function that will be called when a slot was received
	void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel);
	struct MacUE_s* mac;	// Pointer to MAC layer. Needed to call mac interface function

	// old cfo estimate for filtering
	float prev_cfo;
	// Flag stores whether prev_cfo already holds an estimate, i.e. if we were synced before
	int has_synced_once;

	// Sample offset between buffer start and start of subframe
	int rx_offset;

	// random userid which is used during RA procedure
	int rachuserid;
	// assigned userid
	int userid;

	// Locks and Condition variables for RX/TX thread sync
	pthread_mutex_t tx_sync_lock;
	pthread_cond_t tx_sync_cond;
};

typedef struct PhyUE_s* PhyUE;

/************ GENERAL PHY CONFIG FUNCTIONS **********************/
PhyUE phy_ue_init();
void phy_ue_set_mac_interface(PhyUE phy, void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel), struct MacUE_s* mac);
void phy_ue_set_mcs_dl(PhyUE phy, uint mcs);
void phy_ue_reset_symbol_allocation(PhyUE phy, uint subframe);

/***************** PHY TX FUNCTIONS *****************************/
int phy_map_ulslot(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr, uint mcs);
int phy_map_ulctrl(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr);
void phy_ue_write_subframe(PhyUE phy, float complex* rxbuf_time);
void phy_ue_write_symbol(PhyUE phy, float complex* txbuf_time);


/***************** PHY RX FUNCTIONS *****************************/
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples);
int phy_ue_proc_dlctrl(PhyUE phy);
void phy_ue_proc_slot(PhyUE phy, uint slotnr);
//void phy_ue_rx_subframe(PhyUE phy, float complex* rxbuf_time);
void phy_ue_do_rx(PhyUE phy, float complex* rxbuf_time, uint num_samples);


#endif /* PHY_UE_H_ */
