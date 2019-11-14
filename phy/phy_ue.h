/*
 * phy_ue.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_UE_H_
#define PHY_UE_H_

#include "phy_common.h"

typedef enum {NO_SYNC, HAS_SYNC} phy_states;

typedef struct {
	PhyCommon common;	// pointer to common phy objects
	ofdmframegen fg;	// OFDM frame generator object
	ofdmframesync fs;	// OFDM frame receiver object

	// Variables to store the decoded slot assignments
	uint8_t* dlslot_assignments;
	uint8_t* ulslot_assignments;
	uint8_t* ulctrl_assignments;

	// Currently used modulation schemes.
	uint mcs_dl;
	uint mcs_ul;

	// MAC layer function that will be called when a slot was received
	void (*mac_rx_cb)(LogicalChannel);

	// old cfo estimate for filtering
	float prev_cfo;
	// Flag stores whether prev_cfo already holds an estimate, i.e. if we were synced before
	int has_synced_once;
} PhyUE_s;

typedef PhyUE_s* PhyUE;

/************ GENERAL PHY CONFIG FUNCTIONS **********************/
PhyUE phy_init_ue();
void phy_ue_set_mac_cb(PhyUE phy, void (*mac_rx_cb)(LogicalChannel));
void phy_ue_set_mcs_dl(PhyUE phy, uint mcs);
void phy_ue_set_mcs_ul(PhyUE phy, uint mcs);

/***************** PHY TX FUNCTIONS *****************************/
int phy_map_ulslot(PhyUE phy, LogicalChannel chan, uint8_t slot_nr, uint mcs);
int phy_map_ulctrl(PhyUE phy, LogicalChannel chan, uint8_t slot_nr);
void phy_ue_write_subframe(PhyUE phy, float complex* rxbuf_time);


/***************** PHY RX FUNCTIONS *****************************/
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples);
int phy_ue_proc_dlctrl(PhyUE phy);
void phy_ue_proc_slot(PhyUE phy, uint slotnr);
//void phy_ue_rx_subframe(PhyUE phy, float complex* rxbuf_time);
int _ofdm_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd);
void phy_ue_do_rx(PhyUE phy, float complex* rxbuf_time, uint num_samples);


#endif /* PHY_UE_H_ */
