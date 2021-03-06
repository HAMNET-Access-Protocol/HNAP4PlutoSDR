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

#ifndef PHY_UE_H_
#define PHY_UE_H_

#include "phy_common.h"
#include "../platform/platform.h"
#include <pthread.h>

typedef enum {NO_SYNC, HAS_SYNC} phy_states;

// definition of slot assignments types
typedef enum {NOT_ASSIGNED, UE_ASSIGNED, BRCST_ASSIGNED} assignment_t;

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
    void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel, uint is_broadcast);
	struct MacUE_s* mac;	// Pointer to MAC layer. Needed to call mac interface function

	// Pointer to platform object
	struct platform_s* platform;

	// old cfo estimate for filtering
	float prev_cfo;
	// Flag stores whether prev_cfo already holds an estimate, i.e. if we were synced before
	int has_synced_once;

	// Sample offset between buffer start and start of subframe
	int rx_offset;

	// random userid which is used during RA procedure
	int rachuserid;
	// count how often we tried to associate
	int rach_try_cnt;
	// assigned userid
	int userid;

	//define mutex and condition object to run slot processing in separate thread
	pthread_cond_t* rx_slot_signal;
	uint rx_slot_nr;

    // store rx and txgain values from basestation sync signal
    int8_t bs_rxgain;
    int8_t bs_txgain;
    float rssi; // client rssi
};

typedef struct PhyUE_s* PhyUE;

/************ GENERAL PHY CONFIG FUNCTIONS **********************/
PhyUE phy_ue_init();
void phy_ue_destroy(PhyUE phy);
void phy_ue_set_rx_slot_th_signal(PhyUE phy, pthread_cond_t* cond);
void phy_ue_set_mac_interface(PhyUE phy, void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel, uint), struct MacUE_s* mac);
void phy_ue_set_platform_interface(PhyUE phy, struct platform_s* platform);


/********** INTERFACE FUNCTIONS TO MAC LAYER **********************/
// Configuration
int phy_ue_set_mcs_dl(PhyUE phy, uint mcs);
void phy_ue_reset_symbol_allocation(PhyUE phy, uint subframe);

// PHY data mapping
int phy_map_ulslot(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr, uint mcs);
int phy_map_ulctrl(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr);

// PHY slot processing
int phy_ue_proc_dlctrl(PhyUE phy);
void phy_ue_proc_slot(PhyUE phy, uint slotnr);

/***************** PHY RX/TX FUNCTIONS *****************************/
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples);
void phy_ue_do_rx(PhyUE phy, float complex* rxbuf_time, uint num_samples);

void phy_ue_write_symbol(PhyUE phy, float complex* txbuf_time);



#endif /* PHY_UE_H_ */
