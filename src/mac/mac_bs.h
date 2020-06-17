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

#ifndef MAC_MAC_BS_H_
#define MAC_MAC_BS_H_

#include "mac_config.h"
#include "mac_fragmentation.h"
#include "mac_common.h"
#include "tap_dev.h"

#include "../util/ringbuf.h"
#include <liquid/liquid.h>
#include <sys/queue.h>
#include "../phy/phy_bs.h"

enum {DL=0, UL};

// Struct represents an associated user
typedef struct {
	ofdmframesync fs;			// framesync object. Stores freq offset etc.
	uint8_t userid;
	int ul_queue;
	ringbuf msg_control_queue;
	MacFrag fragmenter;
	MacAssmbl reassembler;

	uint timingadvance;
	uint8_t dl_mcs;				// The mcs schemes used for the user
	uint8_t ul_mcs;

	uint8_t dl_mcs_pending;		// Variables used during MCS switch
	uint8_t ul_mcs_pending;
	long int dl_mcs_pending_time;
	long int ul_mcs_pending_time;

    MACstat_s stats;            // struct to collect statistics per user

    long unsigned int last_seen; // subframe No in which user has sent sth the last time
	uint8_t will_end;			 // flag is set to indicate that the connection will be ended
}user_s;

//forward declaration of phy struct that is needed in mac struct
struct PhyBS_s;

// Struct for linked list that contains etheraddr to userid mapping
SLIST_HEAD(slisthead, entry);
struct entry {
    char EtherAddr[6];
    uint userid;
    SLIST_ENTRY(entry) entries;
};

struct MacBS_s {
	ringbuf broadcast_ctrl_queue;
	MacFrag broadcast_data_fragmenter;
	user_s* UE[MAX_USER];

	tap_dev tapdevice;

	uint8_t ul_ctrl_assignments[FRAME_LEN][MAC_ULCTRL_SLOTS];
	uint8_t ul_data_assignments[FRAME_LEN][MAC_DLDATA_SLOTS];
	uint8_t dl_data_assignments[FRAME_LEN][MAC_ULDATA_SLOTS];

	struct PhyBS_s* phy;

    // Store mapping of EtherAddr to userid
    struct slisthead etheraddr_map;

	int last_added_rachuserid;
	int last_added_userid;

	// subframe counter. Note: this is not synchronized with other clients
	// every client starts with 0 once it synced. Used for MAC timers
	long long unsigned int subframe_cnt;
};

typedef struct MacBS_s* MacBS;

// ---------------------- Generic MAC ------------------------ //
MacBS mac_bs_init();
void mac_bs_destroy(MacBS mac);
void mac_bs_set_phy_interface(MacBS mac, struct PhyBS_s* phy);

// --------------- Interface functions for PHY --------------- //
ofdmframesync mac_bs_get_receiver(MacBS mac, uint userid);
void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, uint8_t rach_try_cnt, ofdmframesync fs, int timing_diff);
void mac_bs_update_timingadvance(MacBS mac, uint userid, int timing_diff);
int mac_bs_rx_channel(MacBS mac, LogicalChannel chan, uint userid);

// ----------- Interface functions for higher layer ---------- //
void mac_bs_set_mcs(MacBS mac, uint userid, uint mcs, uint dl_ul);
int mac_bs_add_txdata(MacBS mac, uint8_t destUserID, MacDataFrame frame);
void mac_bs_run_scheduler(MacBS mac);

void* mac_bs_tap_rx_th(void* mac);

#endif /* MAC_MAC_BS_H_ */
