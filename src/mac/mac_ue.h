/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef MAC_MAC_UE_H_
#define MAC_MAC_UE_H_

#include "mac_channels.h"
#include "mac_common.h"
#include "mac_config.h"
#include "mac_fragmentation.h"
#include "tap_dev.h"

struct PhyUE_s;
typedef struct PhyUE_s *PhyUE;

struct MacUE_s {
  uint8_t userid;

  uint8_t dl_mcs;
  uint8_t ul_mcs;
  uint timing_advance;

  ringbuf msg_control_queue;
  MacFrag fragmenter;
  MacAssmbl reassembler;       // reassembles unicast frames
  MacAssmbl reassembler_brcst; // reassemble broadcast frames
  tap_dev tapdevice;

  uint8_t
      ul_ctrl_assignments[MAC_ULCTRL_SLOTS]; // TODO the assignments are already
                                             // defined in PHY instance
  uint8_t ul_data_assignments[MAC_ULDATA_SLOTS];
  uint8_t dl_data_assignments[MAC_DLDATA_SLOTS];

  uint8_t is_associated;

  long long unsigned int
      last_assignment; // Subframe in which user was assigned a slot the last
                       // time used to detect disconnection
  long long unsigned int subframe_cnt;

  struct PhyUE_s *phy;

  MACstat_s stats;
};

typedef struct MacUE_s *MacUE;

/************ GENERAL MAC CONFIG FUNCTIONS **********************/
MacUE mac_ue_init();
void mac_ue_destroy(MacUE mac);
void mac_ue_set_phy_interface(MacUE mac, PhyUE phy);

/************** MAC INTERFACE FUNCTIONS *************************/
void mac_ue_set_assignments(MacUE mac, uint8_t *dlslot, uint8_t *ulslot,
                            uint8_t *ulctrl);
void mac_ue_run_scheduler(MacUE mac);
void mac_ue_rx_channel(MacUE mac, LogicalChannel chan, uint is_broadcast);
int mac_ue_add_txdata(MacUE mac, MacDataFrame frame);
void mac_ue_req_mcs_change(MacUE mac, uint mcs, uint is_ul);

/*** Getter functions ***/
int mac_ue_is_associated(MacUE mac);
int mac_ue_get_timing_advance(MacUE mac);

void *mac_ue_tap_rx_th(void *mac);

#endif /* MAC_MAC_UE_H_ */
