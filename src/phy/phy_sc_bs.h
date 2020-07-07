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

#ifndef PHY_SC_BS_H
#define PHY_SC_BS_H

#include "../mac/mac_bs.h"
#include "../platform/platform.h"
#include "phy_sc_common.h"
#include <pthread.h>

// forward declaration of mac struct
struct MacBS_s;

struct PhyBS_s {
  // pointer to common phy objects
  PhyCommon common;

  sc_receiver rec_rach; // receiver object for RA slot

  // Variables to store the slot assignments
  // 1. array index: 0 for even subframes, 1 for uneven subframe
  // 2. array index: slot index
  uint8_t **ulslot_assignments;
  uint8_t **ulctrl_assignments;

  // store uplink resource allocation on OFDM symbol basis
  // BS has to pick the correct ofdmframesync object depending on the user
  // 1. Index: subframe index: 0 -> even, 1->odd
  // 2. Index ofdm symbol idx
  uint8_t **ul_symbol_alloc;
  uint8_t **dl_symbol_alloc;

  dlctrl_alloc_t *dlctrl_buf; // holds DL ctrl slot data

  // buffer stores data which is sent by users during RACH procedure
  float complex *rach_buffer;

  // stores timing offset of a received RA message
  int rach_timing;

  struct MacBS_s *mac;

  // Pthread condition used to trigger slot processing
  pthread_cond_t *rx_slot_signal;
  // indicates received slot nr to the slot processing thread
  int rx_slot_nr;

  // current rx and txgain values. Broadcasted in the sync slot
  int8_t rxgain;
  int8_t txgain;
};

typedef struct PhyBS_s *PhyBS;

/*************** Initializer functions *********************/
PhyBS phy_bs_init();
void phy_bs_destroy(PhyBS phy);
void phy_bs_set_mac_interface(PhyBS phy, struct MacBS_s *mac);
void phy_bs_set_rx_slot_th_signal(PhyBS phy, pthread_cond_t *cond);

/************* TX mapper functions *************************/
int phy_map_dlslot(PhyBS phy, LogicalChannel chan, uint subframe,
                   uint8_t slot_nr, uint userid, uint mcs);
void phy_map_dlctrl(PhyBS phy, uint subframe);
void phy_assign_dlctrl_dd(PhyBS phy, uint subframe, uint8_t *slot_assignment);
void phy_assign_dlctrl_ud(PhyBS phy, uint subframe, uint8_t *slot_assignment);
void phy_assign_dlctrl_uc(PhyBS phy, uint subframe, uint8_t *slot_assignment);

/************** Main RX/TX functions ***********************/
void phy_bs_rx_symbol(PhyBS phy, float complex *rxbuf_time);
void phy_bs_write_symbol(PhyBS phy, float complex *txbuf_time);
void phy_bs_proc_slot(PhyBS phy, uint slotnr);

#endif // TRANSCEIVER_PHY_SC_BS_H
