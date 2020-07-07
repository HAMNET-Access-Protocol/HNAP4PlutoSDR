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

#ifndef PHY_OFDM_COMMON_H_
#define PHY_OFDM_COMMON_H_

#include "../mac/mac_channels.h"
#include "phy_config.h"

#include <liquid/liquid.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// number of defined MCS schemes
// has to match the number of define schemes in phy_common_init()
#define NUM_MCS_SCHEMES 7

// log makro to log with subframe number
#define LOG_SFN_PHY(level, ...)                                                \
  do {                                                                         \
    if (level >= global_log_level) {                                           \
      printf("[%2d %2d]", phy->common->rx_subframe, phy->common->rx_symbol);   \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0)

// typedef for easier dlctrl slot access
typedef union {
  uint8_t byte;
  struct {
    uint8_t h4 : 4;
    uint8_t l4 : 4;
  };
} dlctrl_alloc_t;

// Struct contains PHY variables common to UE and BS Phy layer
typedef struct {
  uint rx_subframe;
  uint rx_symbol;
  uint tx_subframe;
  uint tx_symbol;
  uint tx_active; // for UEs, TX is only activated after sync is established

  uint8_t *pilot_sc;         // defines which subcarriers are used for pilots
  uint8_t *pilot_symbols_rx; // stores which OFDM symbols in a subframe contain
                             // pilots.
  uint8_t *pilot_symbols_tx;

  // hold TX data in frequency domain
  // 1. Index: subframe index: 0 for even subframes, 1 for uneven
  // 2. Index: ofdm symbol idx.
  // 3. Index: subcarrier idx.
  float complex ***txdata_f;
  // hold RX data in frequency domain
  // 1. Index: ofdm symbol number
  // 2. Index subcarrier idx
  float complex **rxdata_f;

  modem mcs_modem[8]; // array of modems for different mcs
  fec fec_ctrl;   // ctrl slots are encoded with MCS 0. we add a separate coder,
                  // because data and control slots might be decoded in parallel
                  // (multithreading) and cannot use the same coder
  fec mcs_fec[8]; // array of encoders/decoders for different mcs
  fec_scheme mcs_fec_scheme[8];

  interleaver mcs_interlvr[8]; // array of interleavers for different mcs

} PhyCommon_s;

typedef PhyCommon_s *PhyCommon;

// Create the common phy struct
PhyCommon phy_common_init();

void phy_common_destroy(PhyCommon phy);

// returns the Transport Block size of a UL/DL data slot in bits
int get_tbs_size(PhyCommon phy, uint mcs);

// returns the size of an UL control slot in bits
int get_ulctrl_slot_size(PhyCommon phy);

// Modulate the given data to the frequency domain data of the Phy object
// returns the number of symbols that have been generated
void phy_mod(PhyCommon common, uint subframe, uint first_sc, uint last_sc,
             uint first_symb, uint last_symb, uint mcs, uint8_t *data,
             uint buf_len, uint *written_samps);

// Symbol demapper with soft decision
// returns an array with n llr values for each demapped symbol and the number of
// demapped bits
void phy_demod_soft(PhyCommon common, uint first_sc, uint last_sc,
                    uint first_symb, uint last_symb, uint mcs, uint8_t *llr,
                    uint num_llr, uint *written_samps);

// Define which OFDM symbols whithin a subframe contain pilots
void gen_pilot_symbols(PhyCommon phy, uint is_bs);
void gen_pilot_symbols_robust(PhyCommon phy, uint is_bs);
void gen_pilot_symbols_robust2(PhyCommon phy, uint is_bs);

#endif /* PHY_COMMON_H_ */
