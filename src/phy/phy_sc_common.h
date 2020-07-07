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

#ifndef PHY_SC_COMMON_H
#define PHY_SC_COMMON_H

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

#define SC_SYMBOL_UNIT 40

#define OSF 2            // over sampling factor
#define FILTER_BETA 0.3f // RRC filter bandwidth excess factor
#define FILTER_DELAY 5   // sample delay of the RRC filter

// log makro to log with subframe number
#define LOG_SFN_PHY(level, ...)                                                \
  do {                                                                         \
    if (level >= global_log_level) {                                           \
      printf("[%2d %2d]", phy->common->rx_subframe, phy->common->rx_symbol);   \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0)

// Zadoff Chu sequence 1,119
#define ZADOFF_CHU_LEN 120
float complex zadoff_chu_seq[ZADOFF_CHU_LEN];
float complex training_seq[SC_SYMBOL_UNIT];

// typedef for easier dlctrl slot access
typedef union {
  uint8_t byte;
  struct {
    uint8_t h4 : 4;
    uint8_t l4 : 4;
  };
} dlctrl_alloc_t;

// SC receiver states
enum sc_rec_states { SEARCH_ZADOFF, SYNC };

// SC receiver callback
typedef int (*sc_rec_callback)(float complex *_y, unsigned int _M,
                               void *_userdata);

// Struct contains objects required for singlecarrier receiving and
// synchronization
struct sc_receiver_s {
  enum sc_rec_states sync_state;
  int rx_timer;
  int sym_size;
  float complex *sym_buffer;
  windowcf sym_window;
  symsync_crcf symsync;
  detector_cccf sync_correlator;
  eqlms_cccf eqlms;

  sc_rec_callback cb;
  void *cb_userd;

  float g0; // initial gain estimate (from sync sequence)
};

typedef struct sc_receiver_s *sc_receiver;

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

  // RRC interpolation filter
  firinterp_cccf rrc_shaper;

  // hold TX data in time domain
  // 1. Index: subframe index: 0 for even subframes, 1 for uneven
  // 2. Index: symbol idx.
  float complex **txdata;
  // hold RX data in time domain
  // 1. Index:  symbol number
  float complex *rxdata;

  modem mcs_modem[8]; // array of modems for different mcs
  fec fec_ctrl; // ctrl slots are encoded with MCS 0. we add a separate coder,
                // because data and control slots
  // might be decoded in parallel (multithreading) and cannot use the same coder
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

void phy_mod(PhyCommon common, uint subframe, uint first_symb, uint last_symb,
             uint mcs, uint8_t *data, uint buf_len, uint *written_samps);
void phy_demod_soft(PhyCommon common, uint first_symb, uint last_symb, uint mcs,
                    uint8_t *llr, uint num_llr, uint *written_samps);

void phy_pulse_shaper(PhyCommon phy, float complex *sym_out,
                      float complex *samps_in, uint num_samps);
void phy_pulse_shaper_zeros(PhyCommon phy, float complex *sym_out,
                            int num_zeros);

sc_receiver sc_receiver_create(int sym_len, sc_rec_callback cb, void *cb_userd);
void sc_receiver_set_cb(sc_receiver rec, sc_rec_callback cb, void *cb_userd);
void sc_receiver_destroy(sc_receiver rec);
void sc_receiver_reset(sc_receiver rec);
void sc_receiver_execute(sc_receiver rec, float complex *samples,
                         int num_samps);
void sc_receiver_eq_update(sc_receiver rec, float complex *samples,
                           int num_samps);
void sc_receiver_equalize(sc_receiver rec, float complex *samples,
                          int num_samps);
int sc_receiver_get_sync(sc_receiver rec, float complex *samples,
                         int num_samps);

#endif // TRANSCEIVER_PHY_SC_COMMON_H
