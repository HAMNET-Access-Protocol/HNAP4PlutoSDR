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

#include "phy_common.h"
#include "phy_config.h"

// Init the PHY instance
PhyCommon phy_common_init() {
  PhyCommon phy = calloc(sizeof(PhyCommon_s), 1);

  // TX buffer has 2 entries for even/uneven subframes
  phy->txdata_f = malloc(sizeof(float complex **) * 2);

  // alloc buffer for one subframe of symbols in frequency domain
  phy->txdata_f[0] = malloc(sizeof(float complex *) * SUBFRAME_LEN);
  phy->txdata_f[1] = malloc(sizeof(float complex *) * SUBFRAME_LEN);
  phy->rxdata_f = malloc(sizeof(float complex *) * SUBFRAME_LEN);
  for (int i = 0; i < SUBFRAME_LEN; i++) {
    phy->txdata_f[0][i] = calloc(sizeof(float complex) * (nfft), 1);
    phy->txdata_f[1][i] = calloc(sizeof(float complex) * (nfft), 1);
    phy->rxdata_f[i] = calloc(sizeof(float complex) * (nfft), 1);
  }

  // alloc buffer for subcarrier definitions
  phy->pilot_sc = calloc(nfft, 1);
  phy->pilot_symbols_rx = calloc(SUBFRAME_LEN, 1);
  phy->pilot_symbols_tx = calloc(SUBFRAME_LEN, 1);

  // init modulator objects
  phy->mcs_modem[0] = modem_create(LIQUID_MODEM_QPSK);
  phy->mcs_modem[1] = modem_create(LIQUID_MODEM_QPSK);
  phy->mcs_modem[2] = modem_create(LIQUID_MODEM_QAM16);
  phy->mcs_modem[3] = modem_create(LIQUID_MODEM_QAM16);
  phy->mcs_modem[4] = modem_create(LIQUID_MODEM_QAM64);
  phy->mcs_modem[5] = modem_create(LIQUID_MODEM_QAM64);
  phy->mcs_modem[6] = modem_create(LIQUID_MODEM_QAM256);

  // init FEC modules
  phy->fec_ctrl = fec_create(LIQUID_FEC_CONV_V27, NULL);
  phy->mcs_fec[0] = fec_create(LIQUID_FEC_CONV_V27, NULL);
  phy->mcs_fec[1] = fec_create(LIQUID_FEC_CONV_V27P34, NULL);
  phy->mcs_fec[2] = fec_create(LIQUID_FEC_CONV_V27, NULL);
  phy->mcs_fec[3] = fec_create(LIQUID_FEC_CONV_V27P34, NULL);
  phy->mcs_fec[4] = fec_create(LIQUID_FEC_CONV_V27, NULL);
  phy->mcs_fec[5] = fec_create(LIQUID_FEC_CONV_V27P34, NULL);
  phy->mcs_fec[6] = fec_create(LIQUID_FEC_CONV_V27, NULL);

  phy->mcs_fec_scheme[0] = LIQUID_FEC_CONV_V27;
  phy->mcs_fec_scheme[1] = LIQUID_FEC_CONV_V27P34;
  phy->mcs_fec_scheme[2] = LIQUID_FEC_CONV_V27;
  phy->mcs_fec_scheme[3] = LIQUID_FEC_CONV_V27P34;
  phy->mcs_fec_scheme[4] = LIQUID_FEC_CONV_V27;
  phy->mcs_fec_scheme[5] = LIQUID_FEC_CONV_V27P34;
  phy->mcs_fec_scheme[6] = LIQUID_FEC_CONV_V27;

  // init subframe number and rx symbol nr
  phy->rx_subframe = 0;
  phy->rx_symbol = 0;
  phy->tx_subframe = 0;
  phy->tx_symbol = 0;

  // init the interleaver
  for (int mcs = 0; mcs < NUM_MCS_SCHEMES; mcs++) {
    uint payload_size = get_tbs_size(phy, mcs) / 8;
    uint enc_size =
        fec_get_enc_msg_length(phy->mcs_fec_scheme[mcs], payload_size);
    phy->mcs_interlvr[mcs] = interleaver_create(enc_size);
  }

  return phy;
}

void phy_common_destroy(PhyCommon phy) {
  // free buffer for symbols in frequency domain
  for (int i = 0; i < SUBFRAME_LEN; i++) {
    free(phy->txdata_f[0][i]);
    free(phy->txdata_f[1][i]);
    free(phy->rxdata_f[i]);
  }
  free(phy->txdata_f[0]);
  free(phy->txdata_f[1]);

  free(phy->rxdata_f);
  free(phy->txdata_f);

  // free buffer for subcarrier definitions
  free(phy->pilot_sc);
  free(phy->pilot_symbols_rx);
  free(phy->pilot_symbols_tx);

  // delete modulator, fec and interleaver objects
  for (int i = 0; i < NUM_MCS_SCHEMES; i++) {
    modem_destroy(phy->mcs_modem[i]);
    fec_destroy(phy->mcs_fec[i]);
    interleaver_destroy(phy->mcs_interlvr[i]);
  }
  free(phy);
}

// returns the Transport Block size of a UL/DL data slot in bits
int get_tbs_size(PhyCommon phy, uint mcs) {
  uint symbols =
      (SLOT_LEN - pilot_symbols_per_slot) * (num_data_sc + num_pilot_sc) +
      pilot_symbols_per_slot * num_data_sc;
  uint enc_bits =
      symbols * modem_get_bps(phy->mcs_modem[mcs]); // number of encoded bits
  return (enc_bits - 16) *
         fec_get_rate(phy->mcs_fec_scheme[mcs]); // real tbs size. Subtract
                                                 // 16bit for conv encoding
}

// returns the size of the ULCTRL slots in bits
int get_ulctrl_slot_size(PhyCommon phy) {
  uint symbols = num_data_sc;
  uint enc_bits =
      symbols * modem_get_bps(phy->mcs_modem[0]); // number of encoded bits
  return (enc_bits - 16) *
         fec_get_rate(phy->mcs_fec_scheme[0]); // real tbs size. Subtract 16bit
                                               // for conv encoding
}

/* Modulate the given data to the frequency domain data of the Phy object
 * returns the number of symbols that have been generated
 * Params:	common: 	pointer to the common phy struct
 *			subframe:	subframe number. Currently only even and
 *uneven (0/1) is defined first_sc:	index of the first subcarrier that shall
 *be used last_sc:	index of the last subcarrier that shall be used.
 *			first_symb:	index of the first symbol within the
 *subframe that shall be mapped to last_symb:	index of the last symbol mcs:
 *the MCS index that shall be used
 *			data:		array of symbols that will be modulated
 *			buf_len:	length of the data array
 *
 * Returns:	written_samps:	the number of symbols that have been generated
 */
void phy_mod(PhyCommon common, uint subframe, uint first_sc, uint last_sc,
             uint first_symb, uint last_symb, uint mcs, uint8_t *data,
             uint buf_len, uint *written_samps) {
  *written_samps = 0;
  for (int sym_idx = first_symb; sym_idx <= last_symb; sym_idx++) {
    for (int i = first_sc; i <= last_sc; i++) {
      if ((common->pilot_symbols_tx[sym_idx] == NO_PILOT &&
           !(common->pilot_sc[i] == OFDMFRAME_SCTYPE_NULL)) ||
          (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA)) {
        modem_modulate(common->mcs_modem[mcs], (uint)data[(*written_samps)++],
                       &common->txdata_f[subframe][sym_idx][i]);
        if (*written_samps >= buf_len) {
          return;
        }
      }
    }
  }
}

// Symbol demapper with soft decision
// returns an array with n llr values for each demapped symbol and the number of
// demapped bits
void phy_demod_soft(PhyCommon common, uint first_sc, uint last_sc,
                    uint first_symb, uint last_symb, uint mcs, uint8_t *llr,
                    uint num_llr, uint *written_samps) {
  *written_samps = 0;
  uint bps = modem_get_bps(common->mcs_modem[mcs]);

  // demodulate signal
  uint symbol = 0;
  for (int sym_idx = first_symb; sym_idx <= last_symb; sym_idx++) {
    for (int i = first_sc; i <= last_sc; i++) {
      if ((common->pilot_symbols_rx[sym_idx] == NO_PILOT &&
           !(common->pilot_sc[i] == OFDMFRAME_SCTYPE_NULL)) ||
          (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA)) {
        modem_demodulate_soft(common->mcs_modem[mcs],
                              common->rxdata_f[sym_idx][i], &symbol,
                              &llr[*written_samps]);
        *written_samps += bps;
        if (*written_samps + bps >= num_llr) {
          return;
        }
      }
    }
  }
}

void gen_pilot_symbols(PhyCommon phy, uint is_bs) {
  // load subcarrier allocation from phy config
  memcpy(phy->pilot_sc, subcarrier_alloc, nfft);

  // create time domain distribution of ofdm pilots within subcarrier
  // UE transmits in UL and RXs in DL, BS the other way around
  // define pilots accordingly
  uint8_t *pilot_ul, *pilot_dl;
  if (is_bs) {
    pilot_dl = phy->pilot_symbols_tx;
    pilot_ul = phy->pilot_symbols_rx;
  } else {
    pilot_dl = phy->pilot_symbols_rx;
    pilot_ul = phy->pilot_symbols_tx;
  }

  // Reset pilot allocation
  memset(pilot_dl, NO_PILOT, SUBFRAME_LEN);
  memset(pilot_ul, NO_PILOT, SUBFRAME_LEN);

  // DL: dlctrl slot uses pilots
  pilot_dl[0] = PILOT_RESET;
  pilot_dl[1] = PILOT;

  // replicate slot allocation for one slot over the subframe
  for (int slot_nr = 0; slot_nr < NUM_SLOT; slot_nr++) {
    int slot_start = DLCTRL_LEN + 2 * SLOT_GUARD_INTERVAL +
                     slot_nr * (SLOT_LEN + SLOT_GUARD_INTERVAL);
    memcpy(&pilot_dl[slot_start], pilot_symbols, SLOT_LEN);
    pilot_dl[slot_start] = PILOT_RESET; // force first symbol of the slot to
                                        // contain the pilot sequence reset flag
  }

  // Pilot symbols within subframe in UL
  // uldata slots
  memcpy(&pilot_ul[0], pilot_symbols, SLOT_LEN);
  memcpy(&pilot_ul[SLOT_LEN + SLOT_GUARD_INTERVAL], pilot_symbols, SLOT_LEN);
  memcpy(&pilot_ul[2 * (SLOT_LEN + SLOT_GUARD_INTERVAL) + NUM_ULCTRL_SLOT * 2],
         pilot_symbols, SLOT_LEN);
  memcpy(&pilot_ul[3 * (SLOT_LEN + SLOT_GUARD_INTERVAL) + NUM_ULCTRL_SLOT * 2],
         pilot_symbols, SLOT_LEN);

  // force first symbol of the slot to contain the pilot sequence reset flag
  pilot_ul[0] = PILOT_RESET;
  pilot_ul[SLOT_LEN + SLOT_GUARD_INTERVAL] = PILOT_RESET;
  pilot_ul[2 * (SLOT_LEN + SLOT_GUARD_INTERVAL) + NUM_ULCTRL_SLOT * 2] =
      PILOT_RESET;
  pilot_ul[3 * (SLOT_LEN + SLOT_GUARD_INTERVAL) + NUM_ULCTRL_SLOT * 2] =
      PILOT_RESET;

  // ulctrl slots
  pilot_ul[2 * (SLOT_LEN + SLOT_GUARD_INTERVAL)] = PILOT_RESET;
  pilot_ul[2 * (SLOT_LEN + SLOT_GUARD_INTERVAL) + 2] = PILOT_RESET;
}