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

#include "phy_sc_common.h"
#include "phy_config.h"

float complex zadoff_chu_seq[ZADOFF_CHU_LEN] = {0,
                                                1.000000 + 0.000000i,
                                                0.998606 + -0.052775i,
                                                0.987481 + -0.157738i,
                                                0.950237 + -0.311527i,
                                                0.863817 + -0.503806i,
                                                0.702425 + -0.711758i,
                                                0.445738 + -0.895163i,
                                                0.092268 + -0.995734i,
                                                -0.324042 + -0.946043i,
                                                -0.720968 + -0.692968i,
                                                -0.971906 + -0.235370i,
                                                -0.941683 + 0.336501i,
                                                -0.559679 + 0.828709i,
                                                0.092268 + 0.995734i,
                                                0.739009 + 0.673696i,
                                                0.998606 + -0.052775i,
                                                0.623490 + -0.781831i,
                                                -0.222521 + -0.974928i,
                                                -0.922612 + -0.385730i,
                                                -0.821250 + 0.570569i,
                                                0.092268 + 0.995734i,
                                                0.932472 + 0.361242i,
                                                0.702425 + -0.711758i,
                                                -0.421954 + -0.906617i,
                                                -0.991301 + 0.131617i,
                                                -0.118520 + 0.992952i,
                                                0.950237 + 0.311527i,
                                                0.445738 + -0.895163i,
                                                -0.850217 + -0.526432i,
                                                -0.559679 + 0.828709i,
                                                0.836025 + 0.548692i,
                                                0.492360 + -0.870392i,
                                                -0.922612 + -0.385730i,
                                                -0.222521 + 0.974928i,
                                                1.000000 + -0.000000i,
                                                -0.273663 + -0.961826i,
                                                -0.821250 + 0.570569i,
                                                0.836025 + 0.548692i,
                                                0.144690 + -0.989477i,
                                                -0.941683 + 0.336501i,
                                                0.773534 + 0.633755i,
                                                0.092268 + -0.995734i,
                                                -0.850217 + 0.526432i,
                                                0.950237 + 0.311527i,
                                                -0.421954 + -0.906617i,
                                                -0.324042 + 0.946043i,
                                                0.863817 + -0.503806i,
                                                -0.991301 + -0.131617i,
                                                0.739009 + 0.673696i,
                                                -0.273663 + -0.961826i,
                                                -0.222521 + 0.974928i,
                                                0.623490 + -0.781831i,
                                                -0.876815 + 0.480828i,
                                                0.987481 + -0.157738i,
                                                -0.991301 + -0.131617i,
                                                0.932472 + 0.361242i,
                                                -0.850217 + -0.526432i,
                                                0.773534 + 0.633755i,
                                                -0.720968 + -0.692968i,
                                                0.702425 + 0.711758i,
                                                -0.720968 + -0.692968i,
                                                0.773534 + 0.633755i,
                                                -0.850217 + -0.526432i,
                                                0.932472 + 0.361242i,
                                                -0.991301 + -0.131617i,
                                                0.987481 + -0.157738i,
                                                -0.876815 + 0.480828i,
                                                0.623490 + -0.781831i,
                                                -0.222521 + 0.974928i,
                                                -0.273663 + -0.961826i,
                                                0.739009 + 0.673696i,
                                                -0.991301 + -0.131617i,
                                                0.863817 + -0.503806i,
                                                -0.324042 + 0.946043i,
                                                -0.421954 + -0.906617i,
                                                0.950237 + 0.311527i,
                                                -0.850217 + 0.526432i,
                                                0.092268 + -0.995734i,
                                                0.773534 + 0.633755i,
                                                -0.941683 + 0.336501i,
                                                0.144690 + -0.989477i,
                                                0.836025 + 0.548692i,
                                                -0.821250 + 0.570569i,
                                                -0.273663 + -0.961826i,
                                                1.000000 + -0.000000i,
                                                -0.222521 + 0.974928i,
                                                -0.922612 + -0.385730i,
                                                0.492360 + -0.870392i,
                                                0.836025 + 0.548692i,
                                                -0.559679 + 0.828709i,
                                                -0.850217 + -0.526432i,
                                                0.445738 + -0.895163i,
                                                0.950237 + 0.311527i,
                                                -0.118520 + 0.992952i,
                                                -0.991301 + 0.131617i,
                                                -0.421954 + -0.906617i,
                                                0.702425 + -0.711758i,
                                                0.932472 + 0.361242i,
                                                0.092268 + 0.995734i,
                                                -0.821250 + 0.570569i,
                                                -0.922612 + -0.385730i,
                                                -0.222521 + -0.974928i,
                                                0.623490 + -0.781831i,
                                                0.998606 + -0.052775i,
                                                0.739009 + 0.673696i,
                                                0.092268 + 0.995734i,
                                                -0.559679 + 0.828709i,
                                                -0.941683 + 0.336501i,
                                                -0.971906 + -0.235370i,
                                                -0.720968 + -0.692968i,
                                                -0.324042 + -0.946043i,
                                                0.092268 + -0.995734i,
                                                0.445738 + -0.895163i,
                                                0.702425 + -0.711758i,
                                                0.863817 + -0.503806i,
                                                0.950237 + -0.311527i,
                                                0.987481 + -0.157738i,
                                                0.998606 + -0.052775i,
                                                1.000000 + 0.000000i};

// Training sequence is generated with a PN seq. Poly z^6+z+1
// Real parts are the first 40 bits of PN seq, imag part is bit 40-80
float complex training_seq[SC_SYMBOL_UNIT] = {
    1.0000 + 1.0000i,  -1.0000 + 1.0000i, -1.0000 - 1.0000i, -1.0000 + 1.0000i,
    -1.0000 + 1.0000i, -1.0000 + 1.0000i, 1.0000 - 1.0000i,  -1.0000 + 1.0000i,
    -1.0000 + 1.0000i, -1.0000 - 1.0000i, -1.0000 - 1.0000i, 1.0000 + 1.0000i,
    1.0000 + 1.0000i,  -1.0000 - 1.0000i, -1.0000 + 1.0000i, -1.0000 - 1.0000i,
    1.0000 + 1.0000i,  -1.0000 - 1.0000i, 1.0000 + 1.0000i,  -1.0000 + 1.0000i,
    -1.0000 + 1.0000i, 1.0000 + 1.0000i,  1.0000 + 1.0000i,  1.0000 + 1.0000i,
    1.0000 - 1.0000i,  -1.0000 - 1.0000i, 1.0000 - 1.0000i,  -1.0000 - 1.0000i,
    -1.0000 - 1.0000i, -1.0000 + 1.0000i, 1.0000 - 1.0000i,  1.0000 - 1.0000i,
    1.0000 - 1.0000i,  -1.0000 - 1.0000i, -1.0000 + 1.0000i, 1.0000 + 1.0000i,
    -1.0000 - 1.0000i, -1.0000 - 1.0000i, 1.0000 - 1.0000i,  -1.0000 + 1.0000i};

// Init the PHY instance
PhyCommon phy_common_init() {
  PhyCommon phy = calloc(sizeof(PhyCommon_s), 1);

  // TX buffer has 2 entries for even/uneven subframes
  phy->txdata = malloc(sizeof(float complex *) * 2);

  // alloc buffer for one subframe of symbols in frequency domain
  phy->txdata[0] = calloc(sizeof(float complex), SUBFRAME_LEN * SC_SYMBOL_UNIT);
  phy->txdata[1] = calloc(sizeof(float complex), SUBFRAME_LEN * SC_SYMBOL_UNIT);
  phy->rxdata = calloc(sizeof(float complex), SUBFRAME_LEN * SC_SYMBOL_UNIT);

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

  // RRC pulse shaoing filter
  phy->rrc_shaper = firinterp_cccf_create_prototype(
      LIQUID_FIRFILT_RRC, OSF, FILTER_DELAY, FILTER_BETA, 0);

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
  free(phy->txdata[0]);
  free(phy->txdata[1]);

  free(phy->rxdata);
  free(phy->txdata);

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
  uint symbols = (SLOT_LEN - 1) * SC_SYMBOL_UNIT;
  uint enc_bits =
      symbols * modem_get_bps(phy->mcs_modem[mcs]); // number of encoded bits
  return (enc_bits - 16) *
         fec_get_rate(phy->mcs_fec_scheme[mcs]); // real tbs size. Subtract
                                                 // 16bit for conv encoding
}

// returns the size of the ULCTRL slots in bits
int get_ulctrl_slot_size(PhyCommon phy) {
  uint symbols = SC_SYMBOL_UNIT;
  uint enc_bits =
      symbols * modem_get_bps(phy->mcs_modem[0]); // number of encoded bits
  return (enc_bits - 16) *
         fec_get_rate(phy->mcs_fec_scheme[0]); // real tbs size. Subtract 16bit
                                               // for conv encoding
}

/* Modulate the given data
 * returns the number of symbols that have been generated
 * Params:	common: 	pointer to the common phy struct
 *			subframe:	subframe number. Currently only even and
 *uneven (0/1) is defined first_symb:	index of the first symbol within the
 *subframe that shall be mapped to last_symb:	index of the last symbol mcs:
 *the MCS index that shall be used
 *			data:		array of symbols that will be modulated
 *			buf_len:	length of the data array
 *
 * Returns:	written_samps:	the number of symbols that have been generated
 */
void phy_mod(PhyCommon common, uint subframe, uint first_symb, uint last_symb,
             uint mcs, uint8_t *data, uint buf_len, uint *written_samps) {
  *written_samps = 0;
  for (int sym_idx = first_symb; sym_idx <= last_symb; sym_idx++) {
    modem_modulate(common->mcs_modem[mcs], (uint)data[(*written_samps)++],
                   &common->txdata[subframe][sym_idx]);
    if (*written_samps >= buf_len) {
      return;
    }
  }
}

// Symbol demapper with soft decision
// returns an array with n llr values for each demapped symbol and the number of
// demapped bits
void phy_demod_soft(PhyCommon common, uint first_symb, uint last_symb, uint mcs,
                    uint8_t *llr, uint num_llr, uint *written_samps) {
  *written_samps = 0;
  uint bps = modem_get_bps(common->mcs_modem[mcs]);

  // demodulate signal
  uint symbol = 0;
  for (int sym_idx = first_symb; sym_idx <= last_symb; sym_idx++) {
    modem_demodulate_soft(common->mcs_modem[mcs], common->rxdata[sym_idx],
                          &symbol, &llr[*written_samps]);
    *written_samps += bps;
    if (*written_samps + bps >= num_llr) {
      return;
    }
  }
}

void phy_pulse_shaper(PhyCommon phy, float complex *sym_out,
                      float complex *samps_in, uint num_samps) {
  for (int i = 0; i < num_samps; i++)
    firinterp_cccf_execute(phy->rrc_shaper, samps_in[i], &sym_out[OSF * i]);
}

// Create a pulse with zeros. Used to go back to zero correctly, after data
// pulses were shaped
void phy_pulse_shaper_zeros(PhyCommon phy, float complex *sym_out,
                            int num_zeros) {
  for (int i = 0; i < num_zeros; i++)
    firinterp_cccf_execute(phy->rrc_shaper, 0, &sym_out[OSF * i]);
}

sc_receiver sc_receiver_create(int sym_len, sc_rec_callback cb,
                               void *cb_userd) {
  sc_receiver rec = calloc(sizeof(struct sc_receiver_s), 1);

  rec->sync_state = SEARCH_ZADOFF;
  rec->rx_timer = 0;
  rec->sym_buffer = calloc(sizeof(float complex) * sym_len, 1);
  rec->sym_window = windowcf_create(3 * SC_SYMBOL_UNIT);
  rec->sym_size = SC_SYMBOL_UNIT;
  // Sync-slot correlator
  rec->sync_correlator =
      detector_cccf_create(zadoff_chu_seq, ZADOFF_CHU_LEN, 0.7f, 0.02f);
  // Symbol timing sync
  rec->symsync = symsync_crcf_create_rnyquist(LIQUID_FIRFILT_RRC, OSF,
                                              FILTER_DELAY, FILTER_BETA, 32);

  float complex eq_taps[] = {0, 0, 0, 0, 0, 0, 0, 1};
  rec->eqlms = eqlms_cccf_create(eq_taps, 8);
  eqlms_cccf_set_bw(rec->eqlms, 0.5f);

  rec->cb = cb;
  rec->cb_userd = cb_userd;

  rec->g0 = 1;

  return rec;
}

void sc_receiver_destroy(sc_receiver rec) {
  free(rec->sym_buffer);
  detector_cccf_destroy(rec->sync_correlator);
  symsync_crcf_destroy(rec->symsync);
  windowcf_destroy(rec->sym_window);
  eqlms_cccf_destroy(rec->eqlms);

  free(rec);
}

void sc_receiver_set_cb(sc_receiver rec, sc_rec_callback cb, void *cb_userd) {
  rec->cb = cb;
  rec->cb_userd = cb_userd;
}

void sc_receiver_reset(sc_receiver rec) {
  symsync_crcf_reset(rec->symsync);
  eqlms_cccf_reset(rec->eqlms);
  detector_cccf_reset(rec->sync_correlator);
}

void sc_receiver_execute(sc_receiver rec, float complex *samples,
                         int num_samps) {
  uint num_written = 0;
  float tau, dphi, gamma;
  float complex x;

  for (int i = 0; i < num_samps; i += OSF) {
    switch (rec->sync_state) {
    case SEARCH_ZADOFF:
      // Execute RRC matched filter
      symsync_crcf_execute(rec->symsync, &samples[i], OSF, &x, &num_written);
      windowcf_push(rec->sym_window, x);
      if (detector_cccf_correlate(rec->sync_correlator, x, &tau, &dphi,
                                  &gamma)) {
        rec->sync_state = SYNC;
        rec->rx_timer = rec->sym_size;
        // adjust equalizer by using received sync sequence
        float complex *r;
        windowcf_read(rec->sym_window, &r);
        for (int t = 0; t < ZADOFF_CHU_LEN; t++) {
          eqlms_cccf_push(rec->eqlms, r[t] / gamma);
          eqlms_cccf_execute(rec->eqlms, &x);
          eqlms_cccf_step(rec->eqlms, zadoff_chu_seq[t], x);
        }
      }
      break;
    case SYNC:
      symsync_crcf_execute(rec->symsync, &samples[i], OSF, &x, &num_written);
      windowcf_push(rec->sym_window, x);
      rec->sym_buffer[rec->sym_size - rec->rx_timer] = x / rec->g0;
      rec->rx_timer--;
      if (rec->rx_timer == 0) {
        rec->cb(rec->sym_buffer, rec->sym_size, rec->cb_userd);
        rec->rx_timer = rec->sym_size;
      }
      break;
    }
  }
}

void sc_receiver_eq_update(sc_receiver rec, float complex *samples,
                           int num_samps) {
  float complex x_hat;

  for (int i = 0; i < num_samps; i++) {
    eqlms_cccf_push(rec->eqlms, samples[i]);
    eqlms_cccf_execute(rec->eqlms, &x_hat);
    eqlms_cccf_step(rec->eqlms, training_seq[i], x_hat);
  }
  // eqlms_cccf_print(rec->eqlms);
}

void sc_receiver_equalize(sc_receiver rec, float complex *samples,
                          int num_samps) {
  for (int i = 0; i < num_samps; i++) {
    eqlms_cccf_push(rec->eqlms, samples[i]);
    eqlms_cccf_execute(rec->eqlms, &samples[i]);
  }
}

int sc_receiver_get_sync(sc_receiver rec, float complex *samples,
                         int num_samps) {
  uint num_written = 0;
  float tau, dphi, gamma;
  float complex x;

  for (int i = 0; i < num_samps; i += OSF) {
    // Execute RRC matched filter
    symsync_crcf_execute(rec->symsync, &samples[i], OSF, &x, &num_written);
    windowcf_push(rec->sym_window, x);
    if (detector_cccf_correlate(rec->sync_correlator, x, &tau, &dphi, &gamma)) {
      rec->g0 = gamma;
      rec->sync_state = SYNC;
      rec->rx_timer = rec->sym_size;
      // adjust equalizer by using received sync sequence
      float complex *r;
      windowcf_read(rec->sym_window, &r);

      for (int t = 0; t < ZADOFF_CHU_LEN - 1; t++) {
        eqlms_cccf_push(rec->eqlms, r[t] / gamma);
        eqlms_cccf_execute(rec->eqlms, &x);
        eqlms_cccf_step(rec->eqlms, zadoff_chu_seq[t + 1], x);
      }
      // eqlms_cccf_print(rec->eqlms);
      return i;
    }
  }
  return -1;
}
