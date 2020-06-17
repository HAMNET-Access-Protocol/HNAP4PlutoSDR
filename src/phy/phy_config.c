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

#include "phy_config.h"
#include "../util/log.h"
#include <libconfig.h>
#include <liquid/liquid.h>

void phy_config_load_file(char* config_file)
{
    config_t cfg;
    config_setting_t* phy_settings=NULL, *subcarrier_settings=NULL, *symbol_settings=NULL;
    config_init(&cfg);

    /* Read the file. If there is an error, report it and exit. */
    if(! config_read_file(&cfg, config_file))
    {
        LOG(ERR, "[PHY CONFIG] ERR: %s:%d - %s\n", config_error_file(&cfg),
                config_error_line(&cfg), config_error_text(&cfg));
        config_destroy(&cfg);
        exit(EXIT_FAILURE);
    }
    phy_settings = config_lookup(&cfg,"phy");
    if (phy_settings!=NULL) {
        config_setting_lookup_int64(phy_settings, "dl_lo", &dl_lo);
        config_setting_lookup_int64(phy_settings,"ul_lo",&ul_lo);
        config_setting_lookup_int(phy_settings, "nfft",&nfft);
        config_setting_lookup_int(phy_settings, "cp_len",&cp_len);
        config_setting_lookup_int(phy_settings, "samplerate",&samplerate);
        config_setting_lookup_float(phy_settings,"coarse_cfo_filt_param",&coarse_cfo_filt_param);
        config_setting_lookup_float(phy_settings,"agc_rssi_filt_param",&agc_rssi_filt_param);
        config_setting_lookup_int(phy_settings,"agc_change_threshold",&agc_change_threshold);
        config_setting_lookup_int(phy_settings,"agc_desired_rssi",&agc_desired_rssi);

        subcarrier_settings = config_setting_get_member(phy_settings, "subcarrier_alloc");
        if (subcarrier_settings!=NULL && config_setting_length(subcarrier_settings)>0) {
            free(subcarrier_alloc);
            subcarrier_alloc = malloc(nfft);
            num_pilot_sc = 0;
            num_data_sc = 0;
            for (int i = 0; i < nfft; i++) {
                subcarrier_alloc[i] = (char) config_setting_get_int_elem(subcarrier_settings, i);
                switch (subcarrier_alloc[i]) {
                    case OFDMFRAME_SCTYPE_DATA:
                        num_data_sc++;
                        break;
                    case OFDMFRAME_SCTYPE_PILOT:
                        num_pilot_sc++;
                        break;
                    case OFDMFRAME_SCTYPE_NULL:
                        break;
                    default:
                        LOG(ERR, "[PHY CONFIG] error when parsing SC allocation. %d is an unknown alloc type\n",
                            subcarrier_alloc[i]);
                        exit(EXIT_FAILURE);
                        break;
                }
            }
        }
        symbol_settings = config_setting_get_member(phy_settings, "pilot_symbols");
        if (symbol_settings!=NULL && config_setting_length(symbol_settings)>0) {
            free(pilot_symbols);
            pilot_symbols =malloc(nfft);
            pilot_symbols_per_slot = 0;
            for (int i=0; i<SLOT_LEN; i++) {
                pilot_symbols[i] = (char)config_setting_get_int_elem(symbol_settings,i);
                if (pilot_symbols[i]==DATA) {
                    num_data_sc++;
                } else if (pilot_symbols[i]!=NOT_USED) {
                    LOG(ERR, "[PHY CONFIG] error when parsing symbol allocation. %d is an unknown allocation\n",
                        pilot_symbols[i]);
                }
            }
        }
    }
}

// Default config for 64 subcarriers.
void phy_config_default_64()
{
    dl_lo = DEFAULT_LO_FREQ_DL;
    ul_lo = DEFAULT_LO_FREQ_UL;
    nfft = DEFAULT_NFFT;
    cp_len = DEFAULT_CP_LEN;
    samplerate = DEFAULT_SAMPLERATE;
    subcarrier_alloc = malloc(nfft);
    // generate frequency domain allocation of pilot symbols
    // initialize as NULL
    for (int i=0; i<nfft; i++)
        subcarrier_alloc[i] = OFDMFRAME_SCTYPE_NULL;

    // upper band: 20 data subcarriers
    for (int i=0; i<20; i++) {
        subcarrier_alloc[i] = OFDMFRAME_SCTYPE_DATA;
    }

    // lower band: 20 data subcarriers
    for (int i=0; i<20; i++) {
        subcarrier_alloc[nfft-1-i] = OFDMFRAME_SCTYPE_DATA;
    }

    // set pilots. Every 5th subcarrier as pilot
    for (int i=2; i<20; i+=5) {
        subcarrier_alloc[i] = OFDMFRAME_SCTYPE_PILOT;
        subcarrier_alloc[nfft-1-i] = OFDMFRAME_SCTYPE_PILOT;
    }
    num_data_sc = 32;
    num_pilot_sc = 8;
    pilot_symbols_per_slot = 7;
    pilot_symbols = calloc(SLOT_LEN,1);
    for (int i=0; i<SLOT_LEN; i+=2)
        pilot_symbols[i]=DATA;

    coarse_cfo_filt_param = DEFAULT_COARSE_CFO_FILT_PARAM;
    agc_rssi_filt_param = DEFAULT_AGC_RSSI_FILT_PARAM;
    agc_change_threshold = DEFAULT_AGC_CHANGE_THRESHOLD;
    agc_desired_rssi = DEFAULT_AGC_DESIRED_RSSI;
}

void phy_config_print()
{
    printf("[PHY CONFIG] current PHY layer configuration:\n");
    printf("DL LO freq: %lld\n",dl_lo);
    printf("UL LO freq: %lld\n",ul_lo);
    printf("samplerate: %d\n", samplerate);
    printf("FFT size:   %d\n", nfft);
    printf("CP length:  %d\n", cp_len);
    printf("SC alloc:   ");
    for (int x = 0; x < nfft; x += 16) {
        for (int y = 0; y < 16; y++)
            printf("%d ", subcarrier_alloc[x + y]);
        printf("\n            ");
    }
    printf("\nnum_data_sc:   %d\n",num_data_sc);
    printf("num_pilot_sc:  %d\n",num_pilot_sc);
    printf("slot len:      %d\n", SLOT_LEN);
    printf("slot alloc: ");
    for (int x = 0; x < SLOT_LEN; x++) {
        printf("%d ", pilot_symbols[x]);
    }
    printf("\n");

    printf("coarse cfo filter param: %.3f\n",coarse_cfo_filt_param);
}
