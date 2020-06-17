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

#include "../mac/mac_ue.h"
#include "../mac/mac_bs.h"
#include "../phy/phy_ue.h"
#include "../phy/phy_bs.h"
#include "../platform/platform_simulation.h"


extern struct phy_config_s PHY_CONFIG;

uint global_sfn = 0;
uint global_symbol = 0;

uint num_simulated_subframes = 8000;

// UE/BS instances
PhyUE phy_ue;
PhyBS phy_bs;
MacUE mac_ue;
MacBS mac_bs;

uint buflen=0;

platform bs;
platform client;


int run_simulation(uint num_subframes)
{
    global_sfn = 0;
    global_symbol = 0;
    // offset stores the currently compensated TX advance
    // tx_shift stores the shift within the buffer that is caused by the offset
    int offset=0, tx_shift=0, num_samples=buflen;

    // Buffers for simulation
    float complex dl_data[nfft+cp_len];

    while (global_sfn<num_subframes)
    {
        // BS TX
        phy_bs_write_symbol(phy_bs, dl_data);
        bs->platform_tx_prep(bs, dl_data, 0, buflen);
        bs->platform_tx_push(bs);

        // ---------- RX ------------
        client->platform_rx(client, dl_data);
        // process samples
        phy_ue_do_rx(phy_ue, dl_data, nfft+cp_len);
        // show progress
        if (global_sfn%1000==0 && global_symbol==0)
            printf("Processed %d subframes...\n",global_sfn);

        global_symbol++;
        if (global_symbol>=SUBFRAME_LEN) {
            global_symbol=0;
            global_sfn++;
        }

    }
    return 0;
}

void setup_simulation(float snr, float cfo)
{
    // Setup the hardware
    bs = platform_init_simulation(buflen, snr, cfo);
    client = platform_init_simulation(buflen, snr, cfo);
    simulation_connect(bs, client);

    // Create PHY and MAC instances
    phy_ue = phy_ue_init();
    phy_bs = phy_bs_init();
    mac_ue = mac_ue_init();
    mac_bs = mac_bs_init();

    phy_ue_set_mac_interface(phy_ue, mac_ue_rx_channel, mac_ue);
    mac_ue_set_phy_interface(mac_ue, phy_ue);
    phy_ue_set_platform_interface(phy_ue, client);
    phy_bs_set_mac_interface(phy_bs, mac_bs);
    mac_bs_set_phy_interface(mac_bs, phy_bs);
}

void clean_simulation()
{
    phy_bs_destroy(phy_bs);
    phy_ue_destroy(phy_ue);
    mac_bs_destroy(mac_bs);
    mac_ue_destroy(mac_ue);
    bs->end(bs);
    client->end(client);
}

int main(int argc, char* argv[])
{
    phy_config_default_64();
    buflen = nfft+cp_len;
    global_log_level = ERR;

    for (int cfo=0; cfo<1; cfo+=50) {
        for (int snr = 5; snr < 41; snr += 5) {
            printf("Starting simulation with SNR %ddB; cfo %dHz\n", snr, cfo);

            log_coarse_cfo_flag = 0;
            sprintf(coarse_cfo_logfile, "coarse_cfo_est_snr%02d_cfo%d.bin", snr, cfo);
            log_cfo_flag = 1;
            sprintf(cfo_logfile, "cfo_est_snr%02d.bin", snr);

            setup_simulation(snr, cfo);
            run_simulation(num_simulated_subframes);
            clean_simulation();
        }
    }
}
