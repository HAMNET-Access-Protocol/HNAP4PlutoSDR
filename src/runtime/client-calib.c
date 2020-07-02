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

#define _GNU_SOURCE

#include "../mac/mac_ue.h"
#include "../phy/phy_config.h"
#include "../phy/phy_ue.h"
#include "../platform/platform_simulation.h"
#include "../platform/pluto.h"
#include "../util/log.h"
#include <version.h>

#include <getopt.h>
#include <math.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#define SYMBOLS_PER_BUF 2
int buflen = 0;

// Set to 1 in order to use the simulated platform
#define CLIENT_USE_PLATFORM_SIM 0

#define TCXO_HZ 40000000

platform pluto;

// program options
struct option Options[] = {
    {"rxgain", required_argument, NULL, 'g'},
    {"log-file", required_argument, NULL, 'f'},
    {"config", required_argument, NULL, 'c'},
    {"log", required_argument, NULL, 'l'},
    {"help", no_argument, NULL, 'h'},
    {"version", no_argument, NULL, 'v'},
    {NULL},
};
char *helpstring = "Estimate clock drift of the client.\n\n \
Options:\n \
  --rxgain -g:  fix the rxgain to a value [-1 73]\n \
  --config -c:  use a given configuration file\n\
  --log-file -f <filename> log all cfo estimates to a file\n \
  --log -l      specify the log level. Default: 2.\n \
                0=TRACE 1=DEBUG 2=INFO 3=WARN 4=ERR 5=NONE\n";
extern char *optarg;
int rxgain = -100;
int enable_agc = 0;
char filename[80];
FILE *fd = NULL;
int do_log = 0;
char *config_file = NULL;

void signalhandler() {
  if (do_log)
    fclose(fd);

  pluto->end(pluto);
  printf("----------------------\n\n");
  printf("To finish the calibration, run 'fw_printenv xo_correction' to get\n");
  printf("the currently configured xo_correction factor. If nothing is set,\n");
  printf("40000000 is the default value.\n\n");
  printf("The value should be set to xo_correction-corr_factor:\n");
  printf(
      "'fw_setenv xo_correction <new value>' will write the corrected rate\n");
  printf("After the value is set, do 'reboot' to load the new xo correction "
         "factor\n");
  exit(0);
}

// Get first estimates of the carrier frequency offset and tune to the
// correct frequency
void phy_carrier_sync(PhyUE phy, platform hw) {
  float complex *rxbuf_time = calloc(sizeof(float complex), buflen);
  int gain_diff = 0;

  // read some buffers, to ensure we got samples with adjusted rxgain
  for (int i = 0; i < KERNEL_BUF_RX; i++)
    hw->platform_rx(hw, rxbuf_time);

  // Find synchronization sequence for the first time
  while (!phy->has_synced_once) {
    hw->platform_rx(hw, rxbuf_time);
    phy_ue_do_rx(phy, rxbuf_time, buflen);
  }

  // receive some subframes to get a better cfo estimation
  float cfo_hz = 0;
  const int iterations = 8 * 16;
  for (int i = 0; i < iterations; i++) {
    for (int sym = 0; sym < SUBFRAME_LEN / SYMBOLS_PER_BUF; sym++) {
      hw->platform_rx(hw, rxbuf_time);
      phy_ue_do_rx(phy, rxbuf_time, buflen);
      cfo_hz += ofdmframesync_get_cfo(phy->fs) * samplerate / (2 * M_PI);
    }
    gain_diff += agc_desired_rssi - (int)ofdmframesync_get_rssi(phy->fs);
  }
  cfo_hz /= (float)iterations * SUBFRAME_LEN / SYMBOLS_PER_BUF;
  if (enable_agc)
    rxgain += gain_diff / iterations;
  pluto_set_rxgain(hw, rxgain);

  // tune to the correct frequency
  pluto_set_tx_freq(hw, ul_lo + (long)cfo_hz);
  pluto_set_rx_freq(hw, dl_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] retune transceiver with cfo %.3fHz:\n", cfo_hz);
  LOG(INFO, "[CLIENT] TX LO freq: %lldHz\n", ul_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] RX LO freq: %lldHz\n", dl_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] rxgain adjusted: %d\n", rxgain);
  free(rxbuf_time);
}

int main(int argc, char *argv[]) {
  // set thread priority
  struct sched_param prio;
  prio.sched_priority = 3;
  sched_setscheduler(0, SCHED_FIFO, &prio);

  phy_config_default_64();
  buflen = (nfft + cp_len) * SYMBOLS_PER_BUF;

  // parse program args
  int d;
  while ((d = getopt_long(argc, argv, "g:f:c:l:v:h", Options, NULL)) != EOF) {
    switch (d) {
    case 'g':
      rxgain = atoi(optarg);
      if (rxgain < RXGAIN_MIN || rxgain > RXGAIN_MAX) {
        printf("Error: rxgain %d out of range [-1 73]!\n", rxgain);
        exit(0);
      }
      break;
    case 'f':
      do_log = 1;
      if (optarg != NULL) {
        strncpy(filename, optarg, 80);
      }
      break;
    case 'c':
      config_file = calloc(strlen(optarg), 1);
      strcpy(config_file, optarg);
      phy_config_load_file(optarg);
      buflen = (nfft + cp_len) * SYMBOLS_PER_BUF;
      break;
    case 'l':
      global_log_level = atoi(optarg);
      if (global_log_level < TRACE || global_log_level > NONE) {
        printf("ERROR: log level %d undefined!\n", global_log_level);
        exit(EXIT_FAILURE);
      }
      break;
    case 'h':
      printf("%s", helpstring);
      exit(0);
      break;
    case 'v':
      printf("Build version %s %s\n", GIT_TAG, GIT_BRANCH);
      exit(0);
      break;
    default:
      printf("%s", helpstring);
      exit(0);
    }
  }
  phy_config_print();

  // register signal handler
  signal(SIGINT, signalhandler);

  // Init platform
#if CLIENT_USE_PLATFORM_SIM
  platform pluto = platform_init_simulation(NFFT + CP_LEN);
#else
  pluto = init_pluto_platform(buflen, config_file);
  pluto_set_tx_freq(pluto, ul_lo);
  pluto_set_rx_freq(pluto, dl_lo);
  usleep(100000);

  if (rxgain == -100) {
    enable_agc = 1;
    rxgain = pluto_get_rxgain(pluto) + agc_desired_rssi;
  }
  pluto_set_rxgain(pluto, rxgain);
  LOG(INFO, "[CLIENT] initial rxgain %d\n", rxgain);
  usleep(100000);

#endif

  // Init phy and mac layer
  PhyUE phy = phy_ue_init();
  MacUE mac = mac_ue_init();
  phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
  mac_ue_set_phy_interface(mac, phy);

  // Tune to the correct frequency
  phy_carrier_sync(phy, pluto);
  sleep(1); // wait till the transceiver adjusted LO freqs
  phy_ue_destroy(phy);

  // Re Init phy and mac layer for clean startup
  phy = phy_ue_init();

  phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
  mac_ue_set_phy_interface(mac, phy);

  if (do_log) {
    fd = fopen(filename, "w");
    if (!fd) {
      LOG(ERR, "could not open logfile '%s'\n", filename);
      return 0;
    }
  }

  float complex *rxbuf_time = calloc(sizeof(float complex), buflen);

  // read some rxbuffer objects in order to empty rxbuffer queue
  for (int i = 0; i < KERNEL_BUF_RX; i++)
    pluto->platform_rx(pluto, rxbuf_time);

  // Receive
  uint iteration = 0;
  float cfo = 0, cfo_avg = 0, cfo_min = 1000000, cfo_max = -100000;
  int gain_real_avg = 0, gain_real_max = 0;
  int gain_imag_avg = 0, gain_imag_max = 0;
  float last_rssi = agc_desired_rssi;

  while (1) {
    pluto->platform_rx(pluto, rxbuf_time);
    phy_ue_do_rx(phy, rxbuf_time, buflen);
    cfo = ofdmframesync_get_cfo(phy->fs) * 256000 / 6.28;
    if (cfo != 0.0) {
      cfo_avg += cfo;
      if (cfo_max < cfo)
        cfo_max = cfo;
      if (cfo_min > cfo)
        cfo_min = cfo;
      if (do_log)
        fprintf(fd, "%.3f\n", cfo);

      for (int i = 0; i < buflen; i++) {
        int real = abs(creal(rxbuf_time[i]) * 2048.0);
        int imag = abs(cimag(rxbuf_time[i]) * 2048.0);
        gain_real_avg += real;
        gain_imag_avg += imag;
        if (real > gain_real_max)
          gain_real_max = real;
        if (imag > gain_imag_max)
          gain_imag_max = imag;
      }

      // basic AGC: phy->rssi is updated at the start of each sync slot (before
      // the next sync signal)
      if (fabsf(last_rssi - phy->rssi) > 3 && enable_agc) {
        int gain_diff = agc_desired_rssi - roundf(phy->rssi);
        rxgain += gain_diff;
        if (rxgain > RXGAIN_MAX)
          rxgain = RXGAIN_MAX;
        if (rxgain < RXGAIN_MIN)
          rxgain = RXGAIN_MIN;
        pluto_set_rxgain(pluto, rxgain);
        last_rssi = phy->rssi;
        LOG(INFO, "[Client] new rxgain: %d diff: %d rssi: %.3f\n", rxgain,
            gain_diff, phy->rssi);
      }

      if (iteration % 1000 == 0) {
        printf("cfo avg: %.3f, max: %.3f min: %.3f\n", cfo_avg / 1000, cfo_max,
               cfo_min);
        printf("adapt xo by corr_factor=%d\n",
               (int)round(cfo_avg / 1000 * (float)TCXO_HZ / dl_lo));
        printf("Real gain avg: %d, max: %d\n", gain_real_avg / 1000 / buflen,
               gain_real_max);
        printf("Imag gain avg: %d, max: %d\n", gain_imag_avg / 1000 / buflen,
               gain_imag_max);
        printf("RSSI: %.2f\n", ofdmframesync_get_rssi(phy->fs));
        cfo_avg = 0;
        cfo_min = 1000000;
        cfo_max = -100000;
        gain_real_avg = 0;
        gain_real_max = 0;
        gain_imag_avg = 0;
        gain_imag_max = 0;
      }
      iteration++;
    }
  }
}
