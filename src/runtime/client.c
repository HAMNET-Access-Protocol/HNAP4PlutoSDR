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
#include <pthread.h>
#include <sched.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// Config thread core affinities
#define UE_RX_CPUID 1
#define UE_TX_CPUID 1
#define UE_MAC_CPUID 0
#define UE_RX_SLOT_CPUID 0
#define UE_TAP_CPUID 0

// FPGA buffers contain a multiple of ofdm symbols per buffer. We fix this to 2
// symbols for low latency
#define SYMBOLS_PER_BUF 2
int buflen = -1; // size per buffer object in samples

// Set to 1 in order to use the simulated platform
#define CLIENT_USE_PLATFORM_SIM 0

#define CLIENT_SEND_ENABLE 0

// program options
struct option Options[] = {
    {"rxgain", required_argument, NULL, 'g'},
    {"txgain", required_argument, NULL, 't'},
    {"frequency", required_argument, NULL, 'f'},
    {"dl-mcs", required_argument, NULL, 'd'},
    {"ul-mcs", required_argument, NULL, 'u'},
    {"config", required_argument, NULL, 'c'},
    {"help", no_argument, NULL, 'h'},
    {"log", required_argument, NULL, 'l'},
    {"version", no_argument, NULL, 'v'},
    {NULL},
};
char *helpstring = "Client for 70cm Waveform.\n\n \
Options:\n \
   --rxgain -g:    fix the rxgain to a value [-1 73]\n \
   --txgain -t:    fix the txgain to a value [-89 0]\n \
   --frequency -f: tune to a specific (DL) frequency\n \
   --ul-mcs -u:    use given mcs in UL. Default: 0.\n \
   --dl-mcs -d:    use given mcs in DL. Default: 0.\n \
   --config -c     specify a configuration file\n \
   --log -l        specify the log level. Default: 2.\n \
                   0=TRACE 1=DEBUG 2=INFO 3=WARN 4=ERR 5=NONE\n\
   --version -v    show version info";

extern char *optarg;
int rxgain = -100;
int txgain = -100;
int enable_agc = 0;
long long int dl_frequency = -1;
long long int ul_frequency = -1;
int ul_mcs = 0;
int dl_mcs = 0;
char *config_file = NULL; // configuration file string
// struct holds arguments for RX thread
struct rx_th_data_s {
  PhyUE phy;
  platform hw;
  pthread_cond_t *scheduler_signal;
  pthread_mutex_t *scheduler_mutex;
};

// struct holds arguments for TX thread
struct tx_th_data_s {
  PhyUE phy;
  platform hw;
};

// struct holds arguments for MAC thread
struct mac_th_data_s {
  pthread_cond_t *scheduler_signal;
  pthread_mutex_t *scheduler_mutex;
  MacUE mac;
};

// struct with args for RX slot thread
struct rx_slot_th_data_s {
  PhyUE phy;
  pthread_cond_t *rx_slot_signal;
  pthread_mutex_t *rx_slot_mutex;
};

// Main Thread for UE receive
void *thread_phy_ue_rx(void *arg) {
  platform hw = ((struct rx_th_data_s *)arg)->hw;
  PhyUE phy = ((struct rx_th_data_s *)arg)->phy;
  pthread_cond_t *scheduler_signal =
      ((struct rx_th_data_s *)arg)->scheduler_signal;
  pthread_mutex_t *scheduler_mutex =
      ((struct rx_th_data_s *)arg)->scheduler_mutex;
  TIMECHECK_CREATE(timecheck_ue_rx);
  TIMECHECK_INIT(timecheck_ue_rx, "ue.rx_buffer", 10000);

  float complex *rxbuf_time = calloc(sizeof(float complex), buflen);
  float last_rssi = agc_desired_rssi;
  // read some rxbuffer objects in order to empty rxbuffer queue
  for (int i = 0; i < KERNEL_BUF_RX; i++)
    hw->platform_rx(hw, rxbuf_time);

  // Main RX loop
  while (1) {
    // fill buffer
    hw->platform_rx(hw, rxbuf_time);
    // process samples
    TIMECHECK_START(timecheck_ue_rx);
    phy_ue_do_rx(phy, rxbuf_time, buflen);
    // log_bin((uint8_t*)rxbuf_time,BUFLEN*sizeof(float complex),
    // "dl_data.bin","a");
    // Run scheduler after DLCTRL slot was received
    if (phy->common->rx_symbol == DLCTRL_LEN ||
        phy->common->rx_symbol == DLCTRL_LEN + 1) {
      pthread_mutex_lock(scheduler_mutex);
      pthread_cond_signal(scheduler_signal);
      pthread_mutex_unlock(scheduler_mutex);
    }
    // basic AGC: phy->rssi is updated at the start of each sync slot (before
    // the next sync signal)
    if (fabsf(last_rssi - phy->rssi) > agc_change_threshold && enable_agc) {
      int gain_diff = agc_desired_rssi - roundf(phy->rssi);
      rxgain += gain_diff;
      if (rxgain > RXGAIN_MAX)
        rxgain = RXGAIN_MAX;
      if (rxgain < RXGAIN_MIN)
        rxgain = RXGAIN_MIN;
      pluto_set_rxgain(hw, rxgain);
      last_rssi = phy->rssi;
      LOG(INFO, "[Client] new rxgain: %d diff: %d rssi: %.3f\n", rxgain,
          gain_diff, phy->rssi);
    }
    TIMECHECK_STOP_CHECK(timecheck_ue_rx, 1050);
    TIMECHECK_INFO(timecheck_ue_rx);
  }
  return NULL;
}

// Main Thread for UE transmit
void *thread_phy_ue_tx(void *arg) {
  PhyUE phy = ((struct tx_th_data_s *)arg)->phy;
  platform hw = ((struct tx_th_data_s *)arg)->hw;
  TIMECHECK_CREATE(timecheck_ue_tx);
  TIMECHECK_INIT(timecheck_ue_tx, "ue.tx_buffer", 10000);

  float complex *ul_data_tx = calloc(sizeof(float complex), buflen);
  int timing_advance = 0, rx_offset, num_samples, tx_shift = 0;
  ;

  // wait until rx thread has achieved sync
  while (!phy->has_synced_once) {
    hw->platform_tx_push(hw);
    hw->platform_tx_prep(hw, ul_data_tx, 0, buflen);
  }

  LOG(INFO, "[PHY UE] main tx thread started!\n");
  rx_offset = -phy->rx_offset; // TODO use rx offset to align tx
  tx_shift = phy->rx_offset;
  num_samples = buflen - tx_shift;

  while (1) {
    TIMECHECK_START(timecheck_ue_tx);
    // create tx time data
    // first add the last samples from the previous generated symbol
    hw->platform_tx_prep(hw, ul_data_tx + num_samples, 0, tx_shift);
    // create new symbol
    phy_ue_write_symbol(phy, ul_data_tx);
    phy_ue_write_symbol(phy, ul_data_tx + (nfft + cp_len));

    // prepare first part of the new symbol
    hw->platform_tx_prep(hw, ul_data_tx, tx_shift, num_samples);

    TIMECHECK_STOP_CHECK(timecheck_ue_tx, 530);
    TIMECHECK_INFO(timecheck_ue_tx);
    // push buffer
    hw->platform_tx_push(hw);

    // update timing offset for tx. TODO explain chosen tx_Symbol idx
    if (phy->common->tx_symbol >= 29 && phy->common->tx_subframe == 0) {
      // check if offset has changed
      int rx_offset_delta = (phy->rx_offset - rx_offset);
      if (rx_offset_delta > buflen / 2)
        rx_offset_delta -= buflen;
      else if (rx_offset_delta < -buflen / 2)
        rx_offset_delta += buflen;

      int ta_delta = phy->mac->timing_advance - timing_advance;
      int diff = ta_delta - rx_offset_delta;
      if (abs(diff) > 0) {
        LOG(INFO,
            "[Runtime] adapt tx offset. TA diff: %d; RX offset diff: %d old "
            "txshift: %d\n",
            ta_delta, rx_offset_delta, tx_shift);
        LOG(INFO, "[Runtime] Timingadvance: %d rx_offset: %d\n",
            phy->mac->timing_advance, phy->rx_offset);
        // if offset shift-diff is <0, we have to skip ofdm symbols
        while (tx_shift - diff < 0) {
          phy->common->tx_symbol += SYMBOLS_PER_BUF;
          diff -= buflen;
        }
        while (tx_shift - diff >= buflen) {
          phy->common->tx_symbol -= SYMBOLS_PER_BUF;
          diff += buflen;
        }
        tx_shift = tx_shift - diff;
        rx_offset = phy->rx_offset;
        timing_advance = phy->mac->timing_advance;
        num_samples = buflen - tx_shift;

        // set PTT signal delay
        pluto_ptt_set_switch_delay(hw,
                                   (int)((buflen * KERNEL_BUF_TX + tx_shift) *
                                         1000000.0 / samplerate));
      }
    }
  }
  return NULL;
}

void *thread_mac_ue_scheduler(void *arg) {
  MacUE mac = ((struct mac_th_data_s *)arg)->mac;
  pthread_cond_t *cond_signal = ((struct mac_th_data_s *)arg)->scheduler_signal;
  pthread_mutex_t *mutex = ((struct mac_th_data_s *)arg)->scheduler_mutex;
  TIMECHECK_CREATE(timecheck_ue_sched);
  TIMECHECK_INIT(timecheck_ue_sched, "ue.scheduler", 1000);
  uint sched_rounds = 0;

  if (ul_mcs > 0)
    mac_ue_req_mcs_change(mac, ul_mcs, 1);
  if (dl_mcs > 0)
    mac_ue_req_mcs_change(mac, dl_mcs, 0);

  while (1) {
    // Wait for signal from UE rx thread
    pthread_mutex_lock(mutex);
    pthread_cond_wait(cond_signal, mutex);
    TIMECHECK_START(timecheck_ue_sched);

    // add some data to send for client
#if CLIENT_SEND_ENABLE
    MacDataFrame ul_frame = dataframe_create(120);
    for (int i = 0; i < 100; i++)
      ul_frame->data[i] = rand() & 0xFF;
    if (!mac_ue_add_txdata(mac, ul_frame)) {
      dataframe_destroy(ul_frame);
    }
#endif
    mac_ue_run_scheduler(mac);
    sched_rounds++;

    // show some MAC stats
    if (sched_rounds % 1000 == 0) {
      LOG(WARN, "[MAC] channels received:fail %d:%d\n", mac->stats.chan_rx_succ,
          mac->stats.chan_rx_fail);
      LOG(WARN, "      bytes rx: %d bytes tx: %d\n", mac->stats.bytes_rx,
          mac->stats.bytes_tx);
    }

    TIMECHECK_STOP_CHECK(timecheck_ue_sched, 3500);
    TIMECHECK_INFO(timecheck_ue_sched);
    pthread_mutex_unlock(mutex);
  }
  return NULL;
}

void *thread_phy_ue_rx_slot(void *arg) {
  PhyUE phy = ((struct rx_slot_th_data_s *)arg)->phy;
  pthread_cond_t *cond_signal =
      ((struct rx_slot_th_data_s *)arg)->rx_slot_signal;
  pthread_mutex_t *mutex = ((struct rx_slot_th_data_s *)arg)->rx_slot_mutex;

  while (1) {
    // Wait for signal from UE rx thread
    pthread_mutex_lock(mutex);
    pthread_cond_wait(cond_signal, mutex);

    phy_ue_proc_slot(phy, phy->rx_slot_nr);

    pthread_mutex_unlock(mutex);
  }
  return NULL;
}

// Get first estimates of the carrier frequency offset and tune to the
// correct frequency
void phy_carrier_sync(PhyUE phy, platform hw) {
  float complex *rxbuf_time = calloc(sizeof(float complex), buflen);
  int gain_diff = 0;

  // read some buffers, to ensure we got samples with adjusted rxgain
  for (int i = 0; i < KERNEL_BUF_RX; i++)
    hw->platform_rx(hw, rxbuf_time);

  // Find synchronization sequence for the first time.
  while (!phy->has_synced_once) {
    hw->platform_rx(hw, rxbuf_time);
    phy_ue_do_rx(phy, rxbuf_time, buflen);
  }

  // receive some subframes to get a better cfo estimation
  float cfo_hz = 0;
  const int iterations = 16;
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

  // calculate the required txgain, assuming that UL and DL config is
  // symmetrical
  if (enable_agc)
    txgain = phy->bs_txgain + (rxgain - phy->bs_rxgain);
  pluto_set_txgain(hw, txgain);

  // tune to the correct frequency
  pluto_set_tx_freq(hw, ul_lo + (long)cfo_hz);
  pluto_set_rx_freq(hw, dl_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] retune transceiver with cfo %.3fHz:\n", cfo_hz);
  LOG(INFO, "[CLIENT] TX LO freq: %lldHz\n", ul_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] RX LO freq: %lldHz\n", dl_lo + (long)cfo_hz);
  LOG(INFO, "[CLIENT] rxgain adjusted: %d\n", rxgain);
  LOG(INFO, "[CLIENT] txgain adjusted: %d\n", txgain);
  free(rxbuf_time);
}

int main(int argc, char *argv[]) {
  // set main thread prio. We need the thread to be realtime for
  // phy_carrier_sync
  struct sched_param prio;
  prio.sched_priority = 3;
  sched_setscheduler(0, SCHED_FIFO, &prio);

  pthread_t ue_phy_rx_th, ue_phy_tx_th, ue_mac_th, ue_phy_rx_slot_th, ue_tap_th;

  // start by loading default config.
  phy_config_default_64();

  // parse program args
  int d;
  while ((d = getopt_long(argc, argv, "g:t:f:d:u:c:l:v:h", Options, NULL)) !=
         EOF) {
    switch (d) {
    case 'g':
      rxgain = atoi(optarg);
      if (rxgain < RXGAIN_MIN || rxgain > RXGAIN_MAX) {
        printf("Error: rxgain %d out of range [-1 73]!\n", rxgain);
        exit(EXIT_FAILURE);
      }
      break;
    case 't':
      txgain = atoi(optarg);
      if (txgain < TXGAIN_MIN || txgain > TXGAIN_MAX) {
        printf("Error: txgain %d out of range [-89 0]!\n", txgain);
        exit(EXIT_FAILURE);
      }
      break;
    case 'f':
      dl_frequency = atoi(optarg);
      ul_frequency = ul_frequency + dl_frequency - dl_lo;
      break;
    case 'd':
      dl_mcs = atoi(optarg);
      if (dl_mcs < 0 || dl_mcs >= NUM_MCS_SCHEMES) {
        printf("Error: DL MCS %d undefined!\n", dl_mcs);
        exit(EXIT_FAILURE);
      }
      break;
    case 'u':
      ul_mcs = atoi(optarg);
      if (ul_mcs < 0 || ul_mcs >= NUM_MCS_SCHEMES) {
        printf("Error: UL MCS %d undefined!\n", ul_mcs);
        exit(EXIT_FAILURE);
      }
      break;
    case 'c':
      printf("Using config file %s\n", optarg);
      config_file = calloc(strlen(optarg), 1);
      strncpy(config_file, optarg, strlen(optarg));
      phy_config_load_file(optarg);
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
  // init buffer size
  buflen = SYMBOLS_PER_BUF * (nfft + cp_len);

  // if --frequency parameter was specified, use it instead of default/file
  // config
  if (dl_frequency > 0)
    dl_lo = dl_frequency;
  if (ul_frequency > 0)
    ul_lo = ul_frequency;

    // Init platform
#if CLIENT_USE_PLATFORM_SIM
  platform pluto = platform_init_simulation(nfft + cp_len);
#else
  platform pluto = init_pluto_platform(buflen, config_file);
  pluto_enable_ptt(pluto);
  // platform pluto = init_pluto_network_platform(BUFLEN);

  pluto_set_tx_freq(pluto, ul_lo);
  pluto_set_rx_freq(pluto, dl_lo);
  usleep(100000);
  // If the gain is not fixed, we will listen for a moment and use the given
  // value from AGC. We have to use manual mode because this AGC does not work
  // with our OFDM modulation
  LOG(INFO, "[Pluto] configuring gain...\n");
  int curr_gain = pluto_get_rxgain(pluto) + agc_desired_rssi;

  // If fixed TX/RX gain values were given, set them, otherwise use the AGC
  // values
  if (rxgain == -100) {
    enable_agc = 1;
    rxgain = curr_gain;
  }

  if (txgain == -100)
    txgain = -70 + rxgain;

  pluto_set_rxgain(pluto, rxgain);
  pluto_set_txgain(pluto, txgain);
  usleep(100000);

  LOG(INFO, "[CLIENT] set initial RX gain to %d\n", rxgain);
  LOG(INFO, "[CLIENT] set initial TX gain to %d\n", txgain);
#endif

  // seed random
  srand(time(0));

  // Init phy and mac layer
  PhyUE phy = phy_ue_init();
  MacUE mac = mac_ue_init();

  phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
  mac_ue_set_phy_interface(mac, phy);
  phy_ue_set_platform_interface(phy, pluto);

  // Tune to the correct frequency and set gain
  phy_carrier_sync(phy, pluto);
  sleep(1); // wait till the transceiver adjusted LO freqs
  phy_ue_destroy(phy);

  // Re Init phy and mac layer for clean startup
  phy = phy_ue_init();

  phy_ue_set_mac_interface(phy, mac_ue_rx_channel, mac);
  mac_ue_set_phy_interface(mac, phy);
  phy_ue_set_platform_interface(phy, pluto);

#if !CLIENT_USE_PLATFORM_SIM
  pluto_start_monitor(pluto);
#endif

  // create arguments for MAC scheduler thread
  pthread_mutex_t mac_mutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t mac_cond = PTHREAD_COND_INITIALIZER;
  struct mac_th_data_s mac_th_data;
  mac_th_data.mac = mac;
  mac_th_data.scheduler_mutex = &mac_mutex;
  mac_th_data.scheduler_signal = &mac_cond;

  // create arguments for RX thread
  struct rx_th_data_s rx_th_data;
  rx_th_data.hw = pluto;
  rx_th_data.phy = phy;
  rx_th_data.scheduler_signal = &mac_cond;
  rx_th_data.scheduler_mutex = &mac_mutex;

  // create arguments for TX thread
  struct tx_th_data_s tx_th_data;
  tx_th_data.hw = pluto;
  tx_th_data.phy = phy;

  // create arguments for RX slot thread
  pthread_cond_t rx_slot_cond = PTHREAD_COND_INITIALIZER;
  pthread_mutex_t rx_slot_mutex = PTHREAD_MUTEX_INITIALIZER;
  struct rx_slot_th_data_s rx_slot_th_data;
  rx_slot_th_data.phy = phy;
  rx_slot_th_data.rx_slot_mutex = &rx_slot_mutex;
  rx_slot_th_data.rx_slot_signal = &rx_slot_cond;
  phy_ue_set_rx_slot_th_signal(phy, &rx_slot_cond);

  // start RX slot thread
  if (pthread_create(&ue_phy_rx_slot_th, NULL, thread_phy_ue_rx_slot,
                     &rx_slot_th_data) != 0) {
    LOG(ERR, "could not create RX slot processing thread. Abort!\n");
    exit(EXIT_FAILURE);
  } else {
    LOG(INFO, "created RX slot processing thread.\n");
  }
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  CPU_SET(UE_RX_SLOT_CPUID, &cpu_set);
  struct sched_param prio_rt_high, prio_rt_normal;
  prio_rt_high.sched_priority = 2;
  prio_rt_normal.sched_priority = 1;
  pthread_setaffinity_np(ue_phy_rx_slot_th, sizeof(cpu_set_t), &cpu_set);
  pthread_setschedparam(ue_phy_rx_slot_th, SCHED_FIFO, &prio_rt_normal);

  // start RX thread
  if (pthread_create(&ue_phy_rx_th, NULL, thread_phy_ue_rx, &rx_th_data) != 0) {
    LOG(ERR, "could not create RX thread. Abort!\n");
    exit(EXIT_FAILURE);
  } else {
    LOG(INFO, "created RX thread.\n");
  }
  CPU_ZERO(&cpu_set);
  CPU_SET(UE_RX_CPUID, &cpu_set);
  pthread_setaffinity_np(ue_phy_rx_th, sizeof(cpu_set_t), &cpu_set);
  pthread_setschedparam(ue_phy_rx_th, SCHED_FIFO, &prio_rt_high);

  // start TX thread
  if (pthread_create(&ue_phy_tx_th, NULL, thread_phy_ue_tx, &tx_th_data) != 0) {
    LOG(ERR, "could not create TX thread. Abort!\n");
    exit(EXIT_FAILURE);
  } else {
    LOG(INFO, "created TX thread.\n");
  }
  CPU_ZERO(&cpu_set);
  CPU_SET(UE_TX_CPUID, &cpu_set);
  pthread_setaffinity_np(ue_phy_tx_th, sizeof(cpu_set_t), &cpu_set);
  pthread_setschedparam(ue_phy_tx_th, SCHED_FIFO, &prio_rt_high);

  // start MAC thread
  if (pthread_create(&ue_mac_th, NULL, thread_mac_ue_scheduler, &mac_th_data) !=
      0) {
    LOG(ERR, "could not create MAC thread. Abort!\n");
    exit(EXIT_FAILURE);
  } else {
    LOG(INFO, "created MAC thread.\n");
  }
  CPU_ZERO(&cpu_set);
  CPU_SET(UE_MAC_CPUID, &cpu_set);
  pthread_setaffinity_np(ue_mac_th, sizeof(cpu_set_t), &cpu_set);
  pthread_setschedparam(ue_mac_th, SCHED_FIFO, &prio_rt_high);

  // start thread that binds to TAP and receives data
  if (pthread_create(&ue_tap_th, NULL, mac_ue_tap_rx_th, mac) != 0) {
    LOG(ERR, "could not create TAP rx thread. Abort!\n");
    exit(EXIT_FAILURE);
  } else {
    LOG(INFO, "created TAP thread.\n");
  }
  CPU_ZERO(&cpu_set);
  CPU_SET(UE_TAP_CPUID, &cpu_set);
  pthread_setaffinity_np(ue_tap_th, sizeof(cpu_set_t), &cpu_set);

  // printf affinities
  pthread_getaffinity_np(ue_phy_rx_th, sizeof(cpu_set_t), &cpu_set);
  printf("RX Thread CPU mask: ");
  for (int i = 0; i < 4; i++)
    printf("%d ", CPU_ISSET(i, &cpu_set));

  pthread_getaffinity_np(ue_phy_tx_th, sizeof(cpu_set_t), &cpu_set);
  printf("\nTX Thread CPU mask: ");
  for (int i = 0; i < 4; i++)
    printf("%d ", CPU_ISSET(i, &cpu_set));

  pthread_getaffinity_np(ue_mac_th, sizeof(cpu_set_t), &cpu_set);
  printf("\nMAC Thread CPU mask: ");
  for (int i = 0; i < 4; i++)
    printf("%d ", CPU_ISSET(i, &cpu_set));

  pthread_getaffinity_np(ue_phy_rx_slot_th, sizeof(cpu_set_t), &cpu_set);
  printf("\nRX slot proc Thread CPU mask: ");
  for (int i = 0; i < 4; i++)
    printf("%d ", CPU_ISSET(i, &cpu_set));
  printf("\n");

  // main thread: regulary show statistics:
  char stats_buf[512];
  while (1) {
    sleep(60);
    LOG(INFO, "MAC UE status: is associated: %d\n", mac->is_associated);
    SYSLOG(LOG_INFO, "MAC UE status: is associated: %d\n", mac->is_associated);
    if (mac->is_associated) {
      mac_stats_print(stats_buf, 512, &mac->stats);
      LOG(INFO, "%s", stats_buf);
      SYSLOG(LOG_INFO, "%s", stats_buf);
      LOG(INFO, "UL mcs %d DL mcs %d\n", mac->ul_mcs, mac->dl_mcs);
      SYSLOG(LOG_INFO, "UL mcs %d DL mcs %d\n", mac->ul_mcs, mac->dl_mcs);
    }
  }
  static void *ret[4];
  pthread_join(ue_phy_rx_th, &ret[0]);
  pthread_join(ue_phy_tx_th, &ret[1]);
  pthread_join(ue_mac_th, &ret[2]);
  pthread_join(ue_phy_rx_slot_th, &ret[3]);
}
