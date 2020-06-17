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

#ifndef PHY_CONFIG_H_
#define PHY_CONFIG_H_


#ifdef SIM_LOG_BER
#define PHY_TEST_BER
#else
// If this is set, a seperate thread for Viterbi Decoding and MAC RX is used. Should be activated for better performance
// If not used, the phy RX thread has to do Decoding and MAC RX itself.
#define USE_RX_SLOT_THREAD
#endif

// Default LO frequency
#define DEFAULT_LO_FREQ_UL 434900000 // Hz
#define DEFAULT_LO_FREQ_DL 439700000 // Hz

// Default OFDM Parameters
#define DEFAULT_NFFT 64				// FFT size
#define DEFAULT_CP_LEN 4			// Cyclic Prefix Size
#define DEFAULT_SAMPLERATE 256000	// sample rate in Hz

// Subframe/slot length config
#define SLOT_LEN 14			// length of one data slot
#define NUM_SLOT 4			// number of slots per subframe
#define SLOT_GUARD_INTERVAL 1 // number of ofdm symbols between two slots
#define NUM_ULCTRL_SLOT 2	// number of UL control slots
#define SUBFRAME_LEN 64		// number of OFDM symbols per subframe
#define DLCTRL_LEN 2		// number of OFDM symbols for DL control info
#define SYNC_SYMBOLS 4		// number of OFDM symbols for synch signaling
#define FRAME_LEN 8			// number of subframes per frame
#define DL_UL_SHIFT 34		// number of ofdm symbols the UL is shifted behind
#define MAX_USER 16


#define DEFAULT_COARSE_CFO_FILT_PARAM 0.8f

// AGC default configuration
#define DEFAULT_AGC_RSSI_FILT_PARAM 0.25f
#define DEFAULT_AGC_CHANGE_THRESHOLD 3
#define DEFAULT_AGC_DESIRED_RSSI -15

// FIR filters, buffers etc introduce a delay that causes
// uplink data to be received later than expected. Use this
// variable to compensate for this offset at client side.
// value range: [0 15] ofdm symbols. Otherwise waveform wont work
#ifndef USE_SIM
#define DL_UL_SHIFT_COMP_BS 9
#define DL_UL_SHIFT_COMP_UE 8
#else
// The simulation target needs these values to be 0
#define DL_UL_SHIFT_COMP_BS 0
#define DL_UL_SHIFT_COMP_UE 0
#endif


enum {NOT_USED, DATA, PTT_UP, PTT_DOWN}; // definition for tx_symbol allocation variable

// ---------------------------- global PHY layer configurtion  --------------------------------- //

long long int dl_lo;        // Downlink carrier frequency
long long int ul_lo;        // Uplink carrier frequency

int nfft;                   // size of the fft
int cp_len;                 // number of cyclic prefix samples
int samplerate;             // sample-rate in samples/sec
char* subcarrier_alloc;     // subcarrier allocation in frequency domain
int num_data_sc;            // total number of data subcarriers
int num_pilot_sc;           // total number of pilot subcarriers
int pilot_symbols_per_slot; // number of symbols including pilots per slot
char* pilot_symbols;        // ofdm symbol types within a data-slot

// UE regularly re-syncs to the sync sequence to estimate timing offset
// during this also the carrier frequency offset is estimated.
// estimation can estimate larger offsets than the cfo estimation with
// pilots (done during slot receive) but is inaccurate
// We filter the new estimate with the old (pilot based) cfo estimation
// Set this filter to [0 1] to tune estimation (1= solely based on new cfo)
double coarse_cfo_filt_param;

// UE constantly adapts the rxgain based on the rssi. The rssi is calculated based on the
// sync signal, every 8 subframes. The new rssi value is smoothed with an exponential filter
// the filter coefficient alpha can be tuned here
double agc_rssi_filt_param;
// Threshold for the RSSI change [dB] which is required to change
// the transceiver gain
int agc_change_threshold;
// the desired RSSI of the RX path. Used to tune our AGC
// Theoretical limits for RSSI are [-66 0]. For OFDM-QAM waveform this should be set to ~ -15
int agc_desired_rssi;

int log_coarse_cfo_flag;    // set this flag to enable logging the coarse cfo estimate to a file
char coarse_cfo_logfile[80];// name of the coarse cfo logfile

int log_cfo_flag;           // set this flag to enable logging the fine cfo estimates to a file
char cfo_logfile[80];       // name of the  fine cfo logfile

// ---------------------------- PHY config functions --------------------------------- //
// get the configuration from the specified file path
void phy_config_load_file(char* config_file);
// set the configuration to the default for 64 subcarriers, 200KHz bandwidth.
void phy_config_default_64();

// print the current config to console
void phy_config_print();

#endif /* PHY_CONFIG_H_ */
