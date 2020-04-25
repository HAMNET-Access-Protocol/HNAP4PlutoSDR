/*
 * phy_config.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_CONFIG_H_
#define PHY_CONFIG_H_

#include "../config.h"


#ifdef SIM_LOG_BER
#define PHY_TEST_BER
#else
// If this is set, a seperate thread for Viterbi Decoding and MAC RX is used. Should be activated for better performance
// If not used, the phy RX thread has to do Decoding and MAC RX itself.
#define USE_RX_SLOT_THREAD
#endif

// LO frequency
#define LO_FREQ_UL 434900000 // Hz
#define LO_FREQ_DL 439700000 // Hz

// OFDM Parameters
#define NFFT 64				// FFT size
#define CP_LEN 4			// Cyclic Prefix Size
#define NUM_DATA_SC 32
#define NUM_PILOT 8
#define PILOT_SYM_PER_SLOT 7
#define NUM_GUARD 24
#define SAMPLERATE 256000	// sample rate in Hz

// Subframe/slot length config
#define SLOT_LEN 14			// length of one data slot
#define NUM_SLOT 4			// number of slots per subframe
#define NUM_ULCTRL_SLOT 2	// number of UL control slots
#define SUBFRAME_LEN 64		// number of OFDM symbols per subframe
#define DLCTRL_LEN 2		// number of OFDM symbols for DL control info
#define SYNC_SYMBOLS 4		// number of OFDM symbols for synch signaling
#define FRAME_LEN 8			// number of subframes per frame
#define DL_UL_SHIFT 34		// number of ofdm symbols the UL is shifted behind
#define MAX_USER 16

// Two types of pilot allocation schemes are supported:
// 1. Normal (P=ofdm symbol with pilot, D=symbol without pilot)
//		UL: P P D D D D D D P D D D D D
//		DL: P P D D D D D D P D D D D D
// 2. Robust allocation:
//		UL: P D P D P D P D P D P D P D
//		DL: P D P D P D P D P D P D P D
// Robust pilot allocation might enable higher mcs, but takes 10% of resources
#define USE_ROBUST_PILOT 1

// UE regularly re-syncs to the sync sequence to estimate timing offset
// during this also the carrier frequency offset is estimated.
// estimation can estimate larger offsets than the cfo estimation with
// pilots (done during slot receive) but is inaccurate
// We filter the new estimate with the old (pilot based) cfo estimation
// Set this filter to [0 1] to tune estimation (1= solely based on new cfo)
#define SYNC_CFO_FILT_PARAM 0.8f

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


struct phy_config_s {
    float coarse_cfo_filt_param;
    int use_robust_pilot;

    int log_coarse_cfo_flag;
    char coarse_cfo_logfile[80];

    int log_cfo_flag;
    char cfo_logfile[80];
};

// global phy configuration struct
struct phy_config_s PHY_CONFIG;

#endif /* PHY_CONFIG_H_ */
