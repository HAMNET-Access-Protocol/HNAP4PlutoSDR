/*
 * phy_config.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_CONFIG_H_
#define PHY_CONFIG_H_


#define NFFT 64				// FFT size
#define CP_LEN 4			// Cyclic Prefix Size
#define NUM_DATA_SC 32
#define NUM_PILOT 8
#define NUM_GUARD 24
#define SAMPLERATE 256000	// sample rate in Hz
#define SLOT_LEN 14			// length of one data slot
#define NUM_SLOT 4			// number of slots per subframe
#define NUM_ULCTRL_SLOT 2	// number of UL control slots
#define SUBFRAME_LEN 64		// number of OFDM symbols per subframe
#define DLCTRL_LEN 2		// number of OFDM symbols for DL control info
#define SYNC_SYMBOLS 4		// number of OFDM symbols for synch signaling
#define FRAME_LEN 8			// number of subframes per frame
#define DL_UL_SHIFT 34		// number of ofdm symbols the UL is shifted behind
#define MAX_USER 16
#endif /* PHY_CONFIG_H_ */
