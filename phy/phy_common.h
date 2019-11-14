/*
 * phy_common.h
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#ifndef PHY_COMMON_H_
#define PHY_COMMON_H_

#include "phy_config.h"
#include "../mac/mac_channels.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// typedef for easier dlctrl slot access
typedef union {
	uint8_t byte;
	struct {
		uint8_t h4 : 4;
		uint8_t l4 : 4;
	};
} dlctrl_alloc_t;

enum {NO_PILOT, PILOT};		// definition for pilot_symbols variable


// Struct contains PHY variables common to UE and BS Phy layer
typedef struct {
	uint subframe;
	uint rx_symbol;
	uint8_t* pilot_sc;	// defines which subcarriers are used for pilots
	uint8_t* pilot_symbols; // stores whether the ofdm symbols within a slot contain pilots

	// hold subcarrier data in frequency domain
	// 1. Index: ofdm symbol idx.
	// 2. Index: subcarrier idx.
	float complex** txdata_f;
	float complex** rxdata_f;

	modem mcs_modem[8];	// array of modems for different mcs
	fec mcs_fec[8];		// array of encoders/decoders for different mcs
	fec_scheme mcs_fec_scheme[8];

	interleaver mcs_interlvr[8]; // array of interleavers for different mcs

	uint8_t userid;

} PhyCommon_s;

typedef PhyCommon_s* PhyCommon;

// Create the common phy struct
PhyCommon phy_init_common();

int _ofdm_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd);

// returns the Transport Block size of a UL/DL data slot in bits
int get_tbs_size(PhyCommon phy, uint mcs);

// Modulate the given data to the frequency domain data of the Phy object
// returns the number of symbols that have been generated
void phy_mod(PhyCommon common, uint first_sc, uint last_sc, uint first_symb, uint last_symb,
			 uint mcs, uint8_t* data, uint buf_len, uint* written_samps);

// Symbol demapper with soft decision
// returns an array with n llr values for each demapped symbol and the number of demapped bits
void phy_demod_soft(PhyCommon common, uint first_sc, uint last_sc, uint first_symb, uint last_symb,
					uint mcs, uint8_t* llr, uint num_llr, uint* written_samps);

/*
 * Generate the subcarrier allocation table and set corresponding variables in the phy
 * struct.
 */
void gen_sc_alloc(PhyCommon phy);


#endif /* PHY_COMMON_H_ */
