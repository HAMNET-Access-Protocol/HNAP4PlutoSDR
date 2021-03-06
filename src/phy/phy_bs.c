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

#include "phy_bs.h"

#ifdef PHY_TEST_BER
#include "../runtime/test.h"
#endif

// Forward declarations of local helper functions
int phy_bs_proc_rach(PhyBS phy, int timing_diff);
int _bs_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd);


// callback for OFDM RACH receiver object
// is called for every symbol that is received
int _ofdm_rx_rach_cb(float complex* X,unsigned char* p, uint M, void* userd)
{
	PhyBS phy = (PhyBS)userd;
	memcpy(phy->rach_buffer,X,sizeof(float complex)*nfft);
	phy_bs_proc_rach(phy, phy->rach_timing);

	return 0;
}

PhyBS phy_bs_init()
{
	PhyBS phy = calloc(sizeof(struct PhyBS_s),1);

	phy->common = phy_common_init();
#if USE_ROBUST_PILOT
	gen_pilot_symbols_robust(phy->common, 1);
#else
	gen_pilot_symbols(phy->common, 1);
#endif
	// Create OFDM frame generator: nFFt, CPlen, taperlen, subcarrier alloc
	phy->fg = ofdmframegen_create(nfft, cp_len, 0, phy->common->pilot_sc);

	// Create OFDM receiver
	phy->fs_rach = NULL;

    // alloc buffer for dl control slot
    phy->dlctrl_buf = calloc((num_data_sc+num_pilot_sc)*DLCTRL_LEN/8, 1);

	// Alloc memory for slot assignments
	phy->ulslot_assignments = malloc(2*sizeof(uint8_t*));
	phy->ulctrl_assignments = malloc(2*sizeof(uint8_t*));

	for (int i=0; i<2; i++) {
		phy->ulslot_assignments[i] = calloc(sizeof(uint8_t),NUM_SLOT);
		phy->ulctrl_assignments[i] = calloc(sizeof(uint8_t),NUM_ULCTRL_SLOT);
	}

    // buffer for ofdm symbol allocation
    phy->ul_symbol_alloc = malloc(sizeof(uint8_t*)*2);
	phy->ul_symbol_alloc[0] = calloc(sizeof(uint8_t)*SUBFRAME_LEN,1);
    phy->ul_symbol_alloc[1] = calloc(sizeof(uint8_t)*SUBFRAME_LEN,1);

    // allocate memory for rach_buffer
    phy->rach_buffer = calloc(sizeof(float complex)*nfft,1);

    // Set RX position
    phy->common->rx_symbol = SUBFRAME_LEN - DL_UL_SHIFT - DL_UL_SHIFT_COMP_BS;
    phy->common->rx_subframe = FRAME_LEN -1;
    phy->rach_timing = 0;
    phy->rach_remaining_samps = 0;

    phy->txgain = -128;
    phy->rxgain = -128;

    return phy;
}

void phy_bs_destroy(PhyBS phy)
{
	phy_common_destroy(phy->common);
	ofdmframegen_destroy(phy->fg);
	if (phy->fs_rach!=NULL)
		ofdmframesync_destroy(phy->fs_rach);

	free(phy->dlctrl_buf);

	for (int i=0; i<2; i++) {
		free(phy->ulslot_assignments[i]);
		free(phy->ulctrl_assignments[i]);
	}
	free(phy->ulslot_assignments);
	free(phy->ulctrl_assignments);

	free(phy->ul_symbol_alloc[0]);
	free(phy->ul_symbol_alloc[1]);
	free(phy->ul_symbol_alloc);

	free(phy);
}

void phy_bs_set_mac_interface(PhyBS phy, struct MacBS_s* mac)
{
	phy->mac = mac;
}

// Set the condition which is used to signal the slot processing thread
void phy_bs_set_rx_slot_th_signal(PhyBS phy, pthread_cond_t* cond)
{
    phy->rx_slot_signal = cond;
}

void phy_bs_write_sync_info(PhyBS phy, float complex* txbuf_time) {
    PhyCommon common = phy->common;

    uint8_t *repacked_b;
    uint bytes_written = 0;

    uint mcs = 0;
    // fixed MCS 0: r=1/2, bps=2, 16tail bits.
    uint32_t blocksize = get_ulctrl_slot_size(phy->common);

    LogicalChannel chan = lchan_create(blocksize / 8, CRC8);

    chan->data[0] = phy->rxgain;
    chan->data[1] = phy->txgain;
    chan->writepos = 2;
    lchan_calc_crc(chan);

    // scrambling
    scramble_data((uint8_t*)chan->data,chan->payload_len);

    // encode channel
    uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs], blocksize / 8);
    uint8_t *enc_b = malloc(enc_len);
    fec_encode(common->mcs_fec[mcs], blocksize / 8, chan->data, enc_b);

    // repack bytes so that each array entry can be mapped to one symbol
    int num_repacked = enc_len * 8 / modem_get_bps(common->mcs_modem[mcs]);
    repacked_b = malloc(num_repacked);
    liquid_repack_bytes(enc_b, 8, enc_len, repacked_b, modem_get_bps(common->mcs_modem[mcs]), num_repacked,
                        &bytes_written);

    // modulate signal
    uint written_samps = 0;
    float complex subcarriers[nfft];
    for (int i=0; i<nfft; i++) {
        if (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA && written_samps<num_repacked) {
            modem_modulate(common->mcs_modem[mcs],(uint)repacked_b[written_samps++], &subcarriers[i]);
        }
    }
    // write symbol in time domain buffer
    ofdmframegen_writesymbol(phy->fg,subcarriers,txbuf_time);
}


TIMECHECK_CREATE(timecheck_tx);
TIMECHECK_CREATE(check_mod);
TIMECHECK_CREATE(check_fec_tx);
TIMECHECK_CREATE(check_interl_tx);
// create phy data channel in frequency domain
int phy_map_dlslot(PhyBS phy, LogicalChannel chan, uint subframe, uint8_t slot_nr, uint userid, uint mcs)
{
    TIMECHECK_INIT(check_mod,"bs.tx_slot.mod",10000);
    TIMECHECK_INIT(check_fec_tx,"bs.tx_slot.fec",10000);
    TIMECHECK_INIT(check_interl_tx,"bs.tx_slot.interleaver",10000);
    TIMECHECK_INIT(timecheck_tx,"bs.tx_slot",10000);

	PhyCommon common = phy->common;

	uint8_t* repacked_b;
	uint bytes_written=0;
	uint32_t blocksize = get_tbs_size(phy->common, mcs);

	if (blocksize/8 != chan->payload_len) {
		printf("Error: Wrong TBS\n");
		return -1;
	}

#ifdef PHY_TEST_BER
	memcpy(phy_dl[subframe%2][slot_nr], chan->data, chan->payload_len);
#endif
    TIMECHECK_START(timecheck_tx);
    TIMECHECK_START(check_fec_tx);
	// encode channel
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],chan->payload_len);
	uint8_t* enc_b = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], blocksize/8, chan->data, enc_b);
    TIMECHECK_STOP(check_fec_tx);
    TIMECHECK_START(check_interl_tx);
	//interleaving
	uint8_t* interleaved_b = malloc(enc_len);
	interleaver_encode(common->mcs_interlvr[mcs],enc_b,interleaved_b);
    TIMECHECK_STOP(check_interl_tx);
    TIMECHECK_START(check_mod);
	// repack bytes so that each array entry can be mapped to one symbol
	int num_repacked = ceil(enc_len*8.0/modem_get_bps(common->mcs_modem[mcs]));
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(interleaved_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*slot_nr;
	uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(slot_nr+1)-2;

	// modulate signal
	phy_mod(phy->common,subframe,0,nfft-1,first_symb,last_symb, mcs, repacked_b, num_repacked, &total_samps);
    TIMECHECK_STOP(check_mod);
    TIMECHECK_STOP(timecheck_tx);
	free(interleaved_b);
	free(enc_b);
	free(repacked_b);

    TIMECHECK_INFO(timecheck_tx);
    TIMECHECK_INFO(check_mod);
    TIMECHECK_INFO(check_interl_tx);
    TIMECHECK_INFO(check_fec_tx);
	return 0;
}

void phy_map_dlctrl(PhyBS phy, uint subframe)
{
	PhyCommon common = phy->common;

	// use MCS0 for modulation
	uint mcs = 0;

	uint buf_size = (2*NUM_SLOT+NUM_ULCTRL_SLOT)/2;

	// add CRC
	phy->dlctrl_buf[buf_size].byte = crc_generate_key(LIQUID_CRC_8, (uint8_t*)phy->dlctrl_buf,buf_size);

	// scrambling
	scramble_data((uint8_t*)phy->dlctrl_buf,buf_size+1);

	// encode data
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],buf_size+1);
	uint8_t* buf_enc = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], buf_size+1,(uint8_t*)phy->dlctrl_buf, buf_enc);

	// repack bytes and modulate them
	uint bytes_written;
	int num_repacked = enc_len*8/modem_get_bps(common->mcs_modem[mcs]);
	uint8_t* repacked_b = malloc(num_repacked);
	liquid_repack_bytes((uint8_t*)buf_enc,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	phy_mod(common, subframe, 0, nfft-1, 0, DLCTRL_LEN-1, mcs, repacked_b, num_repacked, &total_samps);

	free(buf_enc);
	free(repacked_b);
}

//Set the assignments of Downlink data slots
void phy_assign_dlctrl_dd(PhyBS phy, uint8_t* slot_assignment)
{
	for (int i=0; i<NUM_SLOT; i+=2) {
	    phy->dlctrl_buf[i/2].h4 = slot_assignment[i];
	    phy->dlctrl_buf[i/2].l4 = slot_assignment[i+1];
	}
}

// Set the assignments of Uplink data slots
void phy_assign_dlctrl_ud(PhyBS phy, uint subframe, uint8_t* slot_assignment)
{
	memcpy(phy->ulslot_assignments[subframe], slot_assignment, NUM_SLOT);

	for (int i=0; i<NUM_SLOT; i+=2) {
	    phy->dlctrl_buf[NUM_SLOT/2+i/2].h4 = slot_assignment[i];
	    phy->dlctrl_buf[NUM_SLOT/2+i/2].l4 = slot_assignment[i+1];
	}

	// TODO generalize this
	// set allocation with ofdm symbol granularity. Used to pick correct receiver
	memset(&phy->ul_symbol_alloc[subframe][0], slot_assignment[0], SLOT_LEN);
	memset(&phy->ul_symbol_alloc[subframe][SLOT_LEN+1], slot_assignment[1], SLOT_LEN);
	memset(&phy->ul_symbol_alloc[subframe][2*(SLOT_LEN+1)+4], slot_assignment[2], SLOT_LEN);
	memset(&phy->ul_symbol_alloc[subframe][3*(SLOT_LEN+1)+4], slot_assignment[3], SLOT_LEN);
}

// Set the assignments of Uplink control slots
void phy_assign_dlctrl_uc(PhyBS phy, uint subframe, uint8_t* slot_assignment)
{
	memcpy(phy->ulctrl_assignments[subframe], slot_assignment, NUM_ULCTRL_SLOT);

	for (int i=0; i<NUM_ULCTRL_SLOT; i+=2) {
	    phy->dlctrl_buf[NUM_SLOT+i/2].h4 = slot_assignment[i];
	    phy->dlctrl_buf[NUM_SLOT+i/2].l4 = slot_assignment[i+1];
	}

	// set assignments on ofdm symbol granularity
	phy->ul_symbol_alloc[subframe][2*(SLOT_LEN+1)] = slot_assignment[0];
	phy->ul_symbol_alloc[subframe][2*(SLOT_LEN+1)+2] = slot_assignment[1];

}

// Decode a PHY ul slot and call the MAC callback function
void phy_bs_proc_slot(PhyBS phy, uint slotnr)
{
	PhyCommon common = phy->common;
	uint sfn = common->rx_subframe %2;
	//get user that was supposed to send in this slot
	uint userid =  phy->ulslot_assignments[sfn][slotnr];

	if (userid==0) {
		return; // Slot was not assigned. Nothing to decode
	}
	if (phy->mac->UE[userid]==NULL) {
		// User was assigned but does not exist in config. Should not happen
		LOG(ERR,"[PHY BS] cannot decode data for user %d. User does not exist!\n",userid);
		return;
	}

	uint mcs = phy->mac->UE[userid]->ul_mcs; // TODO create method to fetch this?
	uint32_t blocksize = get_tbs_size(common, mcs);

	uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
	uint8_t* demod_buf = malloc(buf_len);

	// demodulate signal
	uint written_samps = 0;
	uint first_symb = (SLOT_LEN+1)*slotnr;
	uint last_symb = (SLOT_LEN+1)*(slotnr+1)-2; //TODO use more constants and explain how to calc this
	// slot 3 and 4 are shifted back since the ULCTRL lies between slot 2 and 3
	if(slotnr>=2) {
		first_symb += 4;
		last_symb +=4;
	}
	phy_demod_soft(common, 0, nfft-1, first_symb, last_symb, mcs,
				   demod_buf, buf_len, &written_samps);

	//deinterleaving
	uint8_t* deinterleaved_b = malloc(buf_len);
	interleaver_decode_soft(common->mcs_interlvr[mcs],demod_buf,deinterleaved_b);

	// decoding
	LogicalChannel chan = lchan_create(blocksize/8,CRC16);
	fec_decode_soft(common->mcs_fec[mcs], blocksize/8, deinterleaved_b, chan->data);

#ifdef PHY_TEST_BER
	uint32_t num_biterr = 0;
	for (int i=0; i<chan->payload_len;i++)
		num_biterr += liquid_count_ones(phy_ul[sfn][slotnr][i]^chan->data[i]);
	phy_ul_tot_bits += chan->payload_len*8;
	phy_ul_biterr += num_biterr;
#endif

	// pass to upper layer
	if(!mac_bs_rx_channel(phy->mac,chan, userid)) {
		// log when crc check failed
		ofdmframesync fs = mac_bs_get_receiver(phy->mac,userid);
		if (fs!=NULL)
			LOG_SFN_PHY(TRACE,"cfo was: %.3fHz\n",ofdmframesync_get_cfo(fs)*samplerate/6.28);
	}
	free(deinterleaved_b);
	free(demod_buf);
}

// Decode a PHY ul ctrl slot and call the MAC callback function
void phy_bs_proc_ulctrl(PhyBS phy, uint slotnr)
{
	PhyCommon common = phy->common;
	uint sfn = common->rx_subframe %2;

	//get user that was supposed to send in this slot
	uint userid =  phy->ulctrl_assignments[sfn][slotnr];
	uint mcs = 0; // CTRL slots use MCS 0
	uint32_t blocksize = get_ulctrl_slot_size(common);

	uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
	uint8_t* demod_buf = malloc(buf_len);

	// demodulate signal
	uint written_samps = 0;
	uint first_symb = (SLOT_LEN+1)*2 + 2*slotnr;
	uint last_symb  = (SLOT_LEN+1)*2 + 2*slotnr; //TODO use more constants and explain how to calc this

	phy_demod_soft(common, 0, nfft-1, first_symb, last_symb, mcs,
				   demod_buf, buf_len, &written_samps);

	// decoding
	LogicalChannel chan = lchan_create(blocksize/8,CRC8);
	fec_decode_soft(common->fec_ctrl, blocksize/8, demod_buf, chan->data);

	// pass to upper layer
	mac_bs_rx_channel(phy->mac,chan, userid);
	free(demod_buf);
}


int phy_bs_proc_rach(PhyBS phy, int timing_diff)
{
	PhyCommon common = phy->common;

	uint mcs = 0; // CTRL slots use MCS 0
	uint32_t blocksize = get_ulctrl_slot_size(common);

    if (timing_diff<0) {
        // Client sent too early. This should not happen, ignore the request
        LOG(WARN,"[PHY BS] Some client sent Assoc request too early! ignore\n");
        phy->fs_rach = NULL;
        return 0;
    }

    uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
	uint8_t* demod_buf = malloc(buf_len);

	// demodulate signal
	uint written_samps = 0, symbol = 0;
	for (int i=0; i<nfft; i++) {
		if (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA && written_samps<buf_len) {
			modem_demodulate_soft(common->mcs_modem[mcs], phy->rach_buffer[i], &symbol, &demod_buf[written_samps]);
			written_samps += modem_get_bps(common->mcs_modem[mcs]);
		}
	}

	// decoding
	LogicalChannel chan = lchan_create(blocksize/8,CRC8);
	fec_decode_soft(common->fec_ctrl, blocksize/8, demod_buf, chan->data);

	free(demod_buf);

	// TODO Fix definition of Associate Request message
	if (lchan_verify_crc(chan)) {
		uint8_t rach_userid = chan->data[0];
		uint8_t rach_try_cnt = chan->data[1];
		// change callback from RACH cb to normal cb
		ofdmframesync_set_cb(phy->fs_rach,_bs_rx_symbol_cb,phy);
		mac_bs_add_new_ue(phy->mac,rach_userid, rach_try_cnt, phy->fs_rach, timing_diff);
	} else {
	    LOG(WARN,"[PHY BS] assoc request could not be decoded. invalid CRC!\n");
	}
	lchan_destroy(chan);

	//set to NULL to ensure RA procedure does not use sync object anymore
	phy->fs_rach = NULL;
	return 1;
}

// callback for OFDM receiver
// is called for every symbol that is received
int _bs_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd)
{
	PhyBS phy = (PhyBS)userd;
	PhyCommon common = phy->common;

	memcpy(common->rxdata_f[common->rx_symbol],X,sizeof(float complex)*nfft);

	switch (common->rx_symbol) {
	case (SLOT_LEN-1):
		// finished receiving one of the UL slots
#ifdef USE_RX_SLOT_THREAD
            phy->rx_slot_nr = 0;
            if (phy->rx_slot_signal)
                pthread_cond_signal(phy->rx_slot_signal);
            else
#endif
                phy_bs_proc_slot(phy,0);
		break;
	case (2*SLOT_LEN):
		// finished receiving one of the UL slots
#ifdef USE_RX_SLOT_THREAD
            phy->rx_slot_nr = 1;
            if (phy->rx_slot_signal)
                pthread_cond_signal(phy->rx_slot_signal);
            else
#endif
		        phy_bs_proc_slot(phy, 1);
		break;
	case 2*(SLOT_LEN+1):
		// finished receiving first ULCTRL slot
		phy_bs_proc_ulctrl(phy, 0);
		break;
	case 2*(SLOT_LEN+1)+2:
		// finished receiving second ULCTRL slot
		phy_bs_proc_ulctrl(phy,1);
		break;
	case 2*(SLOT_LEN+1)+4+SLOT_LEN-1:
		// finished receiving one of the UL slots
#ifdef USE_RX_SLOT_THREAD
            phy->rx_slot_nr = 2;
            if (phy->rx_slot_signal)
                pthread_cond_signal(phy->rx_slot_signal);
            else
#endif
		        phy_bs_proc_slot(phy,2);
		break;
	case 3*(SLOT_LEN+1)+4+SLOT_LEN-1:
		// finished receiving one of the UL slots
#ifdef USE_RX_SLOT_THREAD
            phy->rx_slot_nr = 3;
            if (phy->rx_slot_signal)
                pthread_cond_signal(phy->rx_slot_signal);
            else
#endif
		        phy_bs_proc_slot(phy,3);
		break;
	default:
		break;
	}

	// Debug log
	/*char name[30];
	sprintf(name,"rxF/rxF_%d_%d.m",common->rx_subframe,common->rx_symbol-1);
	if (common->rx_symbol!=-1) {
	LOG_MATLAB_FC(DEBUG,X, NFFT, name);
	}*/

	return 0;
}

// Create one OFDM symbol in time domain
// Subcarriers in frequency have to be set beforehand!
void phy_bs_write_symbol(PhyBS phy, float complex* txbuf_time)
{
	PhyCommon common = phy->common;
	uint tx_symb = common->tx_symbol;
	uint sfn = common->tx_subframe %2;

	// check if we have to add synch sequence in this subframe
	if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-1-SYNC_SYMBOLS) {
		ofdmframegen_reset(phy->fg);
		ofdmframegen_write_S0a(phy->fg, txbuf_time);
	} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-1-SYNC_SYMBOLS+1) {
		ofdmframegen_write_S0b(phy->fg, txbuf_time);
	} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-1-SYNC_SYMBOLS+2) {
        ofdmframegen_write_S1(phy->fg, txbuf_time);
    } else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-1-SYNC_SYMBOLS+3) {
        phy_bs_write_sync_info(phy, txbuf_time);
	} else if (common->pilot_symbols_tx[tx_symb] == PILOT) {
		ofdmframegen_writesymbol(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
	} else {
		ofdmframegen_writesymbol_nopilot(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
	}

	// clear frequency domain memory of the written symbol, to avoid sending garbage
	// when the symbol is not overwritten in the next subframe
	memset(common->txdata_f[sfn][tx_symb],0,sizeof(float complex)*nfft);

	// Update subframe and symbol counter
	common->tx_symbol++;
	if (common->tx_symbol>=SUBFRAME_LEN) {
		common->tx_symbol = 0;
		common->tx_subframe = (common->tx_subframe+1) % FRAME_LEN;
	}
}

// Main PHY receive function
//receive one ofdm symbol amount of samples and process them
// NOTE: in constrast to the phyUE receive function, the amount of processed
// 		 samples per call is fixed
void phy_bs_rx_symbol(PhyBS phy, float complex* rxbuf_time)
{
	PhyCommon common = phy->common;
	uint rx_sym = nfft+cp_len;
	uint sfn = common->rx_subframe;

	if (sfn == 0 && common->rx_symbol==SUBFRAME_LEN-SLOT_LEN-2) {
		// First symbol of random access slot. Config sync objects
		if (phy->fs_rach!=NULL) {
			ofdmframesync_reset(phy->fs_rach); // if we didnt find a new user in last RACH, sync object still exists. Reset it
		} else {
			phy->fs_rach = ofdmframesync_create(nfft,cp_len,0,phy->common->pilot_sc,_ofdm_rx_rach_cb, phy);
		}
	}

	if (sfn == 0 && common->rx_symbol>=SUBFRAME_LEN-SLOT_LEN-2) {
		// RA slot. Try to find sync sequence
		if (phy->fs_rach && !ofdmframesync_is_synced(phy->fs_rach)) {
			int offset = ofdmframesync_find_data_start(phy->fs_rach, rxbuf_time, rx_sym);
			if (offset !=-1) {
				phy->rach_timing = offset+(nfft+cp_len)*(common->rx_symbol-(SUBFRAME_LEN-SLOT_LEN+SYNC_SYMBOLS-1));
				LOG(INFO,"[PHY BS] detected preamble of association request in (%d %d)! offset %d. cfo %.3f Hz\n",
													common->rx_subframe, common->rx_symbol, phy->rach_timing, ofdmframesync_get_cfo(phy->fs_rach)*samplerate/6.28);
				// rach can be unaligned to symbol boundaries. receive rx_sym-offset samps
				ofdmframesync_execute(phy->fs_rach, rxbuf_time+offset,rx_sym-offset);
			}
		} else if (phy->fs_rach){
			// receive the last samps of the association request
            if (phy->rach_timing <= 0)
				ofdmframesync_execute(phy->fs_rach, rxbuf_time, nfft+cp_len);
            else
				ofdmframesync_execute(phy->fs_rach, rxbuf_time, phy->rach_timing % rx_sym);
		}
	} else {
		// not in RA slot. Do normal receive
		uint userid = phy->ul_symbol_alloc[sfn%2][common->rx_symbol];
		ofdmframesync fs = mac_bs_get_receiver(phy->mac,userid);
		if (fs!=NULL) {
			// if this is the first symbol of a slot, soft reset the
			// sync object
			uint prev_rx_symb = (common->rx_symbol-1) % SUBFRAME_LEN;
			if (common->rx_symbol == 0 || phy->ul_symbol_alloc[sfn%2][prev_rx_symb]==0) {
				ofdmframesync_reset_soft(fs);
			}

			if (common->pilot_symbols_rx[common->rx_symbol] == PILOT) {
				ofdmframesync_reset_msequence(fs);
				ofdmframesync_execute(fs,rxbuf_time,rx_sym);
				LOG_SFN_PHY(TRACE,"[PHY BS] cfo was: %.3fHz\n",ofdmframesync_get_cfo(fs)*samplerate/6.28);
				//ofdmframesync_set_cfo(fs,0); // TODO cfo estimation. Currently not working since we often receive if no data is sent. -> wrong pilot -> wrong cfo
			} else {
				ofdmframesync_execute_nopilot(fs,rxbuf_time,rx_sym);
			}
		}
	}

	// Update the tx counters
	common->rx_symbol++;
	if (common->rx_symbol == SUBFRAME_LEN) {
		common->rx_subframe = (common->rx_subframe+1) % FRAME_LEN;
		common->rx_symbol = 0;
	}
}
