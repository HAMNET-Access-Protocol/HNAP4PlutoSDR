/*
 * phy_ue.c
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#include "phy_ue.h"

#include "../log.h"

// Init the PhyUE struct
PhyUE phy_init_ue()
{
	PhyUE phy = malloc(sizeof(PhyUE_s));

	phy->common = phy_init_common();

	// Create OFDM frame generator: nFFt, CPlen, taperlen, subcarrier alloc
	phy->fg = ofdmframegen_create(NFFT, CP_LEN, 0, phy->common->pilot_sc);

	// Create OFDM receiver
	phy->fs = ofdmframesync_create(NFFT, CP_LEN, 0, phy->common->pilot_sc, _ofdm_rx_symbol_cb, phy);

	phy->dlslot_assignments = calloc(1,NUM_SLOT);
	phy->ulslot_assignments = calloc(1,NUM_SLOT);
	phy->ulctrl_assignments = calloc(1,NUM_ULCTRL_SLOT);

	phy->mac_rx_cb = NULL;

	phy->mcs_dl = 0;
	phy->mcs_ul = 0;

	phy->prev_cfo = 0.0;

    return phy;
}

// Sets the callback function that is called after a phy slot was received
void phy_ue_set_mac_cb(PhyUE phy, void (*mac_rx_cb)(LogicalChannel))
{
	phy->mac_rx_cb = mac_rx_cb;
}

// Setter function for Downlink MCS
void phy_ue_set_mcs_dl(PhyUE phy, uint mcs)
{
	phy->mcs_dl = mcs;
}

// Setter function for Uplink MCS
void phy_ue_set_mcs_ul(PhyUE phy, uint mcs)
{
	phy->mcs_ul = mcs;
}

// Searches for the initial sync sequence
// returns -1 if no sync found, else the sample index of the ofdm symbol after the sync sequence
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples)
{
	PhyCommon common = phy->common;

	int offset = ofdmframesync_find_data_start(phy->fs,rxbuf_time,num_samples);
	if (offset!=-1) {
		common->rx_symbol = NFFT - 1; //there is one more guard symbol in this subframe to receive
		common->subframe = 0;

		//apply filtering of coarse CFO estimation if we have old estimates
		if (phy->has_synced_once == 0) {
			phy->has_synced_once = 1;
			phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
		} else {
			float new_cfo = ofdmframesync_get_cfo(phy->fs);
			float cfo_filt = 0.9*phy->prev_cfo + (1-0.9)*new_cfo;
			ofdmframesync_set_cfo(phy->fs,cfo_filt);
		}
	}
	return offset;
}

// Process the Symbols received in a Downlink Control Slot
// returns 1 if DL CTRL slot was successfully decoded, else 0
// furthermore sets the phy dl/ul_assignments variable accordingly
int phy_ue_proc_dlctrl(PhyUE phy)
{
	const PhyCommon common = phy->common;
	uint dlctrl_size = (NUM_SLOT*2 + NUM_ULCTRL_SLOT)/2;

	// demodulate signal.
	uint llr_len = 2*DLCTRL_LEN*(NFFT-NUM_GUARD);
	uint8_t* llr_buf = malloc(llr_len);
	uint total_samps = 0;
	phy_demod_soft(common, 0, NFFT-1, 0, DLCTRL_LEN-1, 0, llr_buf, llr_len, &total_samps);

	// soft decoding
	dlctrl_alloc_t* dlctrl_buf = malloc(dlctrl_size+1);
	fec_decode_soft(common->mcs_fec[0],dlctrl_size+1, llr_buf, (uint8_t*)dlctrl_buf);

	// verify CRC
	if (!crc_validate_message(LIQUID_CRC_8, (uint8_t*)dlctrl_buf, dlctrl_size, dlctrl_buf[dlctrl_size].byte)) {
		LOG(DEBUG,"[PHY] DLCTRL slot could not be decoded!\n");
		return 0; // CRC not valid
	}

	// pass CAICH data to mac control
	uint idx = 0;
	for (int i=0; i<NUM_SLOT/2; i++) {
		phy->dlslot_assignments[2*i  ] = (dlctrl_buf[idx].h4 == common->userid) ? 1 : 0;
		phy->dlslot_assignments[2*i+1] = (dlctrl_buf[idx++].l4 == common->userid) ? 1 : 0;
	}
	for (int i=0; i<NUM_SLOT/2; i++) {
		phy->ulslot_assignments[2*i  ] = (dlctrl_buf[idx].h4 == common->userid) ? 1 : 0;
		phy->ulslot_assignments[2*i+1] = (dlctrl_buf[idx++].l4 == common->userid) ? 1 : 0;
	}
	for (int i=0; i<NUM_ULCTRL_SLOT/2; i++) {
		phy->ulctrl_assignments[2*i  ] = (dlctrl_buf[idx].h4 == common->userid) ? 1 : 0;
		phy->ulctrl_assignments[2*i+1] = (dlctrl_buf[idx++].l4 == common->userid) ? 1 : 0;
	}

	free(llr_buf);
	free(dlctrl_buf);
	return 1;
}

// Decode a PHY dl slot and call the MAC callback function
void phy_ue_proc_slot(PhyUE phy, uint slotnr)
{
	PhyCommon common = phy->common;

	if (phy->dlslot_assignments[slotnr] == 1) {

		uint32_t blocksize = get_tbs_size(common, phy->mcs_dl);

		uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[phy->mcs_dl],blocksize/8);
		uint8_t* demod_buf = malloc(buf_len);

		LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

		// demodulate signal
		uint written_samps = 0;
		uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*slotnr;
		uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(slotnr+1)-2;
		phy_demod_soft(common, 0, NFFT-1, first_symb, last_symb, phy->mcs_dl,
					   demod_buf, buf_len, &written_samps);

		// decoding
		uint8_t* interleaved_b = malloc(blocksize/8);
		fec_decode_soft(common->mcs_fec[phy->mcs_dl], blocksize/8, demod_buf, interleaved_b);

		//deinterleaving
		chan->userid = common->userid;
		chan->payload_len = blocksize/8;
		chan->writepos = slotnr; //TODO: this is only for simulation to ind the slot nr
		chan->data = malloc(blocksize/8);
		interleaver_decode(common->mcs_interlvr[phy->mcs_dl],interleaved_b,chan->data);

		// pass to upper layer
		phy->mac_rx_cb(chan);

		free(interleaved_b);
		free(demod_buf);

	}
}
/*void phy_ue_rx_subframe(PhyUE phy, float complex* rxbuf_time)
{
	const PhyCommon common = phy->common;

	if (common->rx_symbol != DLCTRL_LEN)
		printf("[PHY] error, unexpected rxsymbol index: %d\n",common->rx_symbol);

	uint subframe_len = SUBFRAME_LEN;
	if (common->subframe == FRAME_LEN-1) {
		subframe_len -= SYNC_SYMBOLS;
	}
	rxbuf_time += DLCTRL_LEN*(NFFT+CP_LEN);
	for (int i=DLCTRL_LEN; i<subframe_len; i++) {
		if (common->pilot_symbols[i] == PILOT) {
			ofdmframesync_execute(phy->fs, rxbuf_time,(NFFT+CP_LEN));
		} else {
			ofdmframesync_execute_nopilot(phy->fs, rxbuf_time, NFFT+CP_LEN);
		}
		rxbuf_time += NFFT+CP_LEN;
	}

	if (common->rx_symbol != subframe_len)
		printf("[PHY] Error: did not receive all ofdm symbols!\n");

	if (common->subframe == FRAME_LEN-1) {
		ofdmframesync_reset(phy->fs);
		int offset = ofdmframesync_find_data_start(phy->fs,rxbuf_time,(NFFT+CP_LEN)*SYNC_SYMBOLS);
		printf("Sync seq offset: %d \n",offset);
		if (offset == -1) {
			printf("[PHY] Did not find sync seq!\n");
			return;
		}

	}

	uint32_t blocksize = get_tbs_size(common, phy->mcs_dl);

	// TODO: fix allocation size. currently hardcoded additional bytes
	uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[phy->mcs_dl],blocksize/8);
	uint8_t* demod_buf = malloc(buf_len);

	for (int i=0; i<NUM_SLOT; i++) {
		if (phy->dlslot_assignments[i] == 1) {
			LogicalChannel chan = malloc(sizeof(LogicalChannel_s));

			// demodulate signal
			uint written_samps = 0;
			uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*i;
			uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(i+1)-2;
			phy_demod_soft(common, 0, NFFT-1, first_symb, last_symb, phy->mcs_dl,
						   demod_buf, buf_len, &written_samps);

			// decoding
			chan->userid = common->userid;
			chan->payload_len = blocksize/8;
			chan->writepos = i; //TODO: this is only for simulation to ind the slot nr
			chan->data = malloc(blocksize/8);
			fec_decode_soft(common->mcs_fec[phy->mcs_dl], blocksize/8, demod_buf, chan->data);

			// pass to upper layer
			phy->mac_rx_cb(chan);
		}
	}

	free(demod_buf);

	common->subframe = (common->subframe + 1) % FRAME_LEN;
}*/

// callback for OFDM receiver
// is called for every symbol that is received
int _ofdm_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd)
{
	PhyUE phy = (PhyUE)userd;
	PhyCommon common = phy->common;

	memcpy(common->rxdata_f[common->rx_symbol++],X,sizeof(float complex)*NFFT);

	switch (common->rx_symbol) {
	case DLCTRL_LEN:
		// finished receiving DLCTRL slot
		phy_ue_proc_dlctrl(phy);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1):
		// finished receiving one of the dl data slots
		phy_ue_proc_slot(phy,0);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*2:
		// finished receiving one of the dl data slots
		phy_ue_proc_slot(phy,1);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*3:
		// finished receiving one of the dl data slots
		phy_ue_proc_slot(phy,2);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*4:
		// finished receiving one of the dl data slots
		phy_ue_proc_slot(phy,3);
		break;
	default:
		break;
	}

	// sync sequence will follow. Reset framesync
	if ((common->subframe == 0) &&
			(common->rx_symbol == DLCTRL_LEN+1+SLOT_LEN*3)) {
		// store old cfo estimation
		phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
		ofdmframesync_reset(phy->fs);
	}

	// Debug log
	char name[30];
	sprintf(name,"rxF/rxF_%d_%d.m",common->subframe,common->rx_symbol-1);
	//if (common->rx_symbol == 5) {
	LOG_MATLAB_FC(DEBUG,X, NFFT, name);
	//}
	if (common->rx_symbol > NFFT-1) {
		common->rx_symbol = 0;
		common->subframe = (common->subframe + 1) % FRAME_LEN;
	}


	return 0;
}

// Main PHY receive function
//receive an arbitrary number of samples and process slots once they are received
void phy_ue_do_rx(PhyUE phy, float complex* rxbuf_time, uint num_samples)
{
	static uint8_t has_synced_once = 0;
	PhyCommon common = phy->common;

	while (num_samples > 0) {
		// find sync sequence
		if (!ofdmframesync_is_synced(phy->fs)) {
			int offset = phy_ue_initial_sync(phy,rxbuf_time,num_samples);
			if (offset!=-1) {
				num_samples -= offset;
				rxbuf_time += offset;
			} else {
				num_samples = 0;
			}
		} else {
			// receive symbols
			uint rx_sym = fmin(NFFT+CP_LEN,num_samples);
			if (common->pilot_symbols[common->rx_symbol] == PILOT) {
				ofdmframesync_execute(phy->fs,rxbuf_time,rx_sym);
			} else {
				ofdmframesync_execute_nopilot(phy->fs,rxbuf_time,rx_sym);
			}
			num_samples -= rx_sym;
			rxbuf_time += rx_sym;
		}
	}
}

// create phy ctrl slot
int phy_map_ulctrl(PhyUE phy, LogicalChannel chan, uint8_t slot_nr)
{
	PhyCommon common = phy->common;

	uint8_t* repacked_b;
	uint bytes_written=0;

	uint mcs=0;
	// fixed MCS 0: r=1/2, bps=2, 16tail bits.
	uint32_t blocksize = ((NFFT-NUM_GUARD-NUM_PILOT)*2-16)/2;

	if (blocksize/8 != chan->payload_len) {
		printf("Error: Wrong TBS\n");
		return -1;
	}

	// encode channel
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],chan->payload_len);
	uint8_t* enc_b = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], blocksize/8, chan->data, enc_b);

	// repack bytes so that each array entry can be mapped to one symbol
	int num_repacked = enc_len*8/modem_get_bps(common->mcs_modem[mcs]);
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(enc_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	uint first_symb = 2*slot_nr;	// slot 0 is mapped to symbol 0, slot 1 is mapped to symb 2.

	// modulate signal
	phy_mod(phy->common,0,NFFT-1,first_symb,first_symb, mcs, repacked_b, num_repacked, &total_samps);

	free(enc_b);
	free(repacked_b);
	return 0;
}

// create phy data slot in frequency domain
int phy_map_ulslot(PhyUE phy, LogicalChannel chan, uint8_t slot_nr, uint mcs)
{
	PhyCommon common = phy->common;

	uint8_t* repacked_b;
	uint bytes_written=0;
	uint32_t blocksize = get_tbs_size(phy->common, mcs);

	if (blocksize/8 != chan->payload_len) {
		printf("Error: Wrong TBS\n");
		return -1;
	}

	//interleaving
	uint8_t* interleaved_b = malloc(chan->payload_len);
	interleaver_encode(common->mcs_interlvr[mcs],chan->data,interleaved_b);

	// encode channel
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],chan->payload_len);
	uint8_t* enc_b = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], blocksize/8, interleaved_b, enc_b);

	// repack bytes so that each array entry can be mapped to one symbol
	int num_repacked = ceil(enc_len*8.0/modem_get_bps(common->mcs_modem[mcs]));
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(enc_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*slot_nr;
	uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(slot_nr+1)-2;

	// modulate signal
	phy_mod(phy->common,0,NFFT-1,first_symb,last_symb, mcs, repacked_b, num_repacked, &total_samps);

	free(interleaved_b);
	free(enc_b);
	free(repacked_b);
	return 0;
}

/*void phy_ue_write_subframe(PhyUE phy, float complex* txbuf_time)
{
	PhyCommon common = phy->common;

	// write the Downlink control channel to the subcarriers
	phy_map_dlctrl(phy);

	// do ofdm modulation
	for (int i=0; i<SUBFRAME_LEN; i++) {
		// check if we have to add synch sequence in this subframe
		if (common->subframe == 0 && i>=SUBFRAME_LEN-SYNC_SYMBOLS) {
			phy_make_syncsig(phy, txbuf_time);
			break;
		}

		if (common->pilot_symbols[i] == PILOT) {
		    ofdmframegen_writesymbol(phy->fg, common->txdata_f[i],txbuf_time);
		} else {
			ofdmframegen_writesymbol_nopilot(phy->fg, common->txdata_f[i],txbuf_time);
		}
		txbuf_time += NFFT+CP_LEN;
	}

	common->subframe = (common->subframe + 1) % FRAME_LEN;

}*/
