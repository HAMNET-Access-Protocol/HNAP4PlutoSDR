/*
 * phy_ue.c
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#include "phy_ue.h"

#include "../log.h"
#include "../mac/mac_config.h"
#include <pthread.h>

// callback declaration
int _ue_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd);

// Init the PhyUE struct
PhyUE phy_ue_init()
{
	PhyUE phy = malloc(sizeof(struct PhyUE_s));

	phy->common = phy_init_common();
	gen_pilot_symbols(phy->common, 0);

	// Create OFDM frame generator: nFFt, CPlen, taperlen, subcarrier alloc
	phy->fg = ofdmframegen_create(NFFT, CP_LEN, 0, phy->common->pilot_sc);

	// Create OFDM receiver
	phy->fs = ofdmframesync_create(NFFT, CP_LEN, 0, phy->common->pilot_sc, _ue_rx_symbol_cb, phy);

	// Alloc memory for slot assignments
	phy->dlslot_assignments = malloc(2*sizeof(uint8_t*));
	phy->ulslot_assignments = malloc(2*sizeof(uint8_t*));
	phy->ulctrl_assignments = malloc(2*sizeof(uint8_t*));

	for (int i=0; i<2; i++) {
		phy->dlslot_assignments[i] = calloc(sizeof(uint8_t),NUM_SLOT);
		phy->ulslot_assignments[i] = calloc(sizeof(uint8_t),NUM_SLOT);
		phy->ulctrl_assignments[i] = calloc(sizeof(uint8_t),NUM_ULCTRL_SLOT);
	}

    // buffer for ofdm symbol allocation
    phy->ul_symbol_alloc = malloc(sizeof(uint8_t*)*2);
	phy->ul_symbol_alloc[0] = malloc(sizeof(uint8_t)*SUBFRAME_LEN);
    phy->ul_symbol_alloc[1] = malloc(sizeof(uint8_t)*SUBFRAME_LEN);

	phy->mac_rx_cb = NULL;

	phy->mcs_dl = 0;

	phy->prev_cfo = 0.0;

	phy->rx_offset = 0;

	phy->rachuserid = -1;
	phy->userid = -1;

	// Init mutexes etc
	phy->tx_sync_cond = (pthread_cond_t)PTHREAD_COND_INITIALIZER;
	phy->tx_sync_lock = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;

    return phy;
}

// Sets the callback function that is called after a phy slot was received
// and the Mac UE instance
void phy_ue_set_mac_interface(PhyUE phy, void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel), struct MacUE_s* mac)
{
	phy->mac_rx_cb = mac_rx_cb;
	phy->mac = mac;
}

// Setter function for Downlink MCS
void phy_ue_set_mcs_dl(PhyUE phy, uint mcs)
{
	phy->mcs_dl = mcs;
}

// Searches for the initial sync sequence
// returns -1 if no sync found, else the sample index of the ofdm symbol after the sync sequence
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples)
{
	PhyCommon common = phy->common;

	int offset = ofdmframesync_find_data_start(phy->fs,rxbuf_time,num_samples);
	if (offset!=-1) {
		common->rx_symbol = SUBFRAME_LEN - 1; //there is one more guard symbol in this subframe to receive
		common->rx_subframe = 0;

		//apply filtering of coarse CFO estimation if we have old estimates
		if (phy->has_synced_once == 0) {
			// init TX counters once
			common->tx_active = 1;
			common->tx_symbol = SUBFRAME_LEN - DL_UL_SHIFT; //TODO clarify what happens for offset=0
			common->tx_subframe = 0;

			phy->has_synced_once = 1;
			phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
			LOG(INFO,"[PHY UE] Got sync! cfo: %.3fHz offset: %d samps\n",phy->prev_cfo*SAMPLERATE/6.28,offset);
		} else {
			float new_cfo = ofdmframesync_get_cfo(phy->fs);
			float cfo_filt = 0.1*phy->prev_cfo + (1-0.1)*new_cfo;
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
	uint sfn = common->rx_subframe % 2; // even or uneven subframe?

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

	// Set the decoded user assignments in the phy struct
	uint idx = 0;
	for (int i=0; i<NUM_SLOT/2; i++) {
		phy->dlslot_assignments[sfn][2*i  ] = (dlctrl_buf[idx].h4 == phy->userid ||
											   dlctrl_buf[idx].h4 == USER_BROADCAST) ? 1 : 0;
		phy->dlslot_assignments[sfn][2*i+1] = (dlctrl_buf[idx].l4 == phy->userid ||
											   dlctrl_buf[idx].l4 == USER_BROADCAST) ? 1 : 0;
		idx++;
	}
	for (int i=0; i<NUM_SLOT/2; i++) {
		phy->ulslot_assignments[sfn][2*i  ] = (dlctrl_buf[idx].h4 == phy->userid) ? 1 : 0;
		phy->ulslot_assignments[sfn][2*i+1] = (dlctrl_buf[idx].l4 == phy->userid) ? 1 : 0;
		idx++;
	}
	for (int i=0; i<NUM_ULCTRL_SLOT/2; i++) {
		phy->ulctrl_assignments[sfn][2*i  ] = (dlctrl_buf[idx].h4 == phy->userid) ? 1 : 0;
		phy->ulctrl_assignments[sfn][2*i+1] = (dlctrl_buf[idx].l4 == phy->userid) ? 1 : 0;
		idx++;
	}

	// Pass slot assignment to MAC
	mac_ue_set_assignments(phy->mac,phy->dlslot_assignments[sfn],
									phy->ulslot_assignments[sfn],
									phy->ulctrl_assignments[sfn]);

	free(llr_buf);
	free(dlctrl_buf);
	return 1;
}

// Decode a PHY dl slot and call the MAC callback function
void phy_ue_proc_slot(PhyUE phy, uint slotnr)
{
	PhyCommon common = phy->common;
	if (phy->dlslot_assignments[common->rx_subframe%2][slotnr] == 1) {

		uint32_t blocksize = get_tbs_size(common, phy->mcs_dl);

		uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[phy->mcs_dl],blocksize/8);
		uint8_t* demod_buf = malloc(buf_len);

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
		LogicalChannel chan = lchan_create(blocksize/8,CRC16);
		interleaver_decode(common->mcs_interlvr[phy->mcs_dl],interleaved_b,chan->data);

		// pass to upper layer
		phy->mac_rx_cb(phy->mac, chan);

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
int _ue_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd)
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
	if ((common->rx_subframe == 0) &&
			(common->rx_symbol == DLCTRL_LEN+1+SLOT_LEN*3)) {
		// store old cfo estimation
		phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
		ofdmframesync_reset(phy->fs);
	}

	// Debug log
	char name[30];
	sprintf(name,"rxF/rxF_%d_%d.m",common->rx_subframe,common->rx_symbol-1);
	if (common->rx_symbol<64) {
	LOG_MATLAB_FC(DEBUG,X, NFFT, name);
	}
	if (common->rx_symbol >= SUBFRAME_LEN) {
		common->rx_symbol = 0;
		common->rx_subframe = (common->rx_subframe + 1) % FRAME_LEN;
	}

	return 0;
}

// Create the symbol containing association request data
void phy_ue_create_assoc_request(PhyUE phy, float complex* txbuf_time)
{
	PhyCommon common = phy->common;

	uint8_t* repacked_b;
	uint bytes_written=0;

	uint mcs=0;
	// fixed MCS 0: r=1/2, bps=2, 16tail bits.
	uint32_t blocksize = get_ulctrl_slot_size(phy->common);

	// TODO generate a defined struct for Association Request message
	LogicalChannel chan = lchan_create(blocksize/8, CRC8);
	if (phy->rachuserid == -1) {
		// RA procedure hasnt started. Select a random ID first
		phy->rachuserid = rand() % MAX_USER;
	}
	chan->data[0] = (uint8_t)phy->rachuserid;
	lchan_calc_crc(chan);

	// encode channel
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
	uint8_t* enc_b = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], blocksize/8, chan->data, enc_b);

	// repack bytes so that each array entry can be mapped to one symbol
	int num_repacked = enc_len*8/modem_get_bps(common->mcs_modem[mcs]);
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(enc_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);


	// modulate signal
	uint written_samps = 0;
	float complex subcarriers[NFFT];
	for (int i=0; i<=NFFT; i++) {
		if (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA) {
			modem_modulate(common->mcs_modem[mcs],(uint)repacked_b[written_samps++], &subcarriers[i]);
		}
	}
	// write symbol in time domain buffer
	ofdmframegen_writesymbol(phy->fg,subcarriers,txbuf_time);

	lchan_destroy(chan);
	free(enc_b);
	free(repacked_b);
}

// reset the ofdm symbol allocation
// should be done at the beginning of a subframe/scheduling period
// this is required to stop the devices from sending old garbage values
void phy_ue_reset_symbol_allocation(PhyUE phy, uint subframe)
{
	memset(phy->ul_symbol_alloc[subframe],NOT_USED,SUBFRAME_LEN);
}

// Create one OFDM symbol in time domain
// Subcarriers in frequency have to be set beforehand!
void phy_ue_write_symbol(PhyUE phy, float complex* txbuf_time)
{
	PhyCommon common = phy->common;
	uint tx_symb = common->tx_symbol;
	uint sfn = common->tx_subframe %2;
	// ensure that DL has achieved sync
	if (common->tx_active) {
		// check for MAC layer sync state
		if(!mac_ue_is_associated(phy->mac)) {
			// Not associated yet. Use Random Access slot to get association
			if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN) {
				ofdmframegen_reset(phy->fg);
				ofdmframegen_write_S0a(phy->fg, txbuf_time);
			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+1) {
				ofdmframegen_write_S0b(phy->fg, txbuf_time);
			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+2) {
				ofdmframegen_write_S1(phy->fg, txbuf_time);
			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+3) {
				phy_ue_create_assoc_request(phy, txbuf_time);
			} else {
				// send zeros
				memset(txbuf_time, 0, sizeof(float complex)*(NFFT+CP_LEN));
			}
		} else if (phy->ul_symbol_alloc[sfn][tx_symb]==DATA){
			// MAC is associated and have data to send.
			if (common->pilot_symbols_tx[tx_symb] == PILOT) {
				ofdmframegen_reset(phy->fg); // TODO we use the same msequence in every pilot symbol sent. Fix this
				ofdmframegen_writesymbol(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
			} else {
				ofdmframegen_writesymbol_nopilot(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
			}
		} else {
			// associated but no data to send. Set zero
			memset(txbuf_time,0,sizeof(float complex)*(NFFT+CP_LEN));
		}
	}

	// Update subframe and symbol counter
	common->tx_symbol++;
	if (common->tx_symbol>=SUBFRAME_LEN) {
		common->tx_symbol = 0;
		common->tx_subframe = (common->tx_subframe+1) % FRAME_LEN;
	}
}

// Main PHY receive function
//receive an arbitrary number of samples and process slots once they are received
void phy_ue_do_rx(PhyUE phy, float complex* rxbuf_time, uint num_samples)
{
	uint remaining_samps = num_samples;
	PhyCommon common = phy->common;

	while (remaining_samps > 0) {
		// find sync sequence
		if (!ofdmframesync_is_synced(phy->fs)) {
			int offset = phy_ue_initial_sync(phy,rxbuf_time,remaining_samps);
			if (offset!=-1) {
				remaining_samps -= offset;
				rxbuf_time += offset;
				phy->rx_offset =  num_samples - remaining_samps;
			} else {
				remaining_samps = 0;
			}
		} else {
			// receive symbols
			uint rx_sym = fmin(NFFT+CP_LEN,remaining_samps);
			if (common->pilot_symbols_rx[common->rx_symbol] == PILOT) {
				ofdmframesync_execute(phy->fs,rxbuf_time,rx_sym);
				LOG(DEBUG,"[PHY UE] cfo updated: %.3f Hz\n",ofdmframesync_get_cfo(phy->fs)*SAMPLERATE/6.28);
			} else {
				ofdmframesync_execute_nopilot(phy->fs,rxbuf_time,rx_sym);
			}
			remaining_samps -= rx_sym;
			rxbuf_time += rx_sym;
		}
	}
}

// create phy ctrl slot
int phy_map_ulctrl(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr)
{
	PhyCommon common = phy->common;

	uint8_t* repacked_b;
	uint bytes_written=0;

	uint mcs=0;
	// fixed MCS 0: r=1/2, bps=2, 16tail bits.
	uint32_t blocksize = get_ulctrl_slot_size(phy->common);

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
	uint first_symb = 2*(SLOT_LEN+1) + 2*slot_nr;	// slot 0 is mapped to symbol 30, slot 1 is mapped to symb 32.

	// modulate signal
	uint sfn = subframe % 2;
	phy_mod(phy->common,sfn,0,NFFT-1,first_symb,first_symb, mcs, repacked_b, num_repacked, &total_samps);

	// activate used OFDM symbols in resource allocation
	phy->ul_symbol_alloc[sfn][first_symb] = DATA;

	free(enc_b);
	free(repacked_b);
	return 0;
}

// create phy data slot in frequency domain
int phy_map_ulslot(PhyUE phy, LogicalChannel chan, uint subframe, uint8_t slot_nr, uint mcs)
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
	uint first_symb = (SLOT_LEN+1)*slot_nr;
	uint last_symb = (SLOT_LEN+1)*(slot_nr+1)-2; //TODO implement generic function to calc all slot allocations
	// slot 3 and 4 are shifted back since the ULCTRL lies between slot 2 and 3
	if(slot_nr>=2) {
		first_symb += 4;
		last_symb +=4;
	}

	// modulate signal
	uint sfn = subframe % 2;
	phy_mod(phy->common,sfn, 0,NFFT-1,first_symb,last_symb, mcs, repacked_b, num_repacked, &total_samps);

	// activate used OFDM symbols in resource allocation
	memset(&phy->ul_symbol_alloc[sfn][first_symb],DATA,last_symb-first_symb+1);

	free(interleaved_b);
	free(enc_b);
	free(repacked_b);
	return 0;
}

// Main Thread for UE receive
void thread_phy_ue_rx(PhyUE phy, platform hw, pthread_cond_t* sched_sync)
{
	float complex* rxbuf_time = calloc(sizeof(float complex),NFFT+CP_LEN);

	// get initial sync
	int offset = -1;
	while (offset == -1) {
		hw->platform_rx(hw, rxbuf_time);
		offset = phy_ue_initial_sync(phy, rxbuf_time, NFFT+CP_LEN);
	}
	LOG(INFO,"[PHY UE] rx thread got sync!\n");
	// start the tx thread
	//pthread_cond_signal(&phy->tx_sync_cond);

	// Main RX loop
	while (1) {
		// fill buffer
		hw->platform_rx(hw, rxbuf_time);
		// process samples
		phy_ue_do_rx(phy, rxbuf_time, NFFT+CP_LEN);

		// Run scheduler after DLCTRL slot was received
		if (phy->common->rx_symbol == DLCTRL_LEN) {
			pthread_cond_signal(sched_sync);
		}
	}
}

// Main Thread for UE transmit
void thread_phy_ue_tx(PhyUE phy, platform hw)
{
	float complex* txbuf_time = calloc(sizeof(float complex),NFFT+CP_LEN);
	uint offset, num_samples;

	// wait until rx thread has achieved sync
	//pthread_mutex_lock(&phy->tx_sync_lock);
	//pthread_cond_wait(&phy->tx_sync_cond, &phy->tx_sync_lock);

	while (!phy->has_synced_once) {
		hw->platform_tx_push(hw);
		hw->platform_tx_prep(hw, txbuf_time, 0, NFFT+CP_LEN);
	}

	LOG(INFO,"[PHY UE] main tx thread started!\n");
	offset = phy->rx_offset;
	num_samples = NFFT+CP_LEN-offset;

	while (1) {
		// push buffer
		hw->platform_tx_push(hw);

		// create tx time data
		// first add the last samples from the previous generated symbol
		hw->platform_tx_prep(hw, txbuf_time, 0, offset);
		// create new symbol
		phy_ue_write_symbol(phy, txbuf_time);

		// prepare first part of the new symbol
		hw->platform_tx_prep(hw, txbuf_time, offset, num_samples);

		// update offset
		if (phy->common->tx_symbol == 0 && phy->common->tx_subframe == 0) {
			offset = phy->rx_offset;
			num_samples = NFFT+CP_LEN-offset;
		}
	}
	hw->end(hw);
}
