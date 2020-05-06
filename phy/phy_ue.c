/*
 * phy_ue.c
 *
 *  Created on: Nov 12, 2019
 *      Author: lukas
 */

#include "phy_ue.h"

#include "../util/log.h"
#include "../mac/mac_ue.h"
#include <pthread.h>
#include "../platform/pluto.h"

#ifdef PHY_TEST_BER
#include "../runtime/test.h"
#endif

// phy configuration
extern struct phy_config_s PHY_CONFIG;

// Declarations of local functions
int _ue_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd);

// Init the PhyUE struct
PhyUE phy_ue_init()
{
	PhyUE phy = calloc(sizeof(struct PhyUE_s),1);

	phy->common = phy_common_init();
#if USE_ROBUST_PILOT
	gen_pilot_symbols_robust(phy->common, 0);
#else
	gen_pilot_symbols(phy->common, 0);
#endif

	// Create OFDM frame generator: nFFt, CPlen, taperlen, subcarrier alloc
	phy->fg = ofdmframegen_create(nfft, cp_len, 0, phy->common->pilot_sc);

	// Create OFDM receiver
	phy->fs = ofdmframesync_create(nfft, cp_len, 0, phy->common->pilot_sc, _ue_rx_symbol_cb, phy);

	// Alloc memory for slot assignments
	phy->dlslot_assignments = malloc(2*sizeof(assignment_t*));
	phy->ulslot_assignments = malloc(2*sizeof(assignment_t*));
	phy->ulctrl_assignments = malloc(2*sizeof(assignment_t*));

	for (int i=0; i<2; i++) {
		phy->dlslot_assignments[i] = calloc(sizeof(assignment_t),NUM_SLOT);
		phy->ulslot_assignments[i] = calloc(sizeof(assignment_t),NUM_SLOT);
		phy->ulctrl_assignments[i] = calloc(sizeof(assignment_t),NUM_ULCTRL_SLOT);
	}

    // buffer for ofdm symbol allocation
    phy->ul_symbol_alloc = malloc(sizeof(uint8_t*)*2);
	phy->ul_symbol_alloc[0] = calloc(sizeof(uint8_t)*SUBFRAME_LEN,1);
    phy->ul_symbol_alloc[1] = calloc(sizeof(uint8_t)*SUBFRAME_LEN,1);

	phy->mac_rx_cb = NULL;
	phy->has_synced_once = 0;

	phy->mcs_dl = 0;

	phy->prev_cfo = 0.0;

	phy->rx_offset = 0;

	phy->rachuserid = -1;
	phy->rach_try_cnt = 0;
	phy->userid = -1;

	// receiving a slot (demod, fec decode, interleaver) will be handled in a
	// separate thread
	phy->rx_slot_signal = NULL;
	phy->rx_slot_nr = 0;

	phy->bs_txgain = -128;
	phy->bs_rxgain = -128;
	phy->rssi = AGC_DESIRED_RSSI;
    return phy;
}

void phy_ue_destroy(PhyUE phy)
{
	phy_common_destroy(phy->common);
	ofdmframegen_destroy(phy->fg);
	ofdmframesync_destroy(phy->fs);

	for (int i=0; i<2; i++) {
		free(phy->dlslot_assignments[i]);
		free(phy->ulslot_assignments[i]);
		free(phy->ulctrl_assignments[i]);
		free(phy->ul_symbol_alloc[i]);
	}
	free(phy->dlslot_assignments);
	free(phy->ulslot_assignments);
	free(phy->ulctrl_assignments);
	free(phy->ul_symbol_alloc);

	free(phy);
}

// Sets the callback function that is called after a phy slot was received
// and the Mac UE instance
void phy_ue_set_mac_interface(PhyUE phy, void (*mac_rx_cb)(struct MacUE_s*, LogicalChannel, uint), struct MacUE_s* mac)
{
	phy->mac_rx_cb = mac_rx_cb;
	phy->mac = mac;
}

// Set the pointer to the platform struct.
// Phy will use this to indicate PTT
void phy_ue_set_platform_interface(PhyUE phy, struct platform_s* platform)
{
    phy->platform = platform;
}

// Setter function for Downlink MCS
int phy_ue_set_mcs_dl(PhyUE phy, uint mcs)
{
	if (mcs<NUM_MCS_SCHEMES) {
		phy->mcs_dl = mcs;
		return 1;
	}
	return 0;
}

// Set the condition which is used to signal the slot processing thread
void phy_ue_set_rx_slot_th_signal(PhyUE phy, pthread_cond_t* cond)
{
	phy->rx_slot_signal = cond;
}

// Searches for the initial sync sequence
// returns -1 if no sync found, else the sample index of the ofdm symbol after the sync sequence
int phy_ue_initial_sync(PhyUE phy, float complex* rxbuf_time, uint num_samples)
{
	PhyCommon common = phy->common;

	int offset = ofdmframesync_find_data_start(phy->fs,rxbuf_time,num_samples);
	if (offset!=-1) {
		common->rx_symbol = SUBFRAME_LEN - 1 - 1; //there is the sync info symbol and one guard symbol remaining in the subframe
		common->rx_subframe = 0;

		//apply filtering of coarse CFO estimation if we have old estimates
		if (phy->has_synced_once == 0) {
			// init TX counters once
			common->tx_active = 1;
			common->tx_symbol = common->rx_symbol + 1 - DL_UL_SHIFT + DL_UL_SHIFT_COMP_UE; //TODO clarify what happens for offset=0
			common->tx_subframe = 0;

			phy->has_synced_once = 1;
			phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
			LOG(INFO,"[PHY UE] Got sync! cfo: %.3fHz offset: %d samps\n",phy->prev_cfo*samplerate/6.28,offset);
			SYSLOG(LOG_INFO,"[PHY UE] Got sync! cfo: %.3fHz offset: %d samps\n",phy->prev_cfo*samplerate/6.28,offset);
		} else {
			float new_cfo = ofdmframesync_get_cfo(phy->fs);
            if (log_coarse_cfo_flag)
                log_bin((uint8_t*)&new_cfo,sizeof(float),coarse_cfo_logfile,"a");

			float cfo_filt = (1-coarse_cfo_filt_param)*phy->prev_cfo + coarse_cfo_filt_param*new_cfo;
			ofdmframesync_set_cfo(phy->fs,cfo_filt);
			LOG_SFN_PHY(DEBUG,"[PHY UE] sync seq. cfo: %.3fHz offset: %d samps\n",new_cfo*samplerate/6.28,offset);

		}
	}
	return offset;
}

// Process the Symbols received in a Downlink Control Slot
// returns 1 if DL CTRL slot was successfully decoded, else 0
// furthermore sets the phy dl/ul_assignments variable accordingly
int phy_ue_proc_dlctrl(PhyUE phy)
{
    PhyCommon common = phy->common;
	uint dlctrl_size = (NUM_SLOT*2 + NUM_ULCTRL_SLOT)/2;
	uint sfn = common->rx_subframe % 2; // even or uneven subframe?

	// demodulate signal.
	uint llr_len = 2*DLCTRL_LEN*(num_data_sc+num_pilot_sc);
	uint8_t* llr_buf = malloc(llr_len);
	uint total_samps = 0;
	phy_demod_soft(common, 0, nfft-1, 0, DLCTRL_LEN-1, 0, llr_buf, llr_len, &total_samps);

	// soft decoding
	dlctrl_alloc_t* dlctrl_buf = malloc(dlctrl_size+1);
	fec_decode_soft(common->fec_ctrl,dlctrl_size+1, llr_buf, (uint8_t*)dlctrl_buf);

	//unscrambling
	unscramble_data((uint8_t*)dlctrl_buf,dlctrl_size+1);
	// verify CRC
	if (!crc_validate_message(LIQUID_CRC_8, (uint8_t*)dlctrl_buf, dlctrl_size, dlctrl_buf[dlctrl_size].byte)) {
		LOG(WARN,"[PHY UE] DLCTRL slot could not be decoded! ");
		for (int i=0; i<dlctrl_size+1; i++)
			LOG(WARN,"%02x",dlctrl_buf[i].byte);
		LOG(WARN,"\n");
		// set dlctrl buf to 0 so that mac will think no slots allocated
		memset(dlctrl_buf,0,dlctrl_size);
	}

	// Set the decoded user assignments in the phy struct
	LOG_SFN_PHY(DEBUG,"[PHY UE] DLCTRL:");
	uint idx = 0;
	for (int i=0; i<NUM_SLOT/2; i++) {
		LOG(DEBUG,"%02x",dlctrl_buf[idx].byte);
		phy->dlslot_assignments[sfn][2*i  ] = dlctrl_buf[idx].h4 == phy->userid ? UE_ASSIGNED : NOT_ASSIGNED;
		phy->dlslot_assignments[sfn][2*i  ] = dlctrl_buf[idx].h4 == USER_BROADCAST ? BRCST_ASSIGNED : phy->dlslot_assignments[sfn][2*i];
		phy->dlslot_assignments[sfn][2*i+1] = dlctrl_buf[idx].l4 == phy->userid ? UE_ASSIGNED : NOT_ASSIGNED;
		phy->dlslot_assignments[sfn][2*i+1] = dlctrl_buf[idx].l4 == USER_BROADCAST ? BRCST_ASSIGNED : phy->dlslot_assignments[sfn][2*i+1];
		idx++;
	}
	for (int i=0; i<NUM_SLOT/2; i++) {
		LOG(DEBUG,"%02x",dlctrl_buf[idx].byte);
		phy->ulslot_assignments[sfn][2*i  ] = (dlctrl_buf[idx].h4 == phy->userid) ? UE_ASSIGNED : NOT_ASSIGNED;
		phy->ulslot_assignments[sfn][2*i+1] = (dlctrl_buf[idx].l4 == phy->userid) ? UE_ASSIGNED : NOT_ASSIGNED;
		idx++;
	}
	for (int i=0; i<NUM_ULCTRL_SLOT/2; i++) {
		LOG(DEBUG,"%02x\n",dlctrl_buf[idx].byte);
		phy->ulctrl_assignments[sfn][2*i  ] = (dlctrl_buf[idx].h4 == phy->userid) ? UE_ASSIGNED : NOT_ASSIGNED;
		phy->ulctrl_assignments[sfn][2*i+1] = (dlctrl_buf[idx].l4 == phy->userid) ? UE_ASSIGNED : NOT_ASSIGNED;
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

TIMECHECK_CREATE(timecheck_ue_rx);
TIMECHECK_CREATE(check_demod);
TIMECHECK_CREATE(check_fec);
TIMECHECK_CREATE(check_interl);
// Decode a PHY dl slot and call the MAC callback function
void phy_ue_proc_slot(PhyUE phy, uint slotnr)
{
    TIMECHECK_INIT(check_demod,"ue.rx_slot.demod",10000);
    TIMECHECK_INIT(check_fec,"ue.rx_slot.fec",10000);
    TIMECHECK_INIT(check_interl,"ue.rx_slot.interleaver",10000);
    TIMECHECK_INIT(timecheck_ue_rx,"ue.rx_slot",10000);

	PhyCommon common = phy->common;
	assignment_t slot_type = phy->dlslot_assignments[common->rx_subframe%2][slotnr];
	if (slot_type != NOT_ASSIGNED) {
        TIMECHECK_START(timecheck_ue_rx);

        // MCS0 is used for Broadcast. For UE specific traffic use the set mcs
		uint mcs = (slot_type == UE_ASSIGNED) ? phy->mcs_dl : 0;
		uint32_t blocksize = get_tbs_size(common, mcs);

		uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
		uint8_t* demod_buf = malloc(buf_len);

		// demodulate signal
		uint written_samps = 0;
		uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*slotnr;
		uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(slotnr+1)-2;
		TIMECHECK_START(check_demod);
		phy_demod_soft(common, 0, nfft-1, first_symb, last_symb, mcs,
					   demod_buf, buf_len, &written_samps);
        TIMECHECK_STOP(check_demod);
		//deinterleaving
		uint8_t* deinterleaved_b = malloc(buf_len);
        TIMECHECK_START(check_interl);
		interleaver_decode_soft(common->mcs_interlvr[mcs],demod_buf,deinterleaved_b);
        TIMECHECK_STOP(check_interl);
        TIMECHECK_START(check_fec);
		// decoding
		LogicalChannel chan = lchan_create(blocksize/8,CRC16);
		fec_decode_soft(common->mcs_fec[mcs], blocksize/8, deinterleaved_b, chan->data);
        TIMECHECK_STOP(check_fec);

#ifdef PHY_TEST_BER
	uint32_t num_biterr = 0;
	// we start calculating ber after subframe 50 to wait that MCS switch happened
	if (global_sfn>50) {
		for (int i=0; i<chan->payload_len;i++)
			num_biterr += liquid_count_ones(phy_dl[common->rx_subframe%2][slotnr][i]^chan->data[i]);
		phy_dl_tot_bits += chan->payload_len*8;
		phy_dl_biterr += num_biterr;
	}
#endif

		// pass to upper layer
        phy->mac_rx_cb(phy->mac, chan, (slot_type==BRCST_ASSIGNED) ? 1:0);

		free(deinterleaved_b);
		free(demod_buf);

        TIMECHECK_STOP_CHECK(timecheck_ue_rx,3500);
        TIMECHECK_INFO(timecheck_ue_rx);
        TIMECHECK_INFO(check_demod);
        TIMECHECK_INFO(check_fec);
        TIMECHECK_INFO(check_interl);
	}
}

// Process the synchronization information
int phy_ue_proc_sync_info(PhyUE phy)
{
    PhyCommon common = phy->common;

    uint mcs = 0; // CTRL slots use MCS 0
    uint32_t blocksize = get_ulctrl_slot_size(common);

    uint buf_len = 8*fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],blocksize/8);
    uint8_t* demod_buf = malloc(buf_len);

    // demodulate signal
    const int symb_idx = SUBFRAME_LEN -2;
    uint written_samps = 0, symbol = 0;
    for (int i=0; i<nfft; i++) {
        if (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA && written_samps<buf_len) {
            modem_demodulate_soft(common->mcs_modem[mcs], common->rxdata_f[symb_idx][i], &symbol, &demod_buf[written_samps]);
            written_samps += modem_get_bps(common->mcs_modem[mcs]);
        }
    }
    // decoding
    LogicalChannel chan = lchan_create(blocksize/8,CRC8);
    fec_decode_soft(common->fec_ctrl, blocksize/8, demod_buf, chan->data);

    // unscrambling
    unscramble_data((uint8_t*)chan->data,chan->payload_len);

    if (lchan_verify_crc(chan)) {
        phy->bs_rxgain = chan->data[0];
        phy->bs_txgain = chan->data[1];
        LOG(DEBUG,"[PHY UE] BS sync info: bs_rxgain: %d bs_txgain: %d\n",phy->bs_rxgain,phy->bs_txgain);
    } else {
        LOG(WARN,"[PHY UE] cannot decode SYNC INFO slot!\n");
    }

    free(demod_buf);
    lchan_destroy(chan);
    return 0;
}

// callback for OFDM receiver
// is called for every symbol that is received
int _ue_rx_symbol_cb(float complex* X,unsigned char* p, uint M, void* userd)
{
	PhyUE phy = (PhyUE)userd;
	PhyCommon common = phy->common;

	memcpy(common->rxdata_f[common->rx_symbol++],X,sizeof(float complex)*nfft);

	switch (common->rx_symbol) {
	case DLCTRL_LEN:
		// finished receiving DLCTRL slot
		phy_ue_proc_dlctrl(phy);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1):
		// finished receiving one of the dl data slots
#ifdef USE_RX_SLOT_THREAD
		phy->rx_slot_nr = 0;
        if (phy->rx_slot_signal)
            pthread_cond_signal(phy->rx_slot_signal);
        else
#endif
            phy_ue_proc_slot(phy,0);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*2:
		// finished receiving one of the dl data slots
#ifdef USE_RX_SLOT_THREAD
		phy->rx_slot_nr = 1;
        if (phy->rx_slot_signal)
            pthread_cond_signal(phy->rx_slot_signal);
        else
#endif
            phy_ue_proc_slot(phy,1);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*3:
		// finished receiving one of the dl data slots
#ifdef USE_RX_SLOT_THREAD
		phy->rx_slot_nr = 2;
        if (phy->rx_slot_signal)
            pthread_cond_signal(phy->rx_slot_signal);
        else
#endif
            phy_ue_proc_slot(phy,2);
		break;
	case DLCTRL_LEN+1+(SLOT_LEN+1)*4:
		// finished receiving one of the dl data slots
		// if subframe==0, it is the sync info slot
		if (common->rx_subframe==0) {
            phy_ue_proc_sync_info(phy);
		} else {
#ifdef USE_RX_SLOT_THREAD
            phy->rx_slot_nr = 3;
            if (phy->rx_slot_signal)
                pthread_cond_signal(phy->rx_slot_signal);
            else
#endif
                phy_ue_proc_slot(phy, 3);
        }
		break;
	default:
		break;
	}

	// log cfo estimate
	if (log_cfo_flag) {
        float cfo = ofdmframesync_get_cfo(phy->fs);
	    log_bin((uint8_t*)&cfo, sizeof(float),cfo_logfile,"a");
    }
	// sync sequence will follow. Reset framesync and adjust gain
	if ((common->rx_subframe == 0) &&
			(common->rx_symbol == DLCTRL_LEN+1+(SLOT_LEN+1)*3)) {
        // set new gain value
        phy->rssi = (1-AGC_RSSI_FILT_PARAM)*phy->rssi + AGC_RSSI_FILT_PARAM*ofdmframesync_get_rssi(phy->fs);
        // store old cfo estimation
		phy->prev_cfo = ofdmframesync_get_cfo(phy->fs);
		ofdmframesync_reset(phy->fs);
        ofdmframesync_set_cfo(phy->fs,0);
	}

	// Debug log
	/*char name[30];
	sprintf(name,"rxF/rxF_%d_%d.m",common->rx_subframe,common->rx_symbol-1);
	if (common->rx_symbol<64) {
	LOG_MATLAB_FC(DEBUG,X, NFFT, name);
	}*/
	if (common->rx_symbol >= SUBFRAME_LEN) {
		common->rx_symbol = 0;
		common->rx_subframe = (common->rx_subframe + 1) % FRAME_LEN;
	}

	return 0;
}

// Create the symbol containing association request data
// TODO implement backoff algorithm. If two users try to assoc at the same time
//      they currently interfere with each other in every RA slot
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
	if (phy->rach_try_cnt == 0) {
		// RA procedure hasnt started. Select a random ID first
		phy->rachuserid = rand() % MAX_USER;
	}
	chan->data[0] = (uint8_t)phy->rachuserid;
	chan->data[1] = (uint8_t)phy->rach_try_cnt++;
	chan->writepos = 2;
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
	float complex subcarriers[nfft];
	for (int i=0; i<nfft; i++) {
		if (common->pilot_sc[i] == OFDMFRAME_SCTYPE_DATA && written_samps<num_repacked) {
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
            if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN-1) {
                // set ptt signal one symbol before tx starts
                phy->platform->ptt_set_tx(phy->platform);
            } else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN) {
				ofdmframegen_reset(phy->fg);
				ofdmframegen_write_S0a(phy->fg, txbuf_time);

			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+1) {
				ofdmframegen_write_S0b(phy->fg, txbuf_time);
			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+2) {
				ofdmframegen_write_S1(phy->fg, txbuf_time);
			} else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+3) {
				phy_ue_create_assoc_request(phy, txbuf_time);
            } else if (common->tx_subframe == 0 && tx_symb == SUBFRAME_LEN-SLOT_LEN+4) {
                phy->platform->ptt_set_rx(phy->platform);
            } else {
				// send zeros
				memset(txbuf_time, 0, sizeof(float complex)*(nfft+cp_len));
			}
		} else if (phy->ul_symbol_alloc[sfn][tx_symb]==DATA){
			// MAC is associated and have data to send.
			if (common->pilot_symbols_tx[tx_symb] == PILOT) {
				ofdmframegen_reset(phy->fg); // TODO we use the same msequence in every pilot symbol sent. Fix this
				ofdmframegen_writesymbol(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
			} else {
				ofdmframegen_writesymbol_nopilot(phy->fg, common->txdata_f[sfn][tx_symb],txbuf_time);
			}
		} else if (phy->ul_symbol_alloc[sfn][tx_symb] == PTT_UP) {
            memset(txbuf_time,0,sizeof(float complex)*(nfft+cp_len));
            phy->platform->ptt_set_tx(phy->platform);
        } else if (phy->ul_symbol_alloc[sfn][tx_symb] == PTT_DOWN) {
            memset(txbuf_time,0,sizeof(float complex)*(nfft+cp_len));
            phy->platform->ptt_set_rx(phy->platform);
		} else {
			// associated but no data to send. Set zero
			memset(txbuf_time,0,sizeof(float complex)*(nfft+cp_len));
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
			uint rx_sym = fmin(nfft+cp_len,remaining_samps);
			if (common->pilot_symbols_rx[common->rx_symbol] == PILOT ||
                    (common->rx_subframe==0 && common->rx_symbol==SUBFRAME_LEN-2)) {
				ofdmframesync_execute(phy->fs,rxbuf_time,rx_sym);
				LOG(TRACE,"[PHY UE] cfo updated: %.3f Hz\n",ofdmframesync_get_cfo(phy->fs)*samplerate/6.28);
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
	phy_mod(phy->common,sfn,0,nfft-1,first_symb,first_symb, mcs, repacked_b, num_repacked, &total_samps);

	// activate used OFDM symbols in resource allocation
	if (phy->ul_symbol_alloc[sfn][first_symb-2]==NOT_USED)
	    phy->ul_symbol_alloc[sfn][first_symb-1] = PTT_UP; // indicate PTT, slot before isnt used
	phy->ul_symbol_alloc[sfn][first_symb] = DATA;
	if (phy->ul_symbol_alloc[sfn][first_symb+2]==NOT_USED)
	    phy->ul_symbol_alloc[sfn][first_symb+1] = PTT_DOWN; // next slot is not used, end PTT here

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

#ifdef PHY_TEST_BER
	memcpy(phy_ul[subframe%2][slot_nr], chan->data, chan->payload_len);
#endif

	// encode channel
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],chan->payload_len);
	uint8_t* enc_b = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], blocksize/8, chan->data, enc_b);

	//interleaving
	uint8_t* interleaved_b = malloc(enc_len);
	interleaver_encode(common->mcs_interlvr[mcs],enc_b, interleaved_b);

	// repack bytes so that each array entry can be mapped to one symbol
	int num_repacked = ceil(enc_len*8.0/modem_get_bps(common->mcs_modem[mcs]));
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(interleaved_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

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
	phy_mod(phy->common,sfn, 0,nfft-1,first_symb,last_symb, mcs, repacked_b, num_repacked, &total_samps);

	// activate used OFDM symbols in resource allocation
	memset(&phy->ul_symbol_alloc[sfn][first_symb],DATA,last_symb-first_symb+1);
    // if the previous slot is not used, we have to set the PTT signal before this data slot
	if (slot_nr==0) {
	    if (phy->ul_symbol_alloc[(sfn-1)%2][SUBFRAME_LEN-2]==NOT_USED)
            phy->ul_symbol_alloc[(sfn-1)%2][SUBFRAME_LEN-1] = PTT_UP; // indicate PTT, slot before isnt used
        else
            phy->ul_symbol_alloc[(sfn-1)%2][SUBFRAME_LEN-1] = DATA; //previous slot already in use. concat them
	} else {
        if (phy->ul_symbol_alloc[sfn][first_symb-2]==NOT_USED)
            phy->ul_symbol_alloc[sfn][first_symb-1] = PTT_UP; // indicate PTT, slot before isnt used
	    else
            phy->ul_symbol_alloc[sfn][first_symb-1] = DATA; //previous slot already in use. concat them
	}
    // if the next slot is not used we have to turn off PTT signal after this slot
    if (slot_nr==NUM_SLOT-1) {
        // for the last slot within a subframe we simply assume that the next slot is unused.
        // if it is used, the property can be overwritten in the next subframe assignment
        phy->ul_symbol_alloc[sfn][last_symb + 1] = PTT_DOWN; // next slot is not used, end PTT here
    } else {
        if (phy->ul_symbol_alloc[sfn][last_symb + 2] == NOT_USED)
            phy->ul_symbol_alloc[sfn][last_symb + 1] = PTT_DOWN; // next slot is not used, end PTT here
    }
    free(interleaved_b);
	free(enc_b);
	free(repacked_b);
	return 0;
}
