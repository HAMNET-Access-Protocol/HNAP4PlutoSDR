#include "phy_bs.h"



// callback for OFDM RACH receiver object
// is called for every symbol that is received
int _ofdm_rx_rach_cb(float complex* X,unsigned char* p, uint M, void* rach_buffer) {
	memcpy((uint8_t*)rach_buffer,X,sizeof(float complex)*NFFT);
	return 0;
}

PhyBS phy_init_bs()
{
	PhyBS phy = malloc(sizeof(PhyBS_s));

	phy->common = phy_init_common();

	// Create OFDM frame generator: nFFt, CPlen, taperlen, subcarrier alloc
	phy->fg = ofdmframegen_create(NFFT, CP_LEN, 0, phy->common->pilot_sc);

	// Create OFDM receiver
	for (int i=0; i<MAX_USER; i++) {
		phy->fs[i] = NULL;
	}
	phy->fs_rach = NULL;;

    // alloc buffer for dl control slot
    phy->dlctrl_buf = calloc((NFFT-NUM_GUARD)*DLCTRL_LEN/8, 1);

    phy->rach_buffer = calloc((NFFT-NUM_GUARD)/8, 1);

    return phy;
}

// generate two ofdm symbols for sync
void phy_make_syncsig(PhyBS phy, float complex* txbuf_time)
{
	ofdmframegen_reset(phy->fg);
    ofdmframegen_write_S0a(phy->fg, txbuf_time);
    txbuf_time += NFFT+CP_LEN;
    ofdmframegen_write_S0b(phy->fg, txbuf_time);
    txbuf_time += NFFT+CP_LEN;
    ofdmframegen_write_S1(phy->fg, txbuf_time);
}

// Write one slot into time domain buffer
// do fft of subcarrier allocation
void phy_write_subframe(PhyBS phy, float complex* txbuf_time)
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

}


// create phy data channel in frequency domain
int phy_map_dlslot(PhyBS phy, LogicalChannel chan, uint8_t slot_nr, uint mcs)
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
	// allocate additional 8 bytes, because sometimes the encoded message len
	// is a bit to short. TODO: fix this
	int num_repacked = enc_len*8/modem_get_bps(common->mcs_modem[mcs])+8;
	repacked_b = malloc(num_repacked);
	liquid_repack_bytes(enc_b,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	uint first_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*slot_nr;
	uint last_symb = DLCTRL_LEN+2+(SLOT_LEN+1)*(slot_nr+1)-2;

	// modulate signal
	phy_mod(phy->common,0,NFFT-1,first_symb,last_symb, mcs, repacked_b, num_repacked, &total_samps);

	// set dlctrl slot
	if (slot_nr % 2 == 0) {
		phy->dlctrl_buf[slot_nr/2].h4 = chan->userid;
	} else {
		phy->dlctrl_buf[slot_nr/2].l4 = chan->userid;
	}

	free(interleaved_b);
	free(enc_b);
	free(repacked_b);
	return 0;
}

void phy_map_dlctrl(PhyBS phy)
{
	PhyCommon common = phy->common;

	// use MCS0 for modulation
	uint mcs = 0;

	uint buf_size = (2*NUM_SLOT+NUM_ULCTRL_SLOT)/2;

	// add CRC
	phy->dlctrl_buf[buf_size].byte = crc_generate_key(LIQUID_CRC_8, (uint8_t*)phy->dlctrl_buf,buf_size);

	// encode data
	uint enc_len = fec_get_enc_msg_length(common->mcs_fec_scheme[mcs],buf_size+1);
	uint8_t* buf_enc = malloc(enc_len);
	fec_encode(common->mcs_fec[mcs], buf_size+1,(uint8_t*)phy->dlctrl_buf, buf_enc);

	// repack bytes and modulate them
	uint bytes_written;
	int num_repacked = enc_len*8/modem_get_bps(common->mcs_modem[mcs]);
	uint8_t* repacked_b = calloc(1,(NFFT-NUM_GUARD)*DLCTRL_LEN); //we alloc more space than actually needed
	liquid_repack_bytes((uint8_t*)buf_enc,8,enc_len,repacked_b,modem_get_bps(common->mcs_modem[mcs]),num_repacked,&bytes_written);

	uint total_samps = 0;
	phy_mod(common, 0, NFFT-1, 0, DLCTRL_LEN-1, mcs, repacked_b, num_repacked, &total_samps);

	free(buf_enc);
	free(repacked_b);
}

int phy_assign_dlctrl_ud(PhyBS phy, uint8_t* assigned_slots, uint userid)
{
	for (int i=0; i<NUM_SLOT; i+=2) {
		if (assigned_slots[i] == 1) {
	        phy->dlctrl_buf[NUM_SLOT/2+i/2].h4 = userid;
		}
		if (assigned_slots[i+1] == 1) {
	        phy->dlctrl_buf[NUM_SLOT/2+i/2].l4 = userid;
		}
	}
	return 0;
}

int phy_assign_caich_uc(PhyBS phy, uint8_t* assigned_slots, uint userid)
{
	for (int i=0; i<NUM_ULCTRL_SLOT; i+=2) {
		if (assigned_slots[i] == 1) {
	        phy->dlctrl_buf[NUM_SLOT+i/2].h4 = userid;
		}
		if (assigned_slots[i+1] == 1) {
	        phy->dlctrl_buf[NUM_SLOT+i/2].l4 = userid;
		}
	}
	return 0;
}


int phy_bs_rx_subframe(PhyBS phy, float complex* rxbuf_time)
{
	PhyCommon common = phy->common;

	common->rx_symbol = 0;

	uint sf_len = SUBFRAME_LEN;
	if (common->subframe == FRAME_LEN-1) {
		sf_len -= SYNC_SYMBOLS;
	}

	for (int i=0; i<sf_len; i++) {
		if (common->pilot_symbols[i] == PILOT) {
			ofdmframesync_execute(phy->fs, rxbuf_time,(NFFT+CP_LEN));
		} else {
			ofdmframesync_execute_nopilot(phy->fs, rxbuf_time, NFFT+CP_LEN);
		}
		rxbuf_time += NFFT+CP_LEN;
	}

	if (common->rx_symbol != sf_len)
		printf("[PHY] Error: did not receive all ofdm symbols!\n");

	return 0;
}

int phy_bs_rx_rach(PhyBS phy, float complex* rxbuf_time)
{
	// create new sync context and try to receive sync sequence
	phy->fs_rach = ofdmframesync_create(NFFT,CP_LEN,0,phy->common->pilot_sc,_ofdm_rx_rach_cb, phy->rach_buffer);
	uint offset = ofdmframesync_find_data_start(phy->fs_rach, rxbuf_time,(NFFT+CP_LEN)*(SLOT_LEN)); //TODO correct length
	if (offset == -1) {
		// no transmission detected
		ofdmframesync_destroy(phy->fs_rach);
		return 0;
	}

	// get timing advance
	uint diff = offset - (NFFT+CP_LEN)*SYNC_SYMBOLS;

	// receive control message
	ofdmframesync_execute(phy->fs_rach, rxbuf_time, NFFT+CP_LEN);

	// rx_rach callback has filled phy->rach_buffer. Decode here

	uint8_t rach_userid = phy->rach_buffer[0];

	mac_bs_add_new_ue(rach_userid, diff); //TODO correct mac call
	return 1;
}
