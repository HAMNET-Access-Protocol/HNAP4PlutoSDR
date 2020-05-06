/*
 * mac_ue.c
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */

#include "mac_ue.h"
#include <unistd.h>
#include "../phy/phy_ue.h"

#ifdef MAC_TEST_DELAY
#include "../runtime/test.h"
#endif

// Allocate memory for MAC instance and init it
MacUE mac_ue_init()
{
	MacUE mac = calloc(sizeof(struct MacUE_s), 1);
	mac->msg_control_queue = ringbuf_create(MAC_CTRL_MSG_BUF_SIZE);
	mac->fragmenter = mac_frag_init();
	mac->reassembler = mac_assmbl_init();
    mac->reassembler_brcst = mac_assmbl_init();
#ifdef MAC_ENABLE_TAP_DEV
	mac->tapdevice = tap_init("tap0");
#endif
	return mac;
}

void mac_ue_destroy(MacUE mac)
{
	mac_frag_destroy(mac->fragmenter);
	mac_assmbl_destroy(mac->reassembler);
	while (!ringbuf_isempty(mac->msg_control_queue)) {
		MacMessage p = ringbuf_get(mac->msg_control_queue);
		mac_msg_destroy(p);
	}
	ringbuf_destroy(mac->msg_control_queue);
	free(mac);
}

// Mac layer needs a pointer to phy layer in order to call
// interface functions
void mac_ue_set_phy_interface(MacUE mac, struct PhyUE_s* phy)
{
	mac->phy = phy;
}

// Generic handler for received messages
int mac_ue_handle_message(MacUE mac, MacMessage msg, uint is_broadcast)
{
	MacDataFrame frame;
	MacMessage response;

	switch (msg->type) {
	case associate_response:
	    // make sure we are not synced yet and the rachuserid is correct
	    if (mac->is_associated || (msg->hdr.AssociateResponse.rachuserid != mac->phy->rachuserid))
	        break;

		// Assert correct version
		if (msg->hdr.AssociateResponse.protoVersion != PROTO_VERSION) {
			LOG(ERR,"[MAC UE] Wrong Protocol version! got %d expected %d",
					msg->hdr.AssociateResponse.protoVersion,PROTO_VERSION);
			break;
		}
		// Check whether we were successfully added
		if (msg->hdr.AssociateResponse.response == assoc_resp_success) {
			mac->is_associated = 1;
			mac->phy->rach_try_cnt = 0; // reset number of association tries
			mac->userid = msg->hdr.AssociateResponse.userid;
			mac->dl_mcs = 0;
			mac->ul_mcs = 0;
            mac->timing_advance = msg->hdr.AssociateResponse.timing_advance;
			phy_ue_set_mcs_dl(mac->phy,0);
			// init mac statistics
			mac_stats_init(&mac->stats);
			LOG_SFN_MAC(INFO,"[MAC UE] successfully associated! userid: %d\n",mac->userid);
			mac->phy->userid = mac->userid; // Notify phy about the userid. TODO define interface functions?
			phy_ue_proc_dlctrl(mac->phy);	// Decode CTRL slot again, since we now know our userid
		} else {
			LOG_SFN_MAC(INFO,"[MAC UE] NACK for assoc req: response is %d\n",msg->hdr.AssociateResponse.response);
		}
		break;
	case dl_mcs_info:
		if (msg->hdr.DLMCSInfo.mcs >= NUM_MCS_SCHEMES) {
			LOG(WARN,"[MAC UE] received erroneous dl mcs info. MCS %d\n",msg->hdr.DLMCSInfo.mcs);
			mac_msg_destroy(msg);
			return 0;
		}
		response = mac_msg_create_control_ack(msg->hdr.DLMCSInfo.ctrl_id);
		if(!ringbuf_put(mac->msg_control_queue,response)) {
			LOG(ERR,"[MAC UE] cannot ack dl_mcs_info msg!\n");
			mac_msg_destroy(response);
		} else {
			mac->dl_mcs = msg->hdr.DLMCSInfo.mcs;
			phy_ue_set_mcs_dl(mac->phy, mac->dl_mcs);
			LOG(INFO,"[MAC UE] switching to DL MCS %d\n",mac->dl_mcs);
		}
		break;
	case ul_mcs_info:
		if (msg->hdr.ULMCSInfo.mcs >= NUM_MCS_SCHEMES) {
			LOG(WARN,"[MAC UE] received erroneous ul mcs info. MCS %d\n",msg->hdr.ULMCSInfo.mcs);
			mac_msg_destroy(msg);
			return 0;
		}
		response = mac_msg_create_control_ack(msg->hdr.ULMCSInfo.ctrl_id);
		if(!ringbuf_put(mac->msg_control_queue,response)) {
			LOG(ERR,"[MAC UE] cannot ack ul_mcs_info msg!\n");
			mac_msg_destroy(response);
		} else {
			mac->ul_mcs = msg->hdr.ULMCSInfo.mcs;
			LOG(INFO,"[MAC UE] switching to UL MCS %d\n",mac->ul_mcs);
		}
		break;
	case timing_advance:
		mac->timing_advance = msg->hdr.TimingAdvance.timingAdvance;
		LOG(INFO,"[MAC UE] Updated TimingAdvance to: %d\n",mac->timing_advance);
		break;
	case session_end:
		// Basestation ended connection. Reset connection state
		mac->is_associated = 0;
		mac->timing_advance = 0;
		mac->phy->userid = -1;
		mac->userid = 0;
		break;
	case dl_data:
        if (is_broadcast)
            frame = mac_assmbl_reassemble(mac->reassembler_brcst, msg);
        else
            frame = mac_assmbl_reassemble(mac->reassembler, msg);
		if (frame != NULL) {
			mac->stats.bytes_rx += frame->size;
            LOG(INFO,"[MAC UE] received dataframe of %d bytes. brdcst: %d\n",frame->size, is_broadcast);
			//PRINT_BIN(INFO,frame->data,frame->size); LOG(INFO,"\n");

#ifdef MAC_ENABLE_TAP_DEV
			tap_send(mac->tapdevice,frame->data,frame->size);
#endif

#ifdef MAC_TEST_DELAY
			static uint sfn=0;
			memcpy(&sfn,frame->data,sizeof(uint));
			if (sfn<num_simulated_subframes)
				mac_dl_timestamps[sfn] += global_sfn*SUBFRAME_LEN+global_symbol;
#endif
			dataframe_destroy(frame);
		}
		break;
	default:
		LOG(WARN,"[MAC UE] unexpected MacMsg ID: %d\n",msg->type);
		mac_msg_destroy(msg);
		return 0;
	}
	mac_msg_destroy(msg);
	return 1;
}

// Set the channel assignments which were decoded in the DLCTRL slot
void mac_ue_set_assignments(MacUE mac, uint8_t* dlslot, uint8_t* ulslot, uint8_t* ulctrl)
{
	memcpy(mac->dl_data_assignments, dlslot, MAC_DLDATA_SLOTS);
	memcpy(mac->ul_data_assignments, ulslot, MAC_ULDATA_SLOTS);
	memcpy(mac->ul_ctrl_assignments, ulctrl, MAC_ULCTRL_SLOTS);
}

// UE scheduler. Is called once per subframe
// Will check the ctrl message and data message queues and try
// to map it to slots. Before running the scheduler, ensure that
// the slot assignment variables are up to date
void mac_ue_run_scheduler(MacUE mac)
{
	uint queuesize = mac_frag_get_buffersize(mac->fragmenter);
	uint slotsize = get_tbs_size(mac->phy->common,mac->ul_mcs)/8;
	int num_assigned = num_slot_assigned(mac->ul_data_assignments,MAC_ULDATA_SLOTS,UE_ASSIGNED);
	uint next_sfn = (mac->phy->common->tx_subframe+1) % 2; // subframe for which the scheduler is run
	// Log schedule
	LOG(TRACE,"[MAC UE] Scheduler user assignments:\n");
	LOG(TRACE,"         DL data slots: %4d %4d %4d %4d\n", mac->dl_data_assignments[0],
			mac->dl_data_assignments[1],mac->dl_data_assignments[2],mac->dl_data_assignments[3]);
	LOG(TRACE,"         UL data slots: %4d %4d %4d %4d\n", mac->ul_data_assignments[0],
			mac->ul_data_assignments[1],mac->ul_data_assignments[2],mac->ul_data_assignments[3]);
	LOG(TRACE,"         UL ctrl slots: %4d %4d\n", mac->ul_ctrl_assignments[0],mac->ul_ctrl_assignments[1]);

	// ensure association
	if (mac->is_associated == 0) {
		return;
	}

	// Check whether the user hasnt been assigned any slots for a long time
	// if so we assume that the client disconnected us
	if (mac->last_assignment + TMR_USER_INACTIVE < mac->subframe_cnt) {
		mac->is_associated = 0;
		mac->timing_advance = 0;
		LOG(WARN,"[MAC UE] assume lost connection to BS! End connection.\n");
	}

	// reset symbol allocation. Will be set during phy modulation
	phy_ue_reset_symbol_allocation(mac->phy, next_sfn%2);

    // iterate over slots and check if the client is assigned to it
	if (num_assigned>0) {
		mac->last_assignment = mac->subframe_cnt;
		for (int i=0; i<MAC_ULDATA_SLOTS; i++) {
			if (mac->ul_data_assignments[i] == 1) {
				num_assigned--;
				LogicalChannel chan = lchan_create(slotsize, CRC16);
				lchan_add_all_msgs(chan, mac->msg_control_queue);
				if (queuesize>0) {
					// client is assigned to slot and has data
                    if (num_assigned==0 && queuesize>lchan_unused_bytes(chan)) {
						// this is the last assigned slot within subframe
						// if we still have remaining data, add a ul_req to this slot
                        uint remaining_bytes = queuesize-lchan_unused_bytes(chan)+mac_msg_get_hdrlen(ul_req);
                        MacMessage msg = mac_msg_create_ul_req(remaining_bytes);
						lchan_add_message(chan, msg);
						mac_msg_destroy(msg);
					}
					MacMessage msg = mac_frag_get_fragment(mac->fragmenter,
													lchan_unused_bytes(chan),1);
					lchan_add_message(chan, msg);
					mac->stats.bytes_tx+=msg->payload_len;
					mac_msg_destroy(msg);
				} else {
					// client is assigned to slot but has no data
					// send keepalive instead.
					MacMessage msg = mac_msg_create_keepalive();
					lchan_add_message(chan, msg);
					mac_msg_destroy(msg);
				}
				lchan_calc_crc(chan);
				phy_map_ulslot(mac->phy,chan,next_sfn, i, mac->ul_mcs);
				lchan_destroy(chan);
				queuesize = mac_frag_get_buffersize(mac->fragmenter);
			}
		}
	}

	// check for ULctrl slots
	if (num_slot_assigned(mac->ul_ctrl_assignments, MAC_ULCTRL_SLOTS, 1)>0) {
		mac->last_assignment = mac->subframe_cnt;

		// check if we have to create ul_req
		if (queuesize>0) {
			MacMessage msg = mac_msg_create_ul_req(queuesize);
			ringbuf_put(mac->msg_control_queue, msg);
		}

		// if there are no ctrl messages to be sent we have to create keepalive
		if (ringbuf_isempty(mac->msg_control_queue)) {
			MacMessage msg = mac_msg_create_keepalive();
			ringbuf_put(mac->msg_control_queue, msg);
		}

		// create logical channel with control messages
		LogicalChannel chan = lchan_create(get_ulctrl_slot_size(mac->phy->common)/8,CRC8);
		lchan_add_all_msgs(chan, mac->msg_control_queue);
		lchan_calc_crc(chan);
		// find the ulctrl slot in which we can transmit
		for (int i=0; i<MAC_ULCTRL_SLOTS; i++) {
			if (mac->ul_ctrl_assignments[i] == UE_ASSIGNED) {
				phy_map_ulctrl(mac->phy,chan,next_sfn,i);
				LOG_SFN_MAC(DEBUG,"[MAC UE] map ulctrl %d %d\n",mac->phy->common->tx_subframe,mac->phy->common->tx_symbol);
			}
		}
		lchan_destroy(chan);
	}
	LOG_SFN_MAC(TRACE,"[MAC UE] scheduler done.\n");
	mac->subframe_cnt++;
}

// Main interface function that is called from PHY when receiving a
// logical channel. Function will extract messages and call
// the message handler
void mac_ue_rx_channel(MacUE mac, LogicalChannel chan, uint is_broadcast)
{
	// Note that user was assigned some slot
	mac->last_assignment = mac->subframe_cnt;

	// Verify the CRC
	if(!lchan_verify_crc(chan)) {
		LOG_SFN_MAC(WARN, "[MAC UE] lchan CRC invalid. Dropping.\n");
		mac->stats.chan_rx_fail++;
		lchan_destroy(chan);
		return;
	}

	MacMessage msg = NULL;
	// Get all messages from the logical channel and handle them
	do {
		msg = lchan_parse_next_msg(chan, 0);
		if (msg!=NULL) {
            mac_ue_handle_message(mac,msg, is_broadcast);
		}
	} while (msg != NULL);

	lchan_destroy(chan);
	mac->stats.chan_rx_succ++;
}

// Add a higher layer packet to the tx queue
int mac_ue_add_txdata(MacUE mac, MacDataFrame frame)
{
	return mac_frag_add_frame(mac->fragmenter, frame);
}

int mac_ue_is_associated(MacUE mac)
{
	return mac->is_associated;
}

// Send a request to switch to another MCS in DL or UL
void mac_ue_req_mcs_change(MacUE mac, uint mcs, uint is_ul)
{
    MacMessage msg = mac_msg_create_mcs_change_req(is_ul,mcs);
    if (!ringbuf_put(mac->msg_control_queue,msg)) {
        LOG(WARN,"[MAC UE] could not add mcs change request to control queue\n");
        mac_msg_destroy(msg);
    }
}

int mac_ue_get_timing_advance(MacUE mac)
{
    return mac->timing_advance;
}

// Thread listens to a tap device and adds data received from tap to mac txqueue
void* mac_ue_tap_rx_th(void* arg)
{
    MacUE mac = (MacUE)arg;
	// wait until tap device is created
	while (mac->tapdevice == NULL) {
		usleep(10000);
	}
	LOG(INFO,"[MAC/TAP] start TAP thread\n");
	while (1) {
		// ensure that we we have space to add a packet to the mac queue
		while (mac_frag_queue_full(mac->fragmenter)) {
			usleep(10000);
		}

		// wait for packet from TAP
		tap_receive(mac->tapdevice);

		// forward to MAC
		if (mac->tapdevice->bytes_rec>0) {
			MacDataFrame frame = dataframe_create(mac->tapdevice->bytes_rec);
			memcpy(frame->data,mac->tapdevice->buffer,frame->size);
			if (!mac_ue_add_txdata(mac, frame)) {
				dataframe_destroy(frame);
				LOG(WARN,"[MAC UE] could not forward TAP data to MAC. queue full\n");
			}
		}
	}
	return NULL;
}
