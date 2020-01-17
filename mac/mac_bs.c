/*
 * mac_bs.c
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */
#include "mac_bs.h"
#include "../log.h"


MacBS mac_bs_init()
{
	MacBS macinst = malloc(sizeof(struct MacBS_s));
	memset(macinst, 0, sizeof(struct MacBS_s));

	for (int i=0; i<MAX_USER; i++) {
		macinst->UE[i] = NULL;
	}
	macinst->broadcast_ctrl_queue = ringbuf_create(MAC_MSG_BUF_SIZE);
	macinst->broadcast_data_fragmenter = mac_frag_init();
	return macinst;
}

void mac_bs_destroy(MacBS mac)
{
	ringbuf_destroy(mac->broadcast_ctrl_queue);
	// TODO mac bs destroy
}

void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, ofdmframesync fs, uint timingadvance)
{
	MacMessage response;
	// find the first unused userid
	uint8_t userid = 0;
	while((userid<MAX_USER) && ((mac->UE[userid]!=NULL) ||
			(userid == USER_BROADCAST) || (userid == USER_UNUSED))) {
		userid++;
	}
	if (userid==MAX_USER) {
		// no free userid. generate NAK
		LOG(INFO,"[MAC BS] add_new_ue: no free userID\n");
		response = mac_msg_create_associate_response(0,rachuserid, assoc_resp_full);
	} else {
		// create user instance and association response
		user_s* new_ue = malloc(sizeof(user_s));
		new_ue->msg_control_queue = ringbuf_create(MAC_MSG_BUF_SIZE);
		new_ue->fragmenter = mac_frag_init();
		new_ue->reassembler = mac_assmbl_init();
		new_ue->userid = userid;
		new_ue->ul_queue = 0;
		new_ue->fs = fs;
		mac->UE[userid] = new_ue;
		response = mac_msg_create_associate_response(userid,rachuserid, assoc_resp_success);

		// Inform the UE of its TA
		mac_bs_update_timingadvance(mac, userid, timingadvance);
	}

	// Response will be sent via broadcast channel
	ringbuf_put(mac->broadcast_ctrl_queue, response);



}


int mac_bs_add_txdata(MacBS mac, uint8_t destUserID, MacDataFrame frame)
{
	//TODO function arg could be ethernet-MAC packet or sth?
	user_s* destUser = mac->UE[destUserID];

	if (destUser == NULL) {
		LOG(WARN,"[MAC BS] add_txdata: user %d does not exist!\n",destUserID);
		return 0;
	}

	uint ret = mac_frag_add_frame(destUser->fragmenter,frame);
	if (ret == 0) {
		LOG(WARN,"[MAC BS] add_txdata: msg queue is full. dropping packet!\n");
		return 0;
	}
	return 1;
}

// Set new timing advance parameter and generate a control message
// to inform the UE
void mac_bs_update_timingadvance(MacBS mac, uint userid, uint timingadvance)
{
	if (mac->UE[userid]) {
		mac->UE[userid]->timingadvance = timingadvance;
		MacMessage msg = mac_msg_create_timing_advance(timingadvance);
		ringbuf_put(mac->UE[userid]->msg_control_queue, msg);
	} else {
		LOG(ERR,"[MAC BS] cannot set TA for user %d. Does not exist\n",userid);
	}
}

// Handle incoming messages from PHY layer
int mac_bs_handle_message(MacBS mac, MacMessage msg, uint8_t userID)
{
	user_s* user = mac->UE[userID];
	MacDataFrame frame;
	switch (msg->type) {
	case ul_req:
		// Update the user uplink queue state
		user->ul_queue=msg->hdr.ULreq.packetqueuesize;
		break;
	case channel_quality:
		LOG(DEBUG, "[MAC BS] channel quality measurement not implemented yet\n");
		break;
	case keepalive:
		break;
	case control_ack:
		break;
	case ul_data:
		frame = mac_assmbl_reassemble(user->reassembler,msg);
		if (frame != NULL) {
			printf("[MAC BS] rec %d bytes: %s\n",frame->size,frame->data);
			//TODO forward received frame to higher layer
			dataframe_destroy(frame);
		}
		break;
	default:
		LOG(WARN,"[MAC BS] unexpected MacMsg ID: %d\n",msg->type);
		return 0;
	}
	mac_msg_destroy(msg);
	return 1;
}

// Main interface function that is called from PHY when receiving a
// logical channel. Function will extract messages and call
// the message handler
void mac_bs_rx_channel(MacBS mac, LogicalChannel chan, uint userid)
{
	// Verify the CRC
	if(!lchan_verify_crc(chan)) {
		LOG(INFO, "[MAC BS] lchan CRC invalid. Dropping.\n");
		return;
	}

	MacMessage msg = NULL;
	// Get all messages from the logical channel and handle them
	do {
		lchan_parse_next_msg(chan, 1);
		if (msg) {
			mac_bs_handle_message(mac,msg, userid);
		}
	} while (msg != NULL);

	lchan_destroy(chan);
}

user_s* get_next_user(MacBS mac, uint curr_user)
{
	uint next_user = curr_user;
	do {
		next_user = (next_user + 1) % MAX_USER;
	} while (mac->UE[next_user] == NULL && curr_user!=next_user);
	return mac->UE[next_user];
}

int ue_has_dldata(user_s* ue)
{
	return (mac_frag_has_fragment(ue->fragmenter) ||
			ringbuf_isempty(ue->msg_control_queue));
}

void mac_bs_run_scheduler(MacBS mac)
{
	uint slot_idx = 0;
	uint user_id = 0;
	// TODO 1. Assign UL ctrl slots

	// 2. check Broadcast queue
	LogicalChannel bchan = NULL;
	if (!ringbuf_isempty(mac->broadcast_ctrl_queue)) {
		bchan = lchan_create(get_tbs_size(mac->phy->common,0)/8, CRC16);
		lchan_add_all_msgs(bchan,mac->broadcast_ctrl_queue);
		// reserve a slot for broadcasting
		mac->dl_data_assignments[slot_idx++] = USER_BROADCAST;
	}

	// 3. iterate over all DL slots and assign it to the users
	// start by disabling all slots
	for (int i=0; i<MAC_DLDATA_SLOTS; i++) {
		mac->dl_data_assignments[i] = USER_UNUSED;
	}
	// assign slots to active users
	while (slot_idx < MAC_DLDATA_SLOTS) {
		// get next user. Round robin allocation
		user_s* ue = get_next_user(mac,user_id);
		if (ue==NULL) {
			// no active user at all. stop
			break;
		}
		// check whether the user has DL data or DL ctrl data
		if (ue_has_dldata(ue)) {
			mac->dl_data_assignments[slot_idx++] = ue->userid;
			user_id = ue->userid; // update last active user
		} else if (ue->userid == user_id) {
			break; // no other active user found. stop
		}
	}

	// 4. iterate over each UL slot and assign it
	// TODO check that assignment do not overlap with DL slots
	slot_idx = 0;
	user_id = 0;
	while (slot_idx < MAC_ULDATA_SLOTS) {
		// get next user. Round robin allocation
		user_s* ue = get_next_user(mac,user_id);
		if (ue==NULL) {
			// no active user at all. stop
			break;
		}

		// check whether the user has pending ul data
		if (ue->ul_queue > 0) {
			mac->ul_data_assignments[slot_idx++] = ue->userid;
			user_id = ue->userid; // update last active user
		} else if (ue->userid == user_id) {
			// no other active user found. assign to
			// current user even if there is no ul req
			mac->ul_data_assignments[slot_idx++] = ue->userid;
			user_id = ue->userid; // update last active user
		}
	}

	// Assign UL ctrl slots
	// TODO correctly assign UL ctrl slots. This assigns to userids that are inactive
	uint id = mac->phy->common->tx_subframe *2;
	mac->ul_ctrl_assignments[0] = id;
	mac->ul_ctrl_assignments[1] = id+1;

	// 5. Generate DL slot data
	for (int slot=0; slot<MAC_DLDATA_SLOTS; slot++) {
		user_s* ue = mac->UE[mac->dl_data_assignments[slot]];
		if (ue == NULL) {
			continue; // skip slots that are assigned for broadcast or disabled
		}

		// Generate logical channel
		uint tbs = get_tbs_size(mac->phy->common, ue->dl_mcs);
		LogicalChannel chan = lchan_create(tbs/8, CRC16);
		lchan_add_all_msgs(chan, ue->msg_control_queue);
		if (mac_frag_has_fragment(ue->fragmenter)) {
			uint payload_size = lchan_unused_bytes(chan);
			MacMessage msg = mac_frag_get_fragment(ue->fragmenter, payload_size, 0);
			lchan_add_message(chan,msg);
			mac_msg_destroy(msg);
		}
		lchan_calc_crc(chan);
        phy_map_dlslot(mac->phy, chan, slot, ue->userid, ue->dl_mcs);
        lchan_destroy(chan);
	}

	// 5.2 set UL assignments
	phy_assign_dlctrl_ud(mac->phy, mac->ul_data_assignments);
	phy_assign_dlctrl_uc(mac->phy, mac->ul_ctrl_assignments);
	// write the Downlink control channel to the subcarriers
	phy_map_dlctrl(mac->phy);

	// Log schedule
	LOG(DEBUG,"[MAC BS] Scheduler user assignments:\n");
	LOG(DEBUG,"DL data slots: %4d %4d %4d %4d\n", mac->dl_data_assignments[0],
			mac->dl_data_assignments[1],mac->dl_data_assignments[2],mac->dl_data_assignments[3]);
	LOG(DEBUG,"UL data slots: %4d %4d %4d %4d\n", mac->dl_data_assignments[0],
			mac->ul_data_assignments[1],mac->ul_data_assignments[2],mac->ul_data_assignments[3]);
	LOG(DEBUG,"UL ctrl slots: %4d %4d\n", mac->ul_ctrl_assignments[0],mac->ul_ctrl_assignments[1]);
}
