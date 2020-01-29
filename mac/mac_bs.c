/*
 * mac_bs.c
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */
#include "mac_bs.h"
#include "../util/log.h"

// log makro to log with subframe number
#define LOG_SFN(level, ...) do { if (level>=LOG_LEVEL) \
	{ printf("[%2d %2d]",mac->phy->common->rx_subframe,mac->phy->common->rx_symbol); \
	  printf(__VA_ARGS__); }} while(0);


MacBS mac_bs_init()
{
	MacBS macinst = calloc(sizeof(struct MacBS_s),1);

	for (int i=0; i<MAX_USER; i++) {
		macinst->UE[i] = NULL;
	}
	macinst->broadcast_ctrl_queue = ringbuf_create(MAC_MSG_BUF_SIZE);
	macinst->broadcast_data_fragmenter = mac_frag_init();

	macinst->last_added_rachuserid=-1;
	macinst->last_added_userid=-1;

	return macinst;
}

void mac_bs_set_phy_interface(MacBS mac, struct PhyBS_s* phy)
{
	mac->phy = phy;
}

void mac_bs_destroy(MacBS mac)
{
	ringbuf_destroy(mac->broadcast_ctrl_queue);
	// TODO mac bs destroy
}

void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, uint8_t rach_try_cnt, ofdmframesync fs, int timing_diff)
{
	MacMessage response=NULL;
	uint8_t userid = 0;

	// if this isnt the first time an association is tried,
	// check if this is the same user as the last one added
	if (rach_try_cnt > 0 && mac->last_added_rachuserid==rachuserid) {
		userid = mac->last_added_userid;
		response = mac_msg_create_associate_response(userid,rachuserid, assoc_resp_success);
		LOG(INFO,"[MAC BS] Double assoc req! rachuserid %d, user ID %d\n",rachuserid,userid);
		// Response will be sent via broadcast channel
		ringbuf_put(mac->broadcast_ctrl_queue, response);
		// Inform the UE of its TA
		mac_bs_update_timingadvance(mac, userid, timing_diff);
	} else {
		// find the first unused userid
		while((userid<MAX_USER) && ((mac->UE[userid]!=NULL) ||
				(userid == USER_BROADCAST) || (userid == USER_UNUSED))) {
			userid++;
		}
		if (userid==MAX_USER) {
			// no free userid. generate NAK
			LOG(INFO,"[MAC BS] add_new_ue: no free userID\n");
			response = mac_msg_create_associate_response(0,rachuserid, assoc_resp_full);
			// Response will be sent via broadcast channel
			ringbuf_put(mac->broadcast_ctrl_queue, response);
		} else {
			// create user instance and association response
			user_s* new_ue = calloc(sizeof(user_s),1);
			new_ue->msg_control_queue = ringbuf_create(MAC_MSG_BUF_SIZE);
			new_ue->fragmenter = mac_frag_init();
			new_ue->reassembler = mac_assmbl_init();
			new_ue->userid = userid;
			new_ue->ul_queue = 0;
			new_ue->dl_mcs = 0;
			new_ue->ul_mcs = 0;
			new_ue->fs = fs;
			mac->UE[userid] = new_ue;
			response = mac_msg_create_associate_response(userid,rachuserid, assoc_resp_success);
			LOG(INFO,"[MAC BS] add new user! rachuserid %d, new ID %d\n",rachuserid,userid);
			// Response will be sent via broadcast channel
			ringbuf_put(mac->broadcast_ctrl_queue, response);
			// Inform the UE of its TA
			mac_bs_update_timingadvance(mac, userid, timing_diff);
			mac->last_added_userid = userid;
			mac->last_added_rachuserid = rachuserid;
		}
	}
}

// Try to get the receiver object for the given userid
// returns NULL if user does not exist
ofdmframesync mac_bs_get_receiver(MacBS mac, uint userid)
{
	if (mac->UE[userid]!=NULL) {
		return mac->UE[userid]->fs;
	} else {
		return NULL;
	}
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
	LOG(DEBUG,"[MAC BS] added txdata frame for user %d\n",destUserID);
	return 1;
}

// Set new timing advance parameter and generate a control message
// to inform the UE
void mac_bs_update_timingadvance(MacBS mac, uint userid, int timing_diff)
{
	if (mac->UE[userid]) {
		mac->UE[userid]->timingadvance = timing_diff;
		MacMessage msg = mac_msg_create_timing_advance(mac->UE[userid]->timingadvance);
		ringbuf_put(mac->UE[userid]->msg_control_queue, msg);
	} else {
		LOG(ERR,"[MAC BS] cannot set TA for user %d. Does not exist\n",userid);
	}
}

// Handle incoming messages from PHY layer
int mac_bs_handle_message(MacBS mac, MacMessage msg, uint8_t userID)
{

	user_s* user = mac->UE[userID];
	if (user==NULL) {
		mac_msg_destroy(msg);
		return 0;
		LOG_SFN(WARN,"[MAC BS] received message for unassociated user %d. Drop\n",userID);
	}
	MacDataFrame frame;
	switch (msg->type) {
	case ul_req:
		// Update the user uplink queue state
		user->ul_queue=msg->hdr.ULreq.packetqueuesize;
		LOG_SFN(DEBUG,"[MAC BS] ul_req from user %d. Queuesize: %d\n", userID,user->ul_queue);
		break;
	case channel_quality:
		LOG_SFN(DEBUG, "[MAC BS] channel quality measurement not implemented yet\n");
		break;
	case keepalive:
		LOG_SFN(DEBUG,"[MAC BS] keepalive from user %d\n",userID);
		break;
	case control_ack:
		break;
	case ul_data:
		frame = mac_assmbl_reassemble(user->reassembler,msg);
		if (frame != NULL) {
			mac->stats.bytes_rx+=frame->size;
			LOG_SFN(INFO,"[MAC BS] received frame with %d bytes!\n",frame->size);
			//TODO forward received frame to higher layer
			uint sfn;
			memcpy(&sfn, frame->data,sizeof(uint));
			LOG(INFO,"[MAC BS] received frame ID: %d\n",sfn);
			dataframe_destroy(frame);
		}
		break;
	default:
		LOG_SFN(WARN,"[MAC BS] unexpected MacMsg ID: %d\n",msg->type);
		mac_msg_destroy(msg);
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
		LOG_SFN(INFO, "[MAC BS] lchan CRC%d invalid. Dropping.\n",8*chan->crc_type);
		lchan_destroy(chan);
		mac->stats.chan_rx_fail++;
		return;
	}

	MacMessage msg = NULL;
	// Get all messages from the logical channel and handle them
	do {
		msg = lchan_parse_next_msg(chan, 1);
		if (msg) {
			mac_bs_handle_message(mac,msg, userid);
		}
	} while (msg != NULL);

	lchan_destroy(chan);
	mac->stats.chan_rx_succ++;
}

user_s* get_next_user(MacBS mac, uint curr_user)
{
	curr_user = curr_user % MAX_USER;
	uint next_user = curr_user;
	do {
		next_user = (next_user + 1) % MAX_USER;
	} while (mac->UE[next_user] == NULL && curr_user!=next_user);
	return mac->UE[next_user];
}

int ue_has_dldata(user_s* ue)
{
	return (mac_frag_has_fragment(ue->fragmenter) ||
			!ringbuf_isempty(ue->msg_control_queue));
}

void mac_bs_map_slot(MacBS mac, uint subframe, uint slot, user_s* ue)
{
	// Generate logical channel
	uint tbs = get_tbs_size(mac->phy->common, ue->dl_mcs);
	LogicalChannel chan = lchan_create(tbs/8, CRC16);
	lchan_add_all_msgs(chan, ue->msg_control_queue);
	if (mac_frag_has_fragment(ue->fragmenter)) {
		uint payload_size = lchan_unused_bytes(chan);
		MacMessage msg = mac_frag_get_fragment(ue->fragmenter, payload_size, 0);
		lchan_add_message(chan,msg);
		mac->stats.bytes_tx+=msg->payload_len;
		mac_msg_destroy(msg);
	}
	lchan_calc_crc(chan);
    phy_map_dlslot(mac->phy, chan, subframe%2, slot, ue->userid, ue->dl_mcs);
    lchan_destroy(chan);
}

void mac_bs_run_scheduler(MacBS mac)
{
	uint slot_idx = 0;
	uint user_id = 0;
	uint next_sfn;

	if (mac->phy->common->tx_symbol==0) {
		// subframe just started. schedule for this one.
		// TODO set rules when the scheduler should run
		next_sfn = mac->phy->common->tx_subframe;
	} else {
		next_sfn = (mac->phy->common->tx_subframe+1) % FRAME_LEN;
	}

	// 1. Assign UL ctrl slots
	// TODO correctly assign UL ctrl slots. This assigns to userids that are inactive
	uint id = mac->phy->common->tx_subframe *2;
	mac->ul_ctrl_assignments[0] = id;
	mac->ul_ctrl_assignments[1] = id+1;

	// 2. check Broadcast queue
	// TODO enable data broadcasting
	LogicalChannel bchan = NULL;
	if (!ringbuf_isempty(mac->broadcast_ctrl_queue)) {
		bchan = lchan_create(get_tbs_size(mac->phy->common,0)/8, CRC16);
		lchan_add_all_msgs(bchan,mac->broadcast_ctrl_queue);
		// reserve a slot for broadcasting
		mac->dl_data_assignments[slot_idx] = USER_BROADCAST;
		// Map slot
		lchan_calc_crc(bchan);
        phy_map_dlslot(mac->phy, bchan, next_sfn%2, slot_idx, USER_BROADCAST, 0);
        lchan_destroy(bchan);
        slot_idx++;
	}

	// 3. iterate over all DL slots and assign it to the users
	// start by disabling all slots
	for (int i=slot_idx; i<MAC_DLDATA_SLOTS; i++) {
		mac->dl_data_assignments[i] = USER_UNUSED;
	}
	// assign slots to active users
	// get first active user
	// TODO if there is much traffic, users with high userid do not get assignments. Better do round robin over multiple subframes
	user_s* ue = get_next_user(mac,0);
	if (ue!=NULL) {
		user_id = ue->userid;
	}
	while (slot_idx < MAC_DLDATA_SLOTS) {
		if (ue==NULL) {
			// no active user at all. stop
			break;
		}
		// get next user. Round robin allocation
		ue = get_next_user(mac,ue->userid);

		// check whether the user has DL data or DL ctrl data
		if (ue_has_dldata(ue)) {
			mac_bs_map_slot(mac,next_sfn,slot_idx,ue);
			mac->dl_data_assignments[slot_idx++] = ue->userid;
			user_id = ue->userid; // update last active user
		} else if (ue->userid == user_id) {
			break; // no other active user found. stop
		}
	}
	// assure that the sync slot is not assigned for user traffic
	if (next_sfn==0) {
		mac->dl_data_assignments[MAC_DLDATA_SLOTS-1] = USER_UNUSED;
	}
	// TODO assure that no data slot is assigned during RA slot?

	// 4. iterate over each UL slot and assign it
	// TODO check that assignment do not overlap with DL slots
	slot_idx = 0;
	// get first active user
	ue = get_next_user(mac,0);
	if (ue!=NULL) {
		user_id = ue->userid;
	}
	while (slot_idx < MAC_ULDATA_SLOTS) {
		if (ue==NULL) {
			// no active user at all. stop
			break;
		}
		// get next user. Round robin allocation
		user_s* ue = get_next_user(mac,ue->userid);

		// check whether the user has pending ul data
		if (ue->ul_queue > 0) {
			mac->ul_data_assignments[slot_idx++] = ue->userid;
			user_id = ue->userid; // update last active user
			// update ul queue len:
			ue->ul_queue -= fmin(ue->ul_queue, get_tbs_size(mac->phy->common, ue->ul_mcs)/8-5);
		} else if (ue->userid == user_id) {
			// no other active user found. assign to
			// current user even if there is no ul req
			//mac->ul_data_assignments[slot_idx++] = ue->userid;
			//user_id = ue->userid; // update last active user
			break; //Disable proactive assignment
		}
	}
	// force RA slot to be not assigned.
	if (next_sfn==0) {
		mac->ul_data_assignments[MAC_ULDATA_SLOTS-1] = USER_UNUSED;
	}

	// 5. set slot assignments in PHY
	phy_assign_dlctrl_dd(mac->phy, mac->dl_data_assignments);
	phy_assign_dlctrl_ud(mac->phy, next_sfn%2, mac->ul_data_assignments);
	phy_assign_dlctrl_uc(mac->phy, next_sfn%2, mac->ul_ctrl_assignments);
	// write the Downlink control channel to the subcarriers
	phy_map_dlctrl(mac->phy, next_sfn%2);

	// Log schedule
	LOG(TRACE,"[MAC BS] Scheduler user assignments:\n");
	LOG(TRACE,"         DL data slots: %4d %4d %4d %4d\n", mac->dl_data_assignments[0],
			mac->dl_data_assignments[1],mac->dl_data_assignments[2],mac->dl_data_assignments[3]);
	LOG(TRACE,"         UL data slots: %4d %4d %4d %4d\n", mac->ul_data_assignments[0],
			mac->ul_data_assignments[1],mac->ul_data_assignments[2],mac->ul_data_assignments[3]);
	LOG(TRACE,"         UL ctrl slots: %4d %4d\n", mac->ul_ctrl_assignments[0],mac->ul_ctrl_assignments[1]);
}
