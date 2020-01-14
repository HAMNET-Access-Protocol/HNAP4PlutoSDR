/*
 * mac_bs.c
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */
#include "mac_bs.h"
#include <log.h>

MacBS mac_bs_init()
{
	MacBS macinst = malloc(sizeof(MacBS_s));
	memset(macinst, 0, sizeof(MacBS_s));

	for (int i=0; i<MAX_USER; i++) {
		macinst->UE = NULL;
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

void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, ofdmframesync fs)
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
	}

	// Response will be sent via broadcast channel
	ringbuf_put(mac->broadcast_ctrl_queue,(void*)response);
}


int mac_bs_add_txdata(MacBS mac, uint8_t destUserID, uint8_t* buf, uint buflen)
{
	//TODO function arg could be ethernet-MAC packet or sth?
	user_s* destUser = mac->UE[destUserID];
	void* data=NULL;

	if (destUser == NULL) {
		LOG(WARN,"[MAC BS] add_txdata: user %d does not exist!\n",destUserID);
		return 0;
	}

	uint ret = mac_frag_add_frame(destUser->fragmenter,buf,buflen);
	if (ret == 0) {
		LOG(WARN,"[MAC BS] add_txdata: msg queue is full. dropping packet!\n");
		return 0;
	}
	return 1;
}

// Handle incoming messages from PHY layer
int mac_bs_handle_message(MacBS mac, MacMessage msg, uint8_t userID)
{
	user_s* user = mac->UE[userID];

	switch (msg->type) {
	case ul_req:
		// Update the user uplink queue state
		user->ul_queue=msg->msg->ULreq.packetqueuesize;
		break;
	case channel_quality:
		LOG(DEBUG, "[MAC BS] channel quality measurement not implemented yet\n");
		break;
	case keepalive:
		break;
	case control_ack:
		break;
	case ul_data:
		MacDataFrame frame = mac_assmbl_reassemble(user->reassembler,msg);
		if (frame != NULL) {
			printf("[MAC BS] rec %d bytes: %s\n",frame->size,frame->data);
			//TODO forward received frame to higher layer
			free(frame->data);
			free(frame);
		}
		break;
	default:
		LOG(WARN,"[MAC BS] unexpected MacMsg ID: %d\n",msg->type);
		return 0;
	}
	mac_msg_destroy(msg);
	return 1;
}
