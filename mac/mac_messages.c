/*
 * mac_messages.c
 *
 *  Created on: Dec 9, 2019
 *      Author: lukas
 */

#include "mac_messages.h"


/* Local Helper functions */

// returns the message size in bytes
// in case of data messages, only the header size is returned
int mac_msg_get_hdrlen(CtrlID_e type)
{
	switch (type) {
	case associate_response:
		return 2;
	case dl_mcs_info:
		return 1;
	case ul_mcs_info:
		return 1;
	case timing_advance:
		return 2;
	case dl_data:
		return 3;
	case ul_req:
		return 2;
	case channel_quality:
		return 1;
	case keepalive:
		return 1;
	case control_ack:
		return 1;
	case ul_data:
		return 3;
	default:
		// incorrect msg type
		return -1;
	}
}

// Init the generic MAC message struct
MacMessage mac_msg_create_generic(CtrlID_e type)
{
	int hdrlen = mac_msg_get_hdrlen(type);
	if (hdrlen < 0) {
		return NULL;
	}

	MacMessage genericmsg = malloc(sizeof(MacMessage_s));

	genericmsg->type = type;
	genericmsg->hdr_len = hdrlen;
	genericmsg->payload_len = 0;
	return genericmsg;
}

/* Mac Message functinons */

MacMessage mac_msg_create_ul_req(uint PacketQueueSize)
{
	MacMessage genericmsg = mac_msg_create_generic(ul_req);
	MacULreq msg = genericmsg->msg.ULreq;

	msg.ctrl_id = ul_req  & 0b111;
	msg.packetqueuesize = PacketQueueSize;
	return genericmsg;
}

MacMessage mac_msg_create_associate_response(uint userID, uint rachUserID,
												uint response)
{
	MacMessage genericmsg = mac_msg_create_generic(associate_response);
	MacAssociateResponse msg = genericmsg->msg.AssociateResponse;

	msg.ctrl_id = associate_response & 0b111;
	msg.userid = userID;
	msg.rachuserid = rachUserID;
	msg.response = response;
	msg.protoVersion = PROTO_VERSION;
	return genericmsg;
}

MacMessage mac_msg_create_dl_mcs_info(uint mcs)
{
	MacMessage genericmsg = mac_msg_create_generic(dl_mcs_info);
	MacDLMCSInfo msg = genericmsg->msg.DLMCSInfo;

	msg.ctrl_id = dl_mcs_info & 0b111;
	msg.mcs = mcs;
	return genericmsg;
}

MacMessage mac_msg_create_ul_mcs_info(uint mcs)
{
	MacMessage genericmsg = mac_msg_create_generic(ul_mcs_info);
	MacULMCSInfo msg = genericmsg->msg.ULMCSInfo;

	msg.ctrl_id = ul_mcs_info & 0b111;
	msg.mcs = mcs;
	return genericmsg;
}

MacMessage mac_msg_create_timing_advance(uint timingAdvance)
{
	MacMessage genericmsg = mac_msg_create_generic(timing_advance);
	MacTimingAdvance msg = genericmsg->msg.TimingAdvance;

	msg->ctrl_id = timing_advance & 0b111;
	msg.timingAdvance = timingAdvance;
	return genericmsg;
}

MacMessage mac_msg_create_dl_data(uint data_length, uint8_t fragment,
							uint8_t seqNr, uint8_t fragNr, uint8_t* data )
{
	MacMessage genericmsg = mac_msg_create_generic(dl_data);
	MacDLdata msg = genericmsg->msg.DLdata;

	msg.ctrl_id = dl_data & 0b111;
	msg.data_length = data_length;
	genericmsg->data = malloc(data_length);
	return genericmsg;
}

// Free all memory allocated for the message
void mac_msg_destroy(MacMessage genericmsg)
{
	free(genericmsg->data);
	free(genericmsg);
}

// Use the MAC message struct to write the binary
// message to the buf
int mac_msg_generate(MacMessage genericmsg, uint8_t* buf, uint buflen)
{
	if (buflen < genericmsg->hdr_len + genericmsg->payload_len) {
		LOG(WARN, "[MAC MSG] cannot create msg, no space in buffer\n");
		return 0; // not enough space in buffer
	}
	// generate the header
	memcpy(buf,&genericmsg->msg,genericmsg->hdr_len);
	buf += genericmsg->hdr_len;

	// if this is a UL/DL data message, we have to add the payload
	if ((genericmsg->type == dl_data) || (genericmsg->type == ul_data)) {
		memcpy(buf, genericmsg->data, genericmsg->payload_len);
	}
	return 1;
}

// read from a binary buffer and try to parse a Mac message
MacMessage mac_msg_parse(uint8_t* buf, uint buflen, uint8_t ul_flag)
{
	CtrlID_e type = (buf[0]>>5) & 0b111;
	// ul control messages start at 8 in the enum.
	if (ul_flag) {
		type += 0b1000;
	}

	MacMessage genericmsg = mac_message_create_generic(type);
	if (genericmsg==NULL) {
		LOG(WARN,"[MAC MSG] undefined message type when parsing\n");

		return NULL; // undefined msg type, cannot decode
	}

	memcpy((uint8_t*)&genericmsg->msg,buf,genericmsg->hdr_len);
	buf+= genericmsg->hdr_len;

	// if this is a UL/DL data message, we have to add the payload
	if ((genericmsg->type == dl_data) || (genericmsg->type == ul_data)) {
		genericmsg->payload_len = genericmsg->msg.DLdata.data_length;

		if (buflen < genericmsg->hdr_len + genericmsg->payload_len) {
			mac_msg_destroy(genericmsg);
			LOG(WARN,"[MAC MSG] error: decoded payload len is larger than submitted buffer\n");
			return NULL;
		}
		memcpy(genericmsg->data, buf, genericmsg->payload_len);
	}

	return genericmsg;
}
