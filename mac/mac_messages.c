/*
 * mac_messages.c
 *
 *  Created on: Dec 9, 2019
 *      Author: lukas
 */

#include "mac_messages.h"
#include "../log.h"

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

	MacMessage genericmsg = calloc(sizeof(MacMessage_s),1);
	genericmsg->type = type;
	genericmsg->hdr_len = hdrlen;
	genericmsg->payload_len = 0;
	genericmsg->data = NULL;
	return genericmsg;
}

/* Mac Message functinons */

MacMessage mac_msg_create_associate_response(uint userID, uint rachUserID,
												uint response)
{
	MacMessage genericmsg = mac_msg_create_generic(associate_response);
	MacAssociateResponse* msg = &genericmsg->hdr.AssociateResponse;

	genericmsg->hdr_bin[0] = (associate_response & 0b111) << 5;
	genericmsg->hdr_bin[0] |= (userID & 0b1111) << 1;
	genericmsg->hdr_bin[0] |= (rachUserID & 0b1000) >> 3;
	genericmsg->hdr_bin[1] = (rachUserID & 0b111) << 5;
	genericmsg->hdr_bin[1] |= (response & 0b111) << 2;
	genericmsg->hdr_bin[1] |= (PROTO_VERSION & 0b11);

	msg->ctrl_id = associate_response & 0b111;
	msg->userid = userID;
	msg->rachuserid = rachUserID;
	msg->response = response;
	msg->protoVersion = PROTO_VERSION;
	return genericmsg;
}

MacMessage mac_msg_create_dl_mcs_info(uint mcs)
{
	MacMessage genericmsg = mac_msg_create_generic(dl_mcs_info);
	MacDLMCSInfo* msg = &genericmsg->hdr.DLMCSInfo;

	genericmsg->hdr_bin[0] = (dl_mcs_info & 0b111) << 5;
	genericmsg->hdr_bin[0] |= mcs & 0b11111;

	msg->ctrl_id = dl_mcs_info & 0b111;
	msg->mcs = mcs;
	return genericmsg;
}

MacMessage mac_msg_create_ul_mcs_info(uint mcs)
{
	MacMessage genericmsg = mac_msg_create_generic(ul_mcs_info);
	MacULMCSInfo* msg = &genericmsg->hdr.ULMCSInfo;

	genericmsg->hdr_bin[0] = (ul_mcs_info & 0b111) << 5;
	genericmsg->hdr_bin[0] |= mcs & 0b11111;

	msg->ctrl_id = ul_mcs_info & 0b111;
	msg->mcs = mcs;
	return genericmsg;
}

MacMessage mac_msg_create_timing_advance(uint timingAdvance)
{
	MacMessage genericmsg = mac_msg_create_generic(timing_advance);
	MacTimingAdvance* msg = &genericmsg->hdr.TimingAdvance;

	genericmsg->hdr_bin[0] = (timing_advance & 0b111) << 5;
	genericmsg->hdr_bin[0] |= (timingAdvance >> 8) & 0b11111;
	genericmsg->hdr_bin[1] = timingAdvance & 0xFF;

	msg->ctrl_id = timing_advance & 0b111;
	msg->timingAdvance = timingAdvance;
	return genericmsg;
}

MacMessage mac_msg_create_dl_data(uint data_length, uint8_t final,
							uint8_t seqNr, uint8_t fragNr, uint8_t* data)
{
	MacMessage genericmsg = mac_msg_create_generic(dl_data);
	MacDLdata* msg = &genericmsg->hdr.DLdata;
	genericmsg->payload_len = data_length;

	genericmsg->hdr_bin[0] = (dl_data &0b111) << 5;
	genericmsg->hdr_bin[0] |= (data_length >> 7) & 0b11111;
	genericmsg->hdr_bin[1] = (data_length & 0b01111111) <<1;
	genericmsg->hdr_bin[1] |= final & 0b1;
	genericmsg->hdr_bin[2] = (seqNr & 0b111) << 5;
	genericmsg->hdr_bin[2] |= fragNr & 0b11111;

	msg->ctrl_id = dl_data & 0b111;
	msg->data_length = data_length;
	msg->fragNr = fragNr;
	msg->seqNr = seqNr;
	msg->final_flag = final;
	genericmsg->data = malloc(data_length);
	memcpy(genericmsg->data,data,data_length);

	return genericmsg;
}

MacMessage mac_msg_create_ul_req(uint PacketQueueSize)
{
	MacMessage genericmsg = mac_msg_create_generic(ul_req);
	MacULreq* msg = &genericmsg->hdr.ULreq;

	genericmsg->hdr_bin[0] = (ul_req & 0b111) << 5;
	genericmsg->hdr_bin[0] |= (PacketQueueSize >> 8) & 0b11111;
	genericmsg->hdr_bin[1] = PacketQueueSize & 0xFF;

	msg->ctrl_id = ul_req  & 0b111;
	msg->packetqueuesize = PacketQueueSize;
	return genericmsg;
}

MacMessage mac_msg_create_channel_quality(uint quality_idx)
{
	MacMessage genericmsg = mac_msg_create_generic(channel_quality);
	MacChannelQuality* msg = &genericmsg->hdr.ChannelQuality;

	genericmsg->hdr_bin[0] = (channel_quality & 0b111) >> 5;

	msg->ctrl_id = channel_quality  & 0b111;
	msg->channel_quality = quality_idx;
	return genericmsg;
}

MacMessage mac_msg_create_keepalive()
{
	MacMessage genericmsg = mac_msg_create_generic(keepalive);
	MacKeepalive* msg = &genericmsg->hdr.Keepalive;

	genericmsg->hdr_bin[0] = (keepalive & 0b111) << 5;

	msg->ctrl_id = keepalive  & 0b111;
	msg->reserved = 0;
	return genericmsg;
}

MacMessage mac_msg_create_control_ack(uint acked_ctrl_id)
{
	MacMessage genericmsg = mac_msg_create_generic(control_ack);
	MacControlAck* msg = &genericmsg->hdr.ControlAck;

	genericmsg->hdr_bin[0] = (control_ack & 0b111)<< 5;
	genericmsg->hdr_bin[0] |= (acked_ctrl_id & 0b111) << 2;

	msg->ctrl_id = control_ack  & 0b111;
	msg->acked_ctrl_id = acked_ctrl_id;
	return genericmsg;
}

MacMessage mac_msg_create_ul_data(uint data_length, uint8_t final,
							uint8_t seqNr, uint8_t fragNr, uint8_t* data)
{
	MacMessage genericmsg = mac_msg_create_generic(ul_data);
	MacULdata* msg = &genericmsg->hdr.ULdata;
	genericmsg->payload_len = data_length;

	genericmsg->hdr_bin[0] = (ul_data &0b111) << 5;
	genericmsg->hdr_bin[0] |= (data_length >> 7) & 0b11111;
	genericmsg->hdr_bin[1] = (data_length & 0b01111111) <<1;
	genericmsg->hdr_bin[1] |= final & 0b1;
	genericmsg->hdr_bin[2] = (seqNr & 0b111) << 5;
	genericmsg->hdr_bin[2] |= fragNr & 0b11111;

	msg->ctrl_id = ul_data & 0b111;
	msg->data_length = data_length;
	msg->fragNr = fragNr;
	msg->seqNr = seqNr;
	msg->final_flag = final;
	genericmsg->data = malloc(data_length);
	memcpy(genericmsg->data,data,data_length);

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
	memcpy(buf,&genericmsg->hdr_bin,genericmsg->hdr_len);
	buf += genericmsg->hdr_len;

	// if this is a UL/DL data message, we have to add the payload
	if ((genericmsg->type == dl_data) || (genericmsg->type == ul_data)) {
		memcpy(buf, genericmsg->data, genericmsg->payload_len);
	}
	return 1;
}

void mac_msg_parse_associate_response(MacMessage msg)
{
	msg->hdr.AssociateResponse.ctrl_id = msg->type & 0b111;
	msg->hdr.AssociateResponse.userid = (msg->hdr_bin[0] & 0b00011110) >>1;
	msg->hdr.AssociateResponse.rachuserid = ((msg->hdr_bin[0] & 0b1) << 3)
											| (msg->hdr_bin[1] & 0b11100000) >> 5;
	msg->hdr.AssociateResponse.response = (msg->hdr_bin[1] & 0b11100) >>2;
	msg->hdr.AssociateResponse.protoVersion = msg->hdr_bin[1] & 0b11;
}

void mac_msg_parse_dl_mcs_info(MacMessage msg)
{
	msg->hdr.DLMCSInfo.ctrl_id = msg->type & 0b111;
	msg->hdr.DLMCSInfo.mcs = msg->hdr_bin[0] & 0b11111;
}

void mac_msg_parse_ul_mcs_info(MacMessage msg)
{
	msg->hdr.ULMCSInfo.ctrl_id = msg->type & 0b111;
	msg->hdr.ULMCSInfo.mcs = msg->hdr_bin[0] & 0b11111;
}

void mac_msg_parse_timing_advance(MacMessage msg)
{
	msg->hdr.TimingAdvance.ctrl_id = msg->type & 0b111;
	msg->hdr.TimingAdvance.timingAdvance = ((msg->hdr_bin[0] & 0b11111)<<8)
											| msg->hdr_bin[1];
}

void mac_msg_parse_dl_data(MacMessage msg)
{
	msg->hdr.DLdata.ctrl_id = msg->type & 0b111;
	msg->hdr.DLdata.data_length = ((msg->hdr_bin[0] & 0b11111) << 7)
									| (msg->hdr_bin[1] >> 1);
	msg->hdr.DLdata.final_flag = msg->hdr_bin[1] & 0b1;
	msg->hdr.DLdata.seqNr = msg->hdr_bin[2] >> 5;
	msg->hdr.DLdata.fragNr = msg->hdr_bin[2] & 0b11111;
}

void mac_msg_parse_ul_req(MacMessage msg)
{
	msg->hdr.ULreq.ctrl_id = msg->type & 0b111;
	msg->hdr.ULreq.packetqueuesize = (msg->hdr_bin[0] & 0b11111) << 8
										| msg->hdr_bin[1];
}

void mac_msg_parse_channel_quality(MacMessage msg)
{
	//TBD!
}

void mac_msg_parse_keepalive(MacMessage msg)
{
	msg->hdr.Keepalive.ctrl_id = msg->type & 0b111;
}

void mac_msg_parse_control_ack(MacMessage msg)
{
	msg->hdr.ControlAck.ctrl_id = msg->type & 0b111;
	msg->hdr.ControlAck.acked_ctrl_id = (msg->hdr_bin[0] & 0b11100) >>2;
}

void mac_msg_parse_ul_data(MacMessage msg)
{
	msg->hdr.ULdata.ctrl_id = msg->type & 0b111;
	msg->hdr.ULdata.data_length = ((msg->hdr_bin[0] & 0b11111) << 7)
									| (msg->hdr_bin[1] >> 1);
	msg->hdr.ULdata.final_flag = msg->hdr_bin[1] & 0b1;
	msg->hdr.ULdata.seqNr = msg->hdr_bin[2] >> 5;
	msg->hdr.ULdata.fragNr = msg->hdr_bin[2] & 0b11111;
}

// read from a binary buffer and try to parse a Mac message
MacMessage mac_msg_parse(uint8_t* buf, uint buflen, uint8_t ul_flag)
{
	if (buflen == 0) {
		return NULL;
	}

	CtrlID_e type = (buf[0]>>5) & 0b111;

	// Catch EOF message
	if (type == msg_none) {
		return NULL;
	}
	// UL control messages start at 8 in the enum -> add 0x8
	if (ul_flag) {
		type += 0b1000;
	}

	MacMessage genericmsg = mac_msg_create_generic(type);
	if (genericmsg==NULL) {
		LOG(WARN,"[MAC MSG] Parse: undefined message type %d\n", type);

		return NULL; // undefined msg type, cannot decode
	}

	// Ensure that buf size is large enough
	if (buflen < genericmsg->hdr_len) {
		mac_msg_destroy(genericmsg);
		return NULL;
	}
	memcpy((uint8_t*)&genericmsg->hdr_bin,buf,genericmsg->hdr_len);
	buf+= genericmsg->hdr_len;

	switch (type) {
	case associate_response:
		mac_msg_parse_associate_response(genericmsg);
		break;
	case dl_mcs_info:
		mac_msg_parse_dl_mcs_info(genericmsg);
		break;
	case ul_mcs_info:
		mac_msg_parse_ul_mcs_info(genericmsg);
		break;
	case timing_advance:
		mac_msg_parse_timing_advance(genericmsg);
		break;
	case dl_data:
		mac_msg_parse_dl_data(genericmsg);
		break;
	case ul_req:
		mac_msg_parse_ul_req(genericmsg);
		break;
	case channel_quality:
		mac_msg_parse_channel_quality(genericmsg);
		break;
	case keepalive:
		mac_msg_parse_keepalive(genericmsg);
		break;
	case control_ack:
		mac_msg_parse_control_ack(genericmsg);
		break;
	case ul_data:
		mac_msg_parse_ul_data(genericmsg);
		break;
	default:
		LOG(WARN,"[MAC MSG] Parse: undefined msg type!\n");
	}

	// if this is a UL/DL data message, we have to add the payload
	if ((genericmsg->type == dl_data) || (genericmsg->type == ul_data)) {
		genericmsg->payload_len = genericmsg->hdr.DLdata.data_length;

		if (buflen < genericmsg->hdr_len + genericmsg->payload_len) {
			mac_msg_destroy(genericmsg);
			LOG(WARN,"[MAC MSG] error: decoded payload len is larger than submitted buffer\n");
			return NULL;
		}
		genericmsg->data = malloc(genericmsg->payload_len);
		memcpy(genericmsg->data, buf, genericmsg->payload_len);
	}

	return genericmsg;
}
