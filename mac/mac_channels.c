/*
 * mac_channels.c
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */


#include "mac_channels.h"
#include <liquid/liquid.h>

#define CRC_LEN	2 // CRC length in bytes (16bit crc)

// allocate memory for a channel object
LogicalChannel lchan_create(uint size)
{
	LogicalChannel chan = malloc(sizeof(LogicalChannel_s));
	chan->data = malloc(size);
	chan->writepos = 0;
	chan->payload_len = size;
	return chan;
}

// free the memory allocated for the channel
void lchan_destroy(LogicalChannel chan)
{
	free(chan->payload_len);
	free(chan);
}

// Add a MAC message to the logical channel object
// returns 1 on success, 0 if there is no space left
int lchan_add_message(LogicalChannel chan, MacMessage msg)
{
	uint msg_len = msg->hdr_len + msg->payload_len;
	uint buf_len = lchan_unused_bytes(chan);
	if ( msg_len < buf_len) {
		mac_msg_generate(msg,chan->data+chan->writepos, buf_len);
		chan->writepos +=msg_len;
		return 1;
	}
	return 0;
}

// Get number of bytes which are unused, i.e. can be used to add
// messages
int lchan_unused_bytes(LogicalChannel chan)
{
	return chan->payload_len - chan->writepos - CRC_LEN;
}

// Calculate the crc-16 for a logical channel and add it
// at the end of the payload
void lchan_calc_crc(LogicalChannel chan)
{
	uint16_t crc = crc_generate_key(LIQUID_CRC_16, chan->data, chan->payload_len-2);
	memcpy(&chan->data[chan->payload_len-2],&crc,2);
}

// Verify the CRC
int lchan_verify_crc(LogicalChannel chan)
{
	uint16_t crc;
	memcpy(&crc, &chan->data[chan->payload_len-2], 2);
	return crc_validate_message(LIQUID_CRC_16, chan->data, chan->payload_len-2, crc);
}
