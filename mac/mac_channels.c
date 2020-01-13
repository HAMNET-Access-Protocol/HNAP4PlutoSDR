/*
 * mac_channels.c
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */


#include "mac_channels.h"
#include <liquid/liquid.h>

// allocate memory for a channel object
void lchan_create(LogicalChannel chan, uint size)
{
	chan->data = malloc(size);
	chan->writepos = 0;
	chan->payload_leN
}

// free the memory allocated for the channel
void lchan_delete(LogicalChannel chan)
{
	free(chan->payload_len);
	free(chan);
}

// Add a MAC message to the logical channel object
// returns 1 on success, 0 if there is no space left
int lchan_add_message(LogicalChannel chan, MacMessage msg)
{
	uint msg_len = msg->hdr_len + msg->payload_len;
	uint buf_len = chan->payload_len - chan->writepos;
	if ( msg_len < buf_len) {
		mac_msg_generate(msg,chan->data+chan->writepos, buf_len);
		chan->writepos +=msg_len;
		return 1;
	}
	return 0;
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
