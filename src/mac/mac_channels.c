/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA
 */


#include "mac_channels.h"
#include <liquid/liquid.h>


// allocate memory for a channel object
// Params: 	size: size in bytes
//			crc_len: 8 for 8bit crc, 16 for 16bit crc
LogicalChannel lchan_create(uint size, uint crc_len)
{
	int crc;
	if (crc_len == CRC8) {
		crc=1;
	} else if (crc_len == CRC16) {
		crc=2;
	} else {
		LOG(ERR,"[MAC CHAN] undefined crc type: %d\n",crc_len);
		return NULL;
	}

	LogicalChannel chan = malloc(sizeof(LogicalChannel_s));
	if (chan==NULL) {
	    LOG(ERR,"[MAC CHAN] cannot allocate memory for chan object!\n");
        return chan;
    }
	chan->data = calloc(size,1);
    if (chan->data==NULL) {
        LOG(ERR,"[MAC CHAN] cannot allocate memory for chan data!\n");
        free(chan);
        return NULL;
    }
	chan->writepos = 0;
	chan->payload_len = size;
	chan->crc_type = crc;
	return chan;
}

// free the memory allocated for the channel
void lchan_destroy(LogicalChannel chan)
{
	free(chan->data);
	free(chan);
}

// Add a MAC message to the logical channel object
// returns 1 on success, 0 if there is no space left
int lchan_add_message(LogicalChannel chan, MacMessage msg)
{
	uint msg_len = msg->hdr_len + msg->payload_len;
	uint buf_len = lchan_unused_bytes(chan);
	if ( msg_len <= buf_len) {
		mac_msg_generate(msg,chan->data+chan->writepos, buf_len);
		chan->writepos +=msg_len;
	} else {
		LOG(ERR,"[MAC CHN] could not add message to channel. Too large!\n");
		return 0;
	}
	if (msg_len < buf_len) {
		// Force next byte to 0, so if this is the last message
		// in the channel, the parser can properly detect the end
		chan->data[chan->writepos] = 0;
	}
	return 1;

}

// Try to get the next MAC message in the channel object
MacMessage lchan_parse_next_msg(LogicalChannel chan, uint ul_flag)
{
	MacMessage msg = NULL;
	uint8_t* data_start = chan->data+chan->writepos;
	uint remaining_bytes = chan->payload_len - chan->writepos - chan->crc_type;
	msg = mac_msg_parse(data_start,remaining_bytes, ul_flag);
	if (msg!=NULL)
		chan->writepos += msg->hdr_len + msg->payload_len;
	else
		chan->writepos = chan->payload_len;
	return msg;
}

// Get number of bytes which are unused, i.e. can be used to add
// messages
int lchan_unused_bytes(LogicalChannel chan)
{
	return chan->payload_len - chan->writepos - chan->crc_type;
}

// Calculate the crc-16 for a logical channel and add it
// at the end of the payload
void lchan_calc_crc(LogicalChannel chan)
{
	// if the payload area is partially unused, fill it up with random
	// bytes. This increases robustness during transmission
	for (int i=chan->writepos+1; i<chan->payload_len; i++) {
		chan->data[i] = (uint8_t)rand();
	}

	if (chan->crc_type*8 == CRC16) {
		uint16_t crc = crc_generate_key(LIQUID_CRC_16, chan->data, chan->payload_len-chan->crc_type);
		chan->data[chan->payload_len-2] = (crc >> 8) & 0xFF; // upper byte
		chan->data[chan->payload_len-1] = crc & 0xFF; // lower byte
	} else {
		uint16_t crc = crc_generate_key(LIQUID_CRC_8, chan->data, chan->payload_len-chan->crc_type);
		chan->data[chan->payload_len-1] = crc & 0xFF;
	}
}

// Verify the CRC
int lchan_verify_crc(LogicalChannel chan)
{
	if (chan->crc_type*8 == CRC16) {
		uint16_t crc = (chan->data[chan->payload_len-2] << 8); //upper byte
		crc |= chan->data[chan->payload_len-1] & 0xFF; // lower byte
		return crc_validate_message(LIQUID_CRC_16, chan->data, chan->payload_len-chan->crc_type, crc);
	} else {
		uint16_t crc = chan->data[chan->payload_len-1];
		return crc_validate_message(LIQUID_CRC_8, chan->data, chan->payload_len-chan->crc_type, crc);
	}
}
