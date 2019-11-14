/*
 * mac_channels.c
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */


#include "mac_channels.h"

void lchan_create(LogicalChannel chan, uint size)
{
	chan->data = malloc(size);
	chan->writepos = 0;
	chan->payload_len = size;
}
void lchan_add_message(LogicalChannel chan, uint8_t* msg, uint len)
{
	memcpy(&chan->data[chan->writepos],msg,len);
	chan->writepos +=len;
}

