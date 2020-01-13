/*
 * mac_channels.h
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_CHANNELS_H_
#define MAC_MAC_CHANNELS_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "mac_messages.h"

typedef struct {
	uint userid;
	uint payload_len;
	uint writepos;
	uint8_t* data;
} LogicalChannel_s;

typedef LogicalChannel_s* LogicalChannel;

// Function declarations
void lchan_create(LogicalChannel chan, uint size);
void lchan_delete(LogicalChannel chan);
int  lchan_add_message(LogicalChannel chan, MacMessage msg);
void lchan_calc_crc(LogicalChannel chan);
int  lchan_verify_crc(LogicalChannel chan);

#endif /* MAC_MAC_CHANNELS_H_ */
