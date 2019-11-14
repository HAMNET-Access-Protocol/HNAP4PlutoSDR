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

typedef struct {
	uint userid;
	uint payload_len;
	uint writepos;
	uint8_t* data;
} LogicalChannel_s;

typedef LogicalChannel_s* LogicalChannel;

#endif /* MAC_MAC_CHANNELS_H_ */
