/*
 * mac_channels.h
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_CHANNELS_H_
#define MAC_MAC_CHANNELS_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "mac_messages.h"
#include "../log.h"

// Define CRC types
#define CRC8 8
#define CRC16 16

typedef struct {
	uint payload_len;
	uint writepos;
	uint crc_type;
	uint8_t* data;
} LogicalChannel_s;

typedef LogicalChannel_s* LogicalChannel;

// Function declarations
LogicalChannel lchan_create(uint size,uint crc_type);
void lchan_destroy(LogicalChannel chan);
int  lchan_unused_bytes(LogicalChannel chan);
int  lchan_add_message(LogicalChannel chan, MacMessage msg);
MacMessage lchan_parse_next_msg(LogicalChannel chan, uint ul_flag);
void lchan_calc_crc(LogicalChannel chan);
int  lchan_verify_crc(LogicalChannel chan);

#endif /* MAC_MAC_CHANNELS_H_ */
