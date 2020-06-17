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

#ifndef MAC_MAC_CHANNELS_H_
#define MAC_MAC_CHANNELS_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "../util/log.h"
#include "mac_messages.h"

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
