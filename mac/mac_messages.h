/*
 * mac_messages.h
 *
 *  Created on: Nov 27, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_MESSAGES_H_
#define MAC_MAC_MESSAGES_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
	associate_response = 1,
	dl_mcs_info,
	ul_mcs_info,
	timing_advance,
	dl_data = 7
} DLctrl_id;

typedef enum {
	ul_req = 1,
	channel_quality,
	keepalive,
	control_ack,
	ul_data = 7
} ULctrl_id;

typedef struct {
	uint16_t ctrl_id :3;
	uint16_t userid :4;
	uint16_t rachuserid :4;
	uint16_t response :3;
	uint16_t protoVersion : 3;
} MacAssociateResponse;

typedef struct {
	uint8_t ctrl_id :3;
	uint8_t mcs:5;
} MacDLMCSInfo;

typedef struct {
	uint8_t ctrl_id :3;
	uint8_t mcs:5;
} MacULMCSInfo;

typedef struct {
	uint16_t ctrl_id :3;
	uint16_t timingAdvance :13;
} MacTimingAdvance;

typedef struct {
	uint16_t ctrl_id :3;
	uint16_t fragment :1;
	uint16_t data_length : 12;
	uint8_t first_byte;
} MacDLdata;

typedef struct {
	uint16_t ctrl_id :3;
	uint16_t packetqueuesize :13;
} MacULreq;

#endif /* MAC_MAC_MESSAGES_H_ */
