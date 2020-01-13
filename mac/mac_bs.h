/*
 * mac_bs.h
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_BS_H_
#define MAC_MAC_BS_H_

#include "mac_config.h"
#include "mac_fragmentation.h"
#include <util/ringbuf.h>
#include <liquid/liquid.h>


typedef struct {
	ofdmframesync fs;
	uint8_t userid;
	uint ul_queue;
	ringbuf msg_control_queue;
	MacFrag fragmenter;
	MacAssmbl reassembler;
}user_s;


typedef struct {
	ringbuf msg_broadcast_queue;
	user_s* UE[MAX_USER];

	uint8_t ul_ctrl_assignments[2];
	uint8_t ul_data_assignments[4];
	uint8_t dl_data_assignments[4];
} MacBS_s;

typedef MacBS_s* MacBS;

void mac_bs_add_new_ue();

#endif /* MAC_MAC_BS_H_ */
