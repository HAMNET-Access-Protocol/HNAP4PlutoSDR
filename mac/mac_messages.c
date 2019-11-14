/*
 * mac_messages.c
 *
 *  Created on: Dec 9, 2019
 *      Author: lukas
 */

#include "mac_messages.h"

void mac_create_ul_req(uint8_t* buf, uint PacketQueueSize) {
	MacULreq* msg = (MacULreq*)buf;

	msg->ctrl_id = ul_req;
	msg->packetqueuesize = PacketQueueSize;
}
