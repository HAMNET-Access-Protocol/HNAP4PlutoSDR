/*
 * mac_common.c
 *
 *  Created on: Jan 14, 2020
 *      Author: lukas
 *
 *	This file contains utility functions that are
 *	used both in Basestation and Client MAC layer
 */

#include "mac_common.h"

#include "../util/ringbuf.h"
#include "mac_channels.h"
#include "mac_messages.h"

MacDataFrame dataframe_create(uint size)
{
	MacDataFrame frame = malloc(sizeof(MacDataFrame_s));
	frame->data = malloc(size);
	frame->size = size;
	return frame;
}

void dataframe_destroy(MacDataFrame frame)
{
	free(frame->data);
	free(frame);
}

// Check how many slots are assigned to the given userid
int num_slot_assigned(uint8_t* assignments, uint num_slots, uint8_t userid)
{
	int num_assigned = 0;
	for (int i=0; i<num_slots; i++) {
		if (assignments[i]==userid) {
			num_assigned++;
		}
	}
	return num_assigned;
}

// Try to dequeue all messages from a message ringbuf and
// add them to the given logical channel
void lchan_add_all_msgs(LogicalChannel lchan, ringbuf ctrl_msg_buf)
{
	// TODO: improve check whether there is enough space to add msg. Msg size currently hardcoded
	while (!ringbuf_isempty(ctrl_msg_buf) && lchan_unused_bytes(lchan)>1) {
		MacMessage msg = ringbuf_get(ctrl_msg_buf);
		lchan_add_message(lchan,msg);
		mac_msg_destroy(msg);
	}
}
