/*
 * mac_common.h
 *
 *  Created on: Jan 15, 2020
 *      Author: lukas
 */

#ifndef MAC_MAC_COMMON_H_
#define MAC_MAC_COMMON_H_

#include <stddef.h>
#include "../util/ringbuf.h"

#include "mac_channels.h"


// Define generic Dataframe
// This object is used for interaction with higher layers
typedef struct {
	uint size;
	uint8_t* data;
} MacDataFrame_s;

// Store some MAC layer statistics
typedef struct {
	uint chan_rx_succ;
	uint chan_rx_fail;
	uint bytes_rx;
	uint bytes_tx;
} MACstat_s;

typedef MacDataFrame_s* MacDataFrame;

/************ Methods for Mac Dataframe *****************/
MacDataFrame dataframe_create(uint size);
void dataframe_destroy(MacDataFrame frame);

/*************** Various utility methods ****************/
int num_slot_assigned(uint8_t* assignments, uint num_slots, uint8_t userid);
void lchan_add_all_msgs(LogicalChannel lchan, ringbuf ctrl_msg_buf);

#endif /* MAC_MAC_COMMON_H_ */
