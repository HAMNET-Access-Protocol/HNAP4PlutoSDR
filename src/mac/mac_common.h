/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef MAC_MAC_COMMON_H_
#define MAC_MAC_COMMON_H_

#include "../util/ringbuf.h"
#include <stddef.h>
#include <time.h>

#include "../util/log.h"
#include "mac_channels.h"

// log makro to log with subframe number
#define LOG_SFN_MAC(level, ...)                                                \
  do {                                                                         \
    if (level >= global_log_level) {                                           \
      printf("[%2d %2d]", mac->phy->common->rx_subframe,                       \
             mac->phy->common->rx_symbol);                                     \
      printf(__VA_ARGS__);                                                     \
    }                                                                          \
  } while (0);

// Define generic Dataframe
// This object is used for interaction with higher layers
typedef struct {
  uint size;
  uint8_t *data;
  uint do_arq;
} MacDataFrame_s;

// Store some MAC layer statistics
typedef struct {
  uint chan_rx_succ;
  uint chan_rx_fail;
  uint bytes_rx;
  uint bytes_tx;
  time_t association_time;
} MACstat_s;

typedef MacDataFrame_s *MacDataFrame;

/************ Methods for Mac Dataframe *****************/
MacDataFrame dataframe_create(uint size);
void dataframe_destroy(MacDataFrame frame);

/*************** Various utility methods ****************/
int num_slot_assigned(uint8_t *assignments, uint num_slots, uint8_t userid);
void lchan_add_all_msgs(LogicalChannel lchan, ringbuf ctrl_msg_buf);

void mac_stats_init(MACstat_s *stats);
int mac_stats_print(char *buf, int buflen, MACstat_s *stats);

#endif /* MAC_MAC_COMMON_H_ */
