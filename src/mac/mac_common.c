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
 *
 *
 * This file contains utility functions that are
 * used both in Basestation and Client MAC layer
 */

#include "mac_common.h"

#include "../util/ringbuf.h"
#include "mac_channels.h"
#include "mac_messages.h"

#include <net/ethernet.h>
#include <netinet/ip.h>

MacDataFrame dataframe_create(uint size) {
  MacDataFrame frame = malloc(sizeof(MacDataFrame_s));
  frame->data = malloc(size);
  frame->size = size;
  frame->do_arq = 0; // do not use ARQ mode by default
  return frame;
}

void dataframe_destroy(MacDataFrame frame) {
  free(frame->data);
  free(frame);
}

// Check how many slots are assigned to the given userid
int num_slot_assigned(uint8_t *assignments, uint num_slots, uint8_t userid) {
  int num_assigned = 0;
  for (int i = 0; i < num_slots; i++) {
    if (assignments[i] == userid) {
      num_assigned++;
    }
  }
  return num_assigned;
}

// Try to dequeue all messages from a message ringbuf and
// add them to the given logical channel
void lchan_add_all_msgs(LogicalChannel lchan, ringbuf ctrl_msg_buf) {
  // TODO: improve check whether there is enough space to add msg. Msg size
  // currently hardcoded
  while (!ringbuf_isempty(ctrl_msg_buf) && lchan_unused_bytes(lchan) > 1) {
    MacMessage msg = ringbuf_get(ctrl_msg_buf);
    lchan_add_message(lchan, msg);
    mac_msg_destroy(msg);
  }
}

// initialize the mac statistics struct
void mac_stats_init(MACstat_s *stats) {
  stats->association_time = time(NULL);
  stats->bytes_tx = 0;
  stats->bytes_rx = 0;
  stats->chan_rx_fail = 0;
  stats->chan_rx_succ = 0;
}

// Print current mac statistics to the given buffer
int mac_stats_print(char *buf, int buflen, MACstat_s *stats) {
  time_t uptime = difftime(time(NULL), stats->association_time);
  uint up_days = uptime / (3600 * 24);
  uint up_hours = (uptime % (3600 * 24)) / 3600;
  uint up_min = (uptime % (3600)) / 60;
  uint up_secs = (uptime % 60);
  return snprintf(buf, buflen,
                  "Uptime: %3d days %02d:%02d:%02d hours\n"
                  "RX frame succ/fail: %5d/%d\n"
                  "RX bytes: %6d   TX bytes: %6d\n",
                  up_days, up_hours, up_min, up_secs, stats->chan_rx_succ,
                  stats->chan_rx_fail, stats->bytes_rx, stats->bytes_tx);
}

int packet_inspect_is_tcpip(uint8_t *buf, uint buflen) {
  // Packet inspection: determine if this is TCP traffic
  // then activate ARQ

  if (buflen < 34) {
    return -1; // buffersize is shorter than Ethernet+IP header size
  }

  uint16_t ether_type = (buf[12] << 8) + buf[13];
  struct iphdr *ip4hdr = (struct iphdr *)&buf[14];
  LOG(DEBUG, "[MAC] Packet inspect: ethertype %04x, proto %d\n", ether_type,
      ip4hdr->protocol);
  if (ether_type == ETHERTYPE_IP) {
    // is IPv4 packet, check if TCP
    if (ip4hdr->protocol == 6) {
      return 1; // This is a IPv4 header with TCP content
    }
  }
  return 0; // this frame does not contain TCP/IP
}