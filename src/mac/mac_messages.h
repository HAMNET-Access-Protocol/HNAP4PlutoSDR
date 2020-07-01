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

#ifndef MAC_MAC_MESSAGES_H_
#define MAC_MAC_MESSAGES_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// MAC Protocol version
#define PROTO_VERSION 0

// lowest 3 bits of this number are equal to the control ID
// that is written to the message itself
// bit 4 indicates wether its a DL (0) or UL (1) message
// e.g. ul_req= 0b1001 -> 1 for UL and 0b001 is ul_req ctrl ID
typedef enum {
  msg_none = 0,
  associate_response = 1,
  dl_mcs_info,
  ul_mcs_info,
  timing_advance,
  session_end,
  dl_data = 7,
  ul_req = 9,
  channel_quality,
  keepalive,
  control_ack,
  mcs_chance_req,
  ul_data = 15
} CtrlID_e;

// Association response types
enum { assoc_resp_success = 0, assoc_resp_full };

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t userid : 4;
  uint32_t rachuserid : 4;
  uint32_t response : 3;
  uint32_t protoVersion : 2;
  uint32_t timing_advance : 8;
} MacAssociateResponse;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t mcs : 5;
} MacDLMCSInfo;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t mcs : 5;
} MacULMCSInfo;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t timingAdvance : 13;
} MacTimingAdvance;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t data_length : 12;
  uint32_t final_flag : 1;
  uint32_t seqNr : 3;
  uint32_t fragNr : 5;
} MacDLdata;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t packetqueuesize : 13;
} MacULreq;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t channel_quality : 5; // TODO to be defined
} MacChannelQuality;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t reserved : 5;
} MacKeepalive;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t acked_ctrl_id : 3;
} MacControlAck;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t ul_flag : 1;
  uint32_t mcs : 4;
} MacMCSChangeReq;

typedef struct {
  uint32_t ctrl_id : 3;
  uint32_t data_length : 12;
  uint32_t final_flag : 1;
  uint32_t seqNr : 3;
  uint32_t fragNr : 5;
} MacULdata;

// Generic struct for Mac Message exchange between Modules
typedef struct {
  uint8_t hdr_bin[4]; // max header len fixed to 4 bytes. Can be changed
  union {
    uint8_t ctrl_id : 3;
    MacAssociateResponse AssociateResponse;
    MacDLMCSInfo DLMCSInfo;
    MacULMCSInfo ULMCSInfo;
    MacTimingAdvance TimingAdvance;
    MacDLdata DLdata;
    MacULreq ULreq;
    MacChannelQuality ChannelQuality;
    MacKeepalive Keepalive;
    MacControlAck ControlAck;
    MacMCSChangeReq MCSChangeReq;
    MacULdata ULdata;
  } hdr;
  CtrlID_e type;
  uint8_t hdr_len;
  uint16_t payload_len;
  uint8_t *data;

} MacMessage_s;

typedef MacMessage_s *MacMessage;

//// Utility functions ////
int mac_msg_get_hdrlen(CtrlID_e type);

//// Functions for creating/destroying MAC messages ////
// Downlink
MacMessage mac_msg_create_associate_response(uint userID, uint rachUserID,
                                             uint response,
                                             uint timing_advance);
MacMessage mac_msg_create_dl_mcs_info(uint mcs);
MacMessage mac_msg_create_ul_mcs_info(uint mcs);
MacMessage mac_msg_create_timing_advance(uint timingAdvance);
MacMessage mac_msg_create_session_end();
MacMessage mac_msg_create_dl_data(uint data_length, uint8_t fragment,
                                  uint8_t seqNr, uint8_t fragNr, uint8_t *data);
// Uplink
MacMessage mac_msg_create_ul_req(uint PacketQueueSize);
MacMessage mac_msg_create_channel_quality(uint quality_idx);
MacMessage mac_msg_create_keepalive();
MacMessage mac_msg_create_control_ack(uint acked_ctrl_id);
MacMessage mac_msg_create_mcs_change_req(uint is_ul, uint mcs);
MacMessage mac_msg_create_ul_data(uint data_length, uint8_t final,
                                  uint8_t seqNr, uint8_t fragNr, uint8_t *data);

void mac_msg_destroy(MacMessage genericmsg);

//// Functions to write/parse messages to/from buffers ////
int mac_msg_generate(MacMessage genericmsg, uint8_t *buf, uint buflen);
MacMessage mac_msg_parse(uint8_t *buf, uint buflen, uint8_t ul_flag);

#endif /* MAC_MAC_MESSAGES_H_ */
