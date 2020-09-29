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

#include "mac_bs.h"
#include "../util/log.h"
#include <net/ethernet.h>
#include <netinet/ip.h>
#include <unistd.h>

#ifdef MAC_TEST_DELAY
#include "../runtime/test.h"
#endif

user_s *ue_create(uint userid, long long unsigned int *subframe_ptr) {
  // create user instance and association response
  user_s *new_ue = calloc(sizeof(user_s), 1);
  new_ue->msg_control_queue = ringbuf_create(MAC_CTRL_MSG_BUF_SIZE);
  new_ue->fragmenter = mac_frag_init(subframe_ptr);
  new_ue->reassembler = mac_assmbl_init();
  new_ue->userid = userid;
  new_ue->ul_queue = 0;
  new_ue->dl_mcs = 0;
  new_ue->ul_mcs = 0;
  new_ue->fs = NULL;

  // init stats struct
  mac_stats_init(&new_ue->stats);

  return new_ue;
}

void ue_destroy(user_s *ue) {
  while (!ringbuf_isempty(ue->msg_control_queue)) {
    MacMessage p = ringbuf_get(ue->msg_control_queue);
    mac_msg_destroy(p);
  }
  ringbuf_destroy(ue->msg_control_queue);
  mac_frag_destroy(ue->fragmenter);
  mac_assmbl_destroy(ue->reassembler);
  ofdmframesync_destroy(ue->fs);
  free(ue);
}

MacBS mac_bs_init() {
  MacBS macinst = calloc(sizeof(struct MacBS_s), 1);

  for (int i = 0; i < MAX_USER; i++) {
    macinst->UE[i] = NULL;
  }
  macinst->broadcast_ctrl_queue = ringbuf_create(MAC_CTRL_MSG_BUF_SIZE);
  macinst->broadcast_data_fragmenter = mac_frag_init(&macinst->subframe_cnt);

#ifdef MAC_ENABLE_TAP_DEV
  macinst->tapdevice = tap_init("tap0");
#endif

  SLIST_INIT(&macinst->etheraddr_map);

  macinst->last_added_rachuserid = -1;
  macinst->last_added_userid = -1;

  return macinst;
}

void mac_bs_destroy(MacBS mac) {
  for (int i = 0; i < MAX_USER; i++) {
    if (mac->UE[i] != NULL) {
      ue_destroy(mac->UE[i]);
    }
  }
  while (!ringbuf_isempty(mac->broadcast_ctrl_queue)) {
    MacMessage p = ringbuf_get(mac->broadcast_ctrl_queue);
    mac_msg_destroy(p);
  }
  ringbuf_destroy(mac->broadcast_ctrl_queue);
  mac_frag_destroy(mac->broadcast_data_fragmenter);

  while (!SLIST_EMPTY(&mac->etheraddr_map)) {
    struct entry *n1 = SLIST_FIRST(&mac->etheraddr_map);
    SLIST_REMOVE_HEAD(&mac->etheraddr_map, entries);
    free(n1);
  }
  free(mac);
}

void mac_bs_set_phy_interface(MacBS mac, struct PhyBS_s *phy) {
  mac->phy = phy;
}

void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, uint8_t rach_try_cnt,
                       ofdmframesync fs, int timing_diff) {
  MacMessage response = NULL;
  uint8_t userid = 0;

  // if this isnt the first time an association is tried,
  // check if this is the same user as the last one added
  if (rach_try_cnt > 0 && mac->last_added_rachuserid == rachuserid &&
      mac->UE[mac->last_added_userid] != NULL) {
    userid = mac->last_added_userid;
    mac->UE[userid]->timingadvance = timing_diff;
    response = mac_msg_create_associate_response(
        userid, rachuserid, assoc_resp_success, timing_diff);
    LOG(INFO, "[MAC BS] Double assoc req! rachuserid %d, user ID %d\n",
        rachuserid, userid);
    // Response will be sent via broadcast channel
    ringbuf_put(mac->broadcast_ctrl_queue, response);
  } else {
    // find the first unused userid
    while ((userid < MAX_USER) &&
           ((mac->UE[userid] != NULL) || (userid == USER_BROADCAST) ||
            (userid == USER_UNUSED))) {
      userid++;
    }
    if (userid == MAX_USER) {
      // no free userid. generate NAK
      LOG(WARN, "[MAC BS] add_new_ue: no free userID, cannot add user\n");
      SYSLOG(LOG_WARNING,
             "[MAC BS] add_new_ue: no free userID, cannot add user\n");
      response =
          mac_msg_create_associate_response(0, rachuserid, assoc_resp_full, 0);
      // Response will be sent via broadcast channel
      ringbuf_put(mac->broadcast_ctrl_queue, response);
    } else {
      // create new UE struct
      mac->UE[userid] = ue_create(userid, &mac->subframe_cnt);
      mac->UE[userid]->fs = fs;
      mac->UE[userid]->last_seen = mac->subframe_cnt;
      mac->UE[userid]->timingadvance = timing_diff;
      response = mac_msg_create_associate_response(
          userid, rachuserid, assoc_resp_success, timing_diff);
      LOG(INFO, "[MAC BS] add new user! rachuserid %d, new ID %d\n", rachuserid,
          userid);
      SYSLOG(LOG_INFO, "[MAC BS] add new user! rachuserid %d, new ID %d\n",
             rachuserid, userid);
      // Response will be sent via broadcast channel
      ringbuf_put(mac->broadcast_ctrl_queue, response);
      mac->last_added_userid = userid;
      mac->last_added_rachuserid = rachuserid;
    }
  }
}

// Change the DL/UL MCS scheme for a user
void mac_bs_set_mcs(MacBS mac, uint userid, uint mcs, uint dl_ul) {
  int ret = 0;
  if (userid >= MAX_USER || mac->UE[userid] == NULL) {
    LOG(ERR, "[MAC BS] cannot find user %d to set MCS", userid);
    return;
  }

  if (dl_ul == DL) {
    MacMessage msg = mac_msg_create_dl_mcs_info(mcs);
    ret = ringbuf_put(mac->UE[userid]->msg_control_queue, msg);
    if (ret) {
      mac->UE[userid]->dl_mcs_pending = mcs;
      mac->UE[userid]->dl_mcs_pending_time =
          mac->subframe_cnt + MAX_RESPONSE_TIME;
    }
  } else {
    MacMessage msg = mac_msg_create_ul_mcs_info(mcs);
    ret = ringbuf_put(mac->UE[userid]->msg_control_queue, msg);
    if (ret) {
      // For uplink, we immediately switch to new MCS, so that client can
      // send ACK using new mcs. If no ack is received we switch back to old mcs
      mac->UE[userid]->ul_mcs_pending = mac->UE[userid]->ul_mcs;
      mac->UE[userid]->ul_mcs = mcs;
      mac->UE[userid]->ul_mcs_pending_time =
          mac->subframe_cnt + MAX_RESPONSE_TIME;
    }
  }
}

// Try to get the receiver object for the given userid
// returns NULL if user does not exist
ofdmframesync mac_bs_get_receiver(MacBS mac, uint userid) {
  if (mac->UE[userid] != NULL) {
    return mac->UE[userid]->fs;
  } else {
    return NULL;
  }
}

int mac_bs_add_txdata(MacBS mac, uint8_t destUserID, MacDataFrame frame) {
  MacFrag fragmenter = NULL;
  if (destUserID == USER_BROADCAST) {
    fragmenter = mac->broadcast_data_fragmenter;
    // force frames that are forwarded using broadcast frag to disable ARQ
    frame->do_arq = 0;
  } else if (mac->UE[destUserID] != NULL) {
    fragmenter = mac->UE[destUserID]->fragmenter;
  } else {
    LOG(WARN, "[MAC BS] add_txdata: user %d does not exist!\n", destUserID);
    return 0;
  }

  uint ret = mac_frag_add_frame(fragmenter, frame);
  if (ret == 0) {
    LOG(WARN, "[MAC BS] add_txdata: msg queue is full. dropping packet!\n");
    return 0;
  }
  LOG(INFO, "[MAC BS] added txdata frame for user %d\n", destUserID);
  return 1;
}

// Set new timing advance parameter and generate a control message
// to inform the UE
void mac_bs_update_timingadvance(MacBS mac, uint userid, int timing_diff) {
  if (mac->UE[userid]) {
    mac->UE[userid]->timingadvance = timing_diff;
    MacMessage msg =
        mac_msg_create_timing_advance(mac->UE[userid]->timingadvance);
    ringbuf_put(mac->UE[userid]->msg_control_queue, msg);
  } else {
    LOG(ERR, "[MAC BS] cannot set TA for user %d. Does not exist\n", userid);
  }
}

// Handle the control ack message
void mac_bs_handle_control_ack(MacBS mac, MacMessage msg, user_s *ue) {
  switch (msg->hdr.ControlAck.acked_ctrl_id) {
  case dl_mcs_info:
    LOG(INFO, "[MAC BS] received ctrl ack for dl_mcs_info\n");
    ue->dl_mcs = ue->dl_mcs_pending;
    ue->dl_mcs_pending_time = 0;
    break;
  case ul_mcs_info:
    LOG(INFO, "[MAC BS] received ctrl ack for ul_mcs_info\n");
    ue->ul_mcs_pending_time = 0;
    // NOTE: mcs has already been switch. We just cancel the timer which reverts
    // the change
    break;
  default:
    LOG(ERR, "[MAC BS] cannot handle control ack for ctrlID %d\n",
        msg->hdr.ControlAck.acked_ctrl_id);
  }
}

// Handle incoming messages from PHY layer
int mac_bs_handle_message(MacBS mac, MacMessage msg, uint8_t userID) {

  user_s *user = mac->UE[userID];
  if (user == NULL) {
    mac_msg_destroy(msg);
    LOG_SFN_MAC(WARN,
                "[MAC BS] received message for unknown user %d. Dropping.\n",
                userID);
    return 0;
  }
  MacDataFrame frame;
  switch (msg->type) {
  case ul_req:
    // Update the user uplink queue state
    user->ul_queue = msg->hdr.ULreq.packetqueuesize;
    LOG_SFN_MAC(INFO, "[MAC BS] ul_req from user %d. Queuesize: %d\n", userID,
                user->ul_queue);
    break;
  case channel_quality:
    LOG_SFN_MAC(DEBUG,
                "[MAC BS] channel quality measurement not implemented yet\n");
    break;
  case keepalive:
    LOG_SFN_MAC(DEBUG, "[MAC BS] keepalive from user %d\n", userID);
    break;
  case control_ack:
    mac_bs_handle_control_ack(mac, msg, user);
    break;
  case mcs_chance_req:
    mac_bs_set_mcs(mac, userID, msg->hdr.MCSChangeReq.mcs,
                   msg->hdr.MCSChangeReq.ul_flag);
    LOG_SFN_MAC(
        INFO, "[MAC BS] mcs_change_request from user %d mcs: %d is_ul %d\n",
        userID, msg->hdr.MCSChangeReq.mcs, msg->hdr.MCSChangeReq.ul_flag)
    break;
  case dl_data_ack:
    mac_frag_ack_fragment(user->fragmenter, msg);
    break;
  case ul_data:
    if (msg->hdr.ULdata.do_ack) {
      MacMessage ack = mac_msg_create_ul_data_ack(ACK, msg->hdr.ULdata.seqNr,
                                                  msg->hdr.ULdata.fragNr);
      ringbuf_put(user->msg_control_queue, ack);
    }
    frame = mac_assmbl_reassemble(user->reassembler, msg);
    if (frame != NULL) {
      user->stats.bytes_rx += frame->size;
      LOG_SFN_MAC(INFO, "[MAC BS] received frame with %d bytes!\n",
                  frame->size);
      // PRINT_BIN(INFO,frame->data,frame->size); LOG(INFO,"\n");
#ifdef MAC_ENABLE_TAP_DEV
      // check if source EtherAddr is already included in our address mapping
      struct entry *np = NULL;
      char *EtherSrcAddr = frame->data + 6;
      SLIST_FOREACH(np, &mac->etheraddr_map, entries) {
        if (strncmp(np->EtherAddr, EtherSrcAddr, 6) == 0)
          break;
      }
      if (np == NULL) {
        // EtherAddr not yet in list, add it
        np = malloc(sizeof(struct entry));
        strncpy(np->EtherAddr, EtherSrcAddr, 6);
        np->userid = userID;
        SLIST_INSERT_HEAD(&mac->etheraddr_map, np, entries);
        LOG(INFO,
            "[MAC] Add EtherAddr: %02x:%02x:%02x:%02x:%02x:%02x -> userid %d\n",
            EtherSrcAddr[0], EtherSrcAddr[1], EtherSrcAddr[2], EtherSrcAddr[3],
            EtherSrcAddr[4], EtherSrcAddr[5], userID);
      }
      tap_send(mac->tapdevice, frame->data, frame->size);
#endif

#ifdef MAC_TEST_DELAY
      uint sfn;
      memcpy(&sfn, frame->data, sizeof(uint));
      if (sfn < num_simulated_subframes)
        mac_ul_timestamps[sfn] += global_sfn * SUBFRAME_LEN + global_symbol;
#endif
      dataframe_destroy(frame);
    }
    break;
  default:
    LOG_SFN_MAC(WARN, "[MAC BS] unexpected MacMsg ID: %d\n", msg->type);
    mac_msg_destroy(msg);
    return 0;
  }
  mac_msg_destroy(msg);
  return 1;
}

// Main interface function that is called from PHY when receiving a
// logical channel. Function will extract messages and call
// the message handler
int mac_bs_rx_channel(MacBS mac, LogicalChannel chan, uint userid) {
  // Verify the CRC
  if (!lchan_verify_crc(chan)) {
    LOG_SFN_MAC(
        WARN, "[MAC BS] lchan CRC%d invalid. Dropping %d bytes from user %d\n",
        8 * chan->crc_type, chan->payload_len, userid);
    mac->UE[userid]->stats.chan_rx_fail++;
    lchan_destroy(chan);
    return 0;
  }

  MacMessage msg = NULL;
  // Get all messages from the logical channel and handle them
  do {
    msg = lchan_parse_next_msg(chan, 1);
    if (msg) {
      mac_bs_handle_message(mac, msg, userid);
    }
  } while (msg != NULL);

  lchan_destroy(chan);
  mac->UE[userid]->stats.chan_rx_succ++;
  mac->UE[userid]->last_seen = mac->subframe_cnt;
  return 1;
}

user_s *get_next_user(MacBS mac, uint curr_user) {
  curr_user = curr_user % MAX_USER;
  uint next_user = curr_user;
  do {
    next_user = (next_user + 1) % MAX_USER;
  } while (mac->UE[next_user] == NULL && curr_user != next_user);
  return mac->UE[next_user];
}

int ue_has_dldata(user_s *ue) {
  return (mac_frag_has_fragment(ue->fragmenter) ||
          !ringbuf_isempty(ue->msg_control_queue));
}

void mac_bs_map_slot(MacBS mac, uint subframe, uint slot, user_s *ue) {
  // Generate logical channel
  uint tbs = get_tbs_size(mac->phy->common, ue->dl_mcs);
  LogicalChannel chan = lchan_create(tbs / 8, CRC16);
  lchan_add_all_msgs(chan, ue->msg_control_queue);
  if (mac_frag_has_fragment(ue->fragmenter)) {
    uint payload_size = lchan_unused_bytes(chan);
    MacMessage msg = mac_frag_get_fragment(ue->fragmenter, payload_size, 0);
    lchan_add_message(chan, msg);
    ue->stats.bytes_tx += msg->payload_len;
    if (msg->hdr.DLdata.do_ack) {
      // if the user is expected to send an ack for this frame, indicade a
      // pending transmission to the scheduler
      ue->ul_queue++;
    }
    mac_msg_destroy(msg);
  }
  lchan_calc_crc(chan);
  phy_map_dlslot(mac->phy, chan, subframe % 2, slot, ue->userid, ue->dl_mcs);
  lchan_destroy(chan);
}

// Find users which did not answer to any slot assignments
// for some time and start session_end procedure
void mac_bs_detect_inactive_users(MacBS mac) {
  for (int userid = 0; userid < MAX_USER; userid++) {
    if (mac->UE[userid] != NULL && userid != USER_BROADCAST) {
      if (mac->UE[userid]->last_seen + TMR_USER_INACTIVE < mac->subframe_cnt) {
        MacMessage msg = mac_msg_create_session_end();
        if (!ringbuf_put(mac->UE[userid]->msg_control_queue, msg))
          mac_msg_destroy(msg);
        // set the flag to end this session
        mac->UE[userid]->will_end = 1;
        LOG(WARN, "[MAC BS] user %d is unresponsive! End connection\n", userid);
      }
    }
  }
}

// check if session end started and remove user all remaining data
// has been sent
void mac_bs_remove_inactive_users(MacBS mac) {
  for (int userid = 0; userid < MAX_USER; userid++) {
    if (mac->UE[userid] != NULL && userid != USER_BROADCAST) {
      if (mac->UE[userid]->will_end &&
          ringbuf_isempty(mac->UE[userid]->msg_control_queue)) {
        user_s *ue = mac->UE[userid];
        mac->UE[userid] = NULL;
        ue_destroy(ue);

        // remove entries from etheraddr_map belonging to userid
        struct entry *np = SLIST_FIRST(&mac->etheraddr_map);
        struct entry *np_next = NULL;
        while (np != NULL) {
          np_next = SLIST_NEXT(np, entries);
          if (np->userid == userid) {
            SLIST_REMOVE(&mac->etheraddr_map, np, entry, entries);
            free(np);
          }
          np = np_next;
        }
      }
    }
  }
}

// Checks whether there are pending or expired MCS changes
void mac_bs_process_mcs_change(MacBS mac) {
  for (int userid = 0; userid < MAX_USER - 1; userid++) {
    user_s *ue = mac->UE[userid];
    if (ue != NULL) {
      if (ue->dl_mcs_pending_time > 0 &&
          ue->dl_mcs_pending_time >= mac->subframe_cnt) {
        // MCS change pending and unacknowledged
        // resend mcs_info message
        MacMessage msg = mac_msg_create_dl_mcs_info(ue->dl_mcs_pending);
        if (!ringbuf_put(ue->msg_control_queue, msg))
          mac_msg_destroy(msg);
      }
      if (ue->dl_mcs_pending_time > 0 &&
          ue->dl_mcs_pending_time < mac->subframe_cnt) {
        // MCS change expired without any ack. Cancel it
        ue->dl_mcs_pending_time = 0;
      }

      if (ue->ul_mcs_pending_time > 0 &&
          ue->ul_mcs_pending_time >= mac->subframe_cnt) {
        // MCS change pending and unacknowledged
        // resend mcs_info message. Note: use ul_mcs since
        // for uplink we switch mcs immediately
        MacMessage msg = mac_msg_create_ul_mcs_info(ue->ul_mcs);
        if (!ringbuf_put(ue->msg_control_queue, msg))
          mac_msg_destroy(msg);
      }
      if (ue->ul_mcs_pending_time > 0 &&
          ue->ul_mcs_pending_time < mac->subframe_cnt) {
        // MCS change expired without any ack. Cancel it
        ue->ul_mcs_pending_time = 0;
        // switch back to old mcs
        ue->ul_mcs = ue->ul_mcs_pending;
      }
    }
  }
}

// Check whether the slot that is about to be assigned is colliding with
// a slot in the other link direction. Need to perform this check since
// clients are only half-duplex
// NOTE: we assume that the scheduler first assignes DL slots in a subframe
// Returns: 1 if there is a overlap, otherwise 0
int dl_ul_overlap_check(MacBS mac, uint userid, int subframe, int slotnr,
                        int is_dl) {
  if (is_dl) {
    // ensure that the user is not already mapped to a UL slot at the same time
    // from previous scheduler iteration
    if (slotnr < 2 &&
        userid == mac->ul_data_assignments[(uint)(subframe - 1) % FRAME_LEN]
                                          [slotnr + 2]) {
      return 1; // there is an overlap
    }
  } else { // is uplink
    // ensure that the user is not already mapped to a DL slot at the same time
    // from current scheduler iteration
    if (slotnr < 2 &&
        (userid == mac->dl_data_assignments[subframe][slotnr + 2] ||
         USER_BROADCAST == mac->dl_data_assignments[subframe][slotnr + 2])) {
      return 1; // there is an overlap
    }
    // ensure that UL slot does not overlap with Sync slot
    if (subframe == 0 && slotnr == 1)
      return 1;
  }
  // ensure that the user was not mapped to a ULCTRL slot in previous subframe
  // user wont be able to decode DLCTRL slot in this case, to he cannot be
  // assigned at all
  if (userid == mac->ul_ctrl_assignments[(uint)(subframe - 1) % FRAME_LEN][0] ||
      userid == mac->ul_ctrl_assignments[(uint)(subframe - 1) % FRAME_LEN][1]) {
    return 1; // there is an overlap
  }
  return 0; // no overlap
}

void mac_bs_run_scheduler(MacBS mac) {
  uint slot_idx = 0;
  uint user_id = 0;
  uint next_sfn;
  user_s *ue = NULL;

  LOG(TRACE, "[MAC BS] run scheduler\n");

  // Run MAC procedures
  mac_bs_process_mcs_change(mac);

  // Run unresponsive user detection
  mac_bs_detect_inactive_users(mac);

  if (mac->phy->common->tx_symbol == 0) {
    // subframe just started. schedule for this one.
    // TODO set rules when the scheduler should run
    next_sfn = mac->phy->common->tx_subframe;
  } else {
    next_sfn = (mac->phy->common->tx_subframe + 1) % FRAME_LEN;
  }

  // assure that the sync slot is not assigned for user traffic
  uint available_slots =
      (next_sfn == 0) ? (MAC_DLDATA_SLOTS - 1) : MAC_DLDATA_SLOTS;

  // 1. Assign UL ctrl slots
  // Every active user gets an assignment every 8th subframe
  uint id = mac->phy->common->tx_subframe * 2;
  mac->ul_ctrl_assignments[next_sfn][0] = mac->UE[id] != NULL ? id : 0;
  mac->ul_ctrl_assignments[next_sfn][1] = mac->UE[id + 1] != NULL ? id + 1 : 0;

  // 2. DL mapping: start by disabling all slots
  for (int i = slot_idx; i < MAC_DLDATA_SLOTS; i++) {
    mac->dl_data_assignments[next_sfn][i] = USER_UNUSED;
  }
  // 2.1 check Broadcast queue
  // We allocate max 1 slot per subframe for broadcasting with priority
  // over unicast traffic
  // Map possible broadcast slot to the end of the subframe, since we can be
  // sure that there is no user assigned to a colliding UL slot yet
  if (mac_frag_has_fragment(mac->broadcast_data_fragmenter) ||
      !ringbuf_isempty(mac->broadcast_ctrl_queue)) {
    // Generate logical channel
    uint tbs = get_tbs_size(mac->phy->common, 0);
    LogicalChannel chan = lchan_create(tbs / 8, CRC16);
    lchan_add_all_msgs(chan, mac->broadcast_ctrl_queue);
    if (mac_frag_has_fragment(mac->broadcast_data_fragmenter)) {
      uint payload_size = lchan_unused_bytes(chan);
      MacMessage msg = mac_frag_get_fragment(mac->broadcast_data_fragmenter,
                                             payload_size, 0);
      lchan_add_message(chan, msg);
      mac_msg_destroy(msg);
    }
    lchan_calc_crc(chan);
    phy_map_dlslot(mac->phy, chan, next_sfn % 2, available_slots - 1,
                   USER_BROADCAST, 0);
    lchan_destroy(chan);
    mac->dl_data_assignments[next_sfn][available_slots - 1] = USER_BROADCAST;
    available_slots--;
  }

  // 2.2. iterate over all remaining DL slots and assign it to the users
  // assign slots to active users
  // get first active user
  // TODO if there is much traffic, users with high userid do not get
  // assignments. Better do round robin over multiple subframes
  ue = get_next_user(mac, 0);
  if (ue != NULL) {
    user_id = ue->userid;
  }

  while (slot_idx < available_slots) {
    if (ue == NULL) {
      // no active user at all. stop
      break;
    }
    // get next user. Round robin allocation
    ue = get_next_user(mac, ue->userid);

    // check whether the user has DL data or DL ctrl data and we can assign it
    if (ue_has_dldata(ue) &&
        !dl_ul_overlap_check(mac, ue->userid, next_sfn, slot_idx, 1)) {
      mac_bs_map_slot(mac, next_sfn, slot_idx, ue);
      mac->dl_data_assignments[next_sfn][slot_idx++] = ue->userid;
      user_id = ue->userid; // update last active user
    } else if (ue->userid == user_id) {
      // no active that can be mapped was found. try next slot
      mac->dl_data_assignments[next_sfn][slot_idx++] = USER_UNUSED;
    }
  }

  // 3. iterate over each UL slot and assign it
  // start by disabling all slots
  for (int i = 0; i < MAC_ULDATA_SLOTS; i++) {
    mac->ul_data_assignments[next_sfn][i] = USER_UNUSED;
  }
  slot_idx = 0;
  // get first active user
  ue = get_next_user(mac, 0);
  if (ue != NULL) {
    user_id = ue->userid;
  }

  // force RA slot to be not assigned. (Slot 3 in subframe 0)
  available_slots = (next_sfn == 0) ? (MAC_ULDATA_SLOTS - 1) : MAC_ULDATA_SLOTS;

  while (slot_idx < available_slots) {
    if (ue == NULL) {
      // no active user at all. stop
      break;
    }
    // get next user. Round robin allocation
    ue = get_next_user(mac, ue->userid);

    // check whether the user has pending ul data
    if (ue->ul_queue > 0 &&
        !dl_ul_overlap_check(mac, ue->userid, next_sfn, slot_idx, 0)) {
      mac->ul_data_assignments[next_sfn][slot_idx++] = ue->userid;
      user_id = ue->userid; // update last active user
      // update ul queue len:
      ue->ul_queue -= get_tbs_size(mac->phy->common, ue->ul_mcs) / 8 - 5;
      if (ue->ul_queue < 0)
        ue->ul_queue = 0;
    } else if (ue->userid == user_id) {
      // no user has data / can be assigned to this slot. Try next
      mac->ul_data_assignments[next_sfn][slot_idx++] = USER_UNUSED;
    }
  }

  // 4. set slot assignments in PHY
  phy_assign_dlctrl_dd(mac->phy, next_sfn % 2,
                       mac->dl_data_assignments[next_sfn]);
  phy_assign_dlctrl_ud(mac->phy, next_sfn % 2,
                       mac->ul_data_assignments[next_sfn]);
  phy_assign_dlctrl_uc(mac->phy, next_sfn % 2,
                       mac->ul_ctrl_assignments[next_sfn]);
  // write the Downlink control channel to the subcarriers
  phy_map_dlctrl(mac->phy, next_sfn % 2);

  // Log schedule
  LOG(TRACE, "[MAC BS] Scheduler user assignments for subframe %d:\n",
      next_sfn);
  LOG(TRACE, "         DL data slots: %4d %4d %4d %4d\n",
      mac->dl_data_assignments[next_sfn][0],
      mac->dl_data_assignments[next_sfn][1],
      mac->dl_data_assignments[next_sfn][2],
      mac->dl_data_assignments[next_sfn][3]);
  LOG(TRACE, "         UL data slots: %4d %4d %4d %4d\n",
      mac->ul_data_assignments[next_sfn][0],
      mac->ul_data_assignments[next_sfn][1],
      mac->ul_data_assignments[next_sfn][2],
      mac->ul_data_assignments[next_sfn][3]);
  LOG(TRACE, "         UL ctrl slots: %4d %4d\n",
      mac->ul_ctrl_assignments[next_sfn][0],
      mac->ul_ctrl_assignments[next_sfn][1]);

  // Remove inactive users
  mac_bs_remove_inactive_users(mac);

  // update mac subframe counter
  // TODO: let phy handle this? What if scheduler is not called
  mac->subframe_cnt++;
}

void *mac_bs_tap_rx_th(void *arg) {
  MacBS mac = (MacBS)arg;
  tap_dev dev = mac->tapdevice;

  // wait until tap device is created
  while (dev == NULL) {
    usleep(10000);
  }
  LOG(INFO, "[MAC/TAP] start TAP thread\n");
  while (1) {
    tap_receive(dev);

    if (mac->tapdevice->bytes_rec > 0) {
      MacDataFrame frame = dataframe_create(dev->bytes_rec);
      memcpy(frame->data, dev->buffer, dev->bytes_rec);

      // Packet inspection: determine if this is TCP traffic
      // then activate ARQ
      uint16_t ether_type = (frame->data[12] << 8) + frame->data[13];
      struct iphdr *ip4hdr = (struct iphdr *)&frame->data[14];
      frame->do_arq = 0; // no ARQ by default
      LOG(DEBUG, "[TAP] Packet inspect: ethertype %04x, proto %d\n", ether_type,
          ip4hdr->protocol);
      if (ether_type == ETHERTYPE_IP) {
        // is IPv4 packet, check if TCP
        if (ip4hdr->protocol == 6) {
          frame->do_arq = 1;
        }
      }
      // find correct userid to forward EtherFrame to
      // if no entry is found, broadcast channel is used
      struct entry *np = NULL;
      uint userid = USER_BROADCAST;
      SLIST_FOREACH(np, &mac->etheraddr_map, entries) {
        if (strncmp(frame->data, np->EtherAddr, 6) == 0) {
          userid = np->userid;
          break;
        }
      }
      if (!mac_bs_add_txdata(mac, userid, frame)) {
        dataframe_destroy(frame);
        LOG(ERR, "[MAC BS] could not add Ether frame to MAC\n");
      }
    }
  }
  return NULL;
}
