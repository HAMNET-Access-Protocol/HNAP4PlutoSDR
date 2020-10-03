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

#include "mac_fragmentation.h"

#include "mac_config.h"
#include "mac_messages.h"
#include <ringbuf.h>

#define MAX_SEQNR 16          // 4 bits are allocated for seqNr in MacMessage
#define MAX_FRAGNR 32         // 5 bits are allocated for fragNr in MacMessage
#define MAX_FRAGMENT_SIZE 512 // max size per fragment, currently static

#define ARQ_WINDOW_LEN 15 // arq window len must be smaller than MAX_SEQNR
#define ARQ_ACK_TIMEOUT                                                        \
  15 // number of subframes until the sender assumes a timeout for
     // the ACK. 1 subframe = 17ms
#define ARQ_MAX_RETRANSMITS 4 // maximum number of retransmits per fragment

struct MacFragmenter_s {
  uint um_seqNr;
  uint um_fragNr;
  uint am_seqNr;
  uint am_fragNr;
  MacDataFrame curr_frame;
  ringbuf frame_queue;
  uint bytes_sent;
  uint bytes_buffered;

  MacMessage am_send_window[ARQ_WINDOW_LEN];
  uint8_t am_window_retransmits[ARQ_WINDOW_LEN];
  long long unsigned int am_window_timestamp[ARQ_WINDOW_LEN];
  uint8_t am_last_ack_idx;
  uint8_t am_window_idx;
  uint am_window_bytes_buffered;

  long long unsigned int
      *subframe; // pointer to subframe counter of main MAC instance
};

struct MacReassembler_um_s {
  uint frame_open;
  uint seqNr;
  uint fragNr;
  uint8_t *fragments[MAX_FRAGNR];
  uint fragments_len[MAX_FRAGNR];
  uint frame_len;
};

struct MacReassembler_am_s {
  uint8_t seq_got_final[MAX_SEQNR];
  uint8_t num_fragments[MAX_SEQNR];
  uint8_t frag_rcv_bitmask[MAX_SEQNR][MAX_FRAGNR];
  uint8_t *fragments[MAX_SEQNR][MAX_FRAGNR];
  uint fragments_len[MAX_SEQNR][MAX_FRAGNR];
};

typedef struct MacReassembler_um_s *MacAssmblUM;
typedef struct MacReassembler_am_s *MacAssmblAM;

struct MacReassembler_s {
  MacAssmblUM um_assmbl;
  MacAssmblAM am_assmbl;
};

void arq_window_put(MacFrag frag, MacMessage msg) {
  frag->am_send_window[frag->am_window_idx] = msg;
  frag->am_window_retransmits[frag->am_window_idx] = 0;
  frag->am_window_timestamp[frag->am_window_idx] = *frag->subframe;
  frag->am_window_idx = (frag->am_window_idx + 1) % ARQ_WINDOW_LEN;
  frag->am_window_bytes_buffered += msg->payload_len;
}

void arq_window_remove(MacFrag frag, int idx) {
  frag->am_window_retransmits[idx] = 0;
  frag->am_window_bytes_buffered -= frag->am_send_window[idx]->payload_len;
  mac_msg_destroy(frag->am_send_window[idx]);
  frag->am_send_window[idx] = NULL;
  frag->am_window_timestamp[idx] = 0;
  if (idx == frag->am_last_ack_idx) {
    // last element from window has been removed, find new last element
    while (frag->am_last_ack_idx != frag->am_window_idx &&
           frag->am_send_window[frag->am_last_ack_idx] == NULL)
      frag->am_last_ack_idx = (frag->am_last_ack_idx + 1) % ARQ_WINDOW_LEN;
  }
}

void arq_window_ack_msg(MacFrag frag, uint8_t acked_seqnr,
                        uint8_t acked_fragnr) {
  for (int idx = 0; idx < ARQ_WINDOW_LEN; idx++) {
    if (frag->am_send_window[idx] != NULL) {
      uint8_t seqnr = frag->am_send_window[idx]->hdr.DLdata.seqNr;
      uint8_t fragnr = frag->am_send_window[idx]->hdr.DLdata.fragNr;
      if (seqnr == acked_seqnr && fragnr == acked_fragnr) {
        arq_window_remove(frag, idx);
        return;
      }
    }
  }
}

int arq_window_isempty(MacFrag frag) {
  return (frag->am_last_ack_idx == frag->am_window_idx);
}

int arq_window_isfull(MacFrag frag) {
  return ((frag->am_window_idx + 1) % ARQ_WINDOW_LEN == frag->am_last_ack_idx);
}

MacFrag mac_frag_init(long long unsigned int *subframe_ptr) {
  MacFrag frag = calloc(1, sizeof(struct MacFragmenter_s));
  frag->frame_queue = ringbuf_create(MAC_DATA_BUF_SIZE);
  frag->curr_frame = NULL;
  frag->subframe = subframe_ptr;
  return frag;
}

void mac_frag_destroy(MacFrag frag) {
  while (!ringbuf_isempty(frag->frame_queue)) {
    MacDataFrame p = ringbuf_get(frag->frame_queue);
    dataframe_destroy(p);
  }
  ringbuf_destroy(frag->frame_queue);
  free(frag->curr_frame);
  free(frag);
}

int mac_frag_add_frame(MacFrag frag, MacDataFrame frame) {
  if (frame->size > MAC_MTU) {
    LOG(WARN, "[MAC FRAG] incoming frame size exceeds MTU! %d bytes\n",
        frame->size);
    return 0;
  }
  if (ringbuf_isfull(frag->frame_queue)) {
    LOG(WARN, "[MAC FRAG] cannot enqueue frame. queue full\n");
    return 0;
  } else {
    ringbuf_put(frag->frame_queue, frame);
    frag->bytes_buffered += frame->size;
    return 1;
  }
}

int mac_frag_has_fragment(MacFrag frag) {
  // check if arq window is full. We cannot
  // send further data in this case
  if (arq_window_isfull(frag)) {
    return 0;
  }
  // Check if there is any frame buffered that can be sent.
  if (!ringbuf_isempty(frag->frame_queue) || (frag->curr_frame != NULL)) {
    return 1;
  }
  // Check if there is any retransmission that can be made
  for (int i = 0; i < ARQ_WINDOW_LEN; i++) {
    int idx = (frag->am_last_ack_idx + i) % ARQ_WINDOW_LEN;
    if (frag->am_send_window[idx] != NULL) {
      if (frag->am_window_timestamp[idx] + ARQ_ACK_TIMEOUT < *frag->subframe) {
        return 1;
      }
    }
  }

  // there is nothing to send
  return 0;
}

int mac_frag_queue_full(MacFrag frag) {
  return ringbuf_isfull(frag->frame_queue);
}

int mac_frag_get_buffersize(MacFrag frag) {
  if (frag->curr_frame) {
    return (frag->curr_frame->size - frag->bytes_sent) + frag->bytes_buffered +
           frag->am_window_bytes_buffered;
  } else {
    return frag->bytes_buffered + frag->am_window_bytes_buffered;
  }
}

void mac_frag_ack_fragment(MacFrag frag, MacMessage ack) {
  uint8_t seqnr = ack->hdr.DLdataAck.seqNr;
  uint8_t fragnr = ack->hdr.DLdataAck.fragNr;

  LOG(DEBUG, "[MAC FRAG] got ACK for %d:%d\n", seqnr, fragnr);
  if (ack->hdr.DLdataAck.ack_type == ACK) {
    arq_window_ack_msg(frag, seqnr, fragnr);
  }
}

MacMessage mac_frag_get_fragment(MacFrag frag, uint max_frag_size,
                                 uint is_uplink) {
  uint bytes_remain = 0, final_flag, data_len;
  MacMessage fragment = NULL;

  if (mac_frag_has_fragment(frag)) {

    // ensure that we can transmit at least one payload byte
    if (max_frag_size <= mac_msg_get_hdrlen(ul_data)) {
      return NULL;
    }
    // check if there are timed out fragments that have not been acknowledged
    // retransmit them
    for (int i = 0; i < ARQ_WINDOW_LEN; i++) {
      int idx = (frag->am_last_ack_idx + i) % ARQ_WINDOW_LEN;
      if (frag->am_send_window[idx] != NULL) {
        if (frag->am_window_timestamp[idx] + ARQ_ACK_TIMEOUT <
            *frag->subframe) {
          // no ack received, retransmit this frame
          fragment = mac_msg_copy(frag->am_send_window[idx]);
          frag->am_window_retransmits[idx]++;
          frag->am_window_timestamp[idx] = *frag->subframe;
          if (frag->am_window_retransmits[idx] > ARQ_MAX_RETRANSMITS) {
            arq_window_remove(frag, idx);
          }
          LOG(DEBUG,
              "[MAC FRAG] retransmit fragment %d:%d. Num retransmits: %d\n",
              fragment->hdr.DLdata.seqNr, fragment->hdr.DLdata.fragNr,
              frag->am_window_retransmits[idx]);
          return fragment;
        }
      }
    }

    if (frag->curr_frame) {
      // there is a open frame that is being fragmented
      bytes_remain = frag->curr_frame->size - frag->bytes_sent;
    } else {
      // fetch new frame from queue
      MacDataFrame sdu = ringbuf_get(frag->frame_queue);
      if (sdu == NULL) {
        LOG(ERR, "[MAC FRAG] cannot fetch any SDU from buf\n");
        return NULL;
      }
      frag->bytes_buffered -= sdu->size;
      frag->curr_frame = sdu;
      if (sdu->do_arq) {
        frag->am_fragNr = 0;
        frag->am_seqNr = (frag->am_seqNr + 1) % MAX_SEQNR;
      } else {
        frag->um_fragNr = 0;
        frag->um_seqNr = (frag->um_seqNr + 1) % MAX_SEQNR;
      }
      frag->bytes_sent = 0;
      bytes_remain = sdu->size;
    }

    // get fragment size and final flag
    if (max_frag_size >= bytes_remain + mac_msg_get_hdrlen(ul_data)) {
      data_len = bytes_remain;
      final_flag = 1;
    } else {
      data_len = max_frag_size - mac_msg_get_hdrlen(ul_data);
      final_flag = 0;
    }

    if (frag->curr_frame->do_arq) {
      if (!arq_window_isfull(frag)) {
        // create MAC Message in Acknowledged mode and add it to send window
        if (is_uplink) {
          fragment = mac_msg_create_ul_data(
              data_len, 1, final_flag, frag->am_seqNr, frag->am_fragNr++,
              frag->curr_frame->data + frag->bytes_sent);
        } else {
          fragment = mac_msg_create_dl_data(
              data_len, 1, final_flag, frag->am_seqNr, frag->am_fragNr++,
              frag->curr_frame->data + frag->bytes_sent);
        }
        arq_window_put(frag, fragment);
        fragment = mac_msg_copy(fragment);
      }
    } else {
      // create MAC Message in Unacknowledged Mode
      if (is_uplink) {
        fragment = mac_msg_create_ul_data(
            data_len, 0, final_flag, frag->um_seqNr, frag->um_fragNr++,
            frag->curr_frame->data + frag->bytes_sent);
      } else {
        fragment = mac_msg_create_dl_data(
            data_len, 0, final_flag, frag->um_seqNr, frag->um_fragNr++,
            frag->curr_frame->data + frag->bytes_sent);
      }
    }

    // update fragmenter state
    frag->bytes_sent += data_len;
    if (final_flag) {
      dataframe_destroy(frag->curr_frame);
      frag->curr_frame = NULL;
    }

    return fragment;
  } else {
    return NULL; // No fragment available
  }
}

MacAssmbl mac_assmbl_init() {
  MacAssmbl assmbl = calloc(sizeof(struct MacReassembler_s), 1);

  assmbl->um_assmbl = calloc(sizeof(struct MacReassembler_um_s), 1);
  assmbl->am_assmbl = calloc(sizeof(struct MacReassembler_am_s), 1);

  for (int i = 0; i < MAX_SEQNR; i++) {
    for (int j = 0; j < MAX_FRAGNR; j++) {
      assmbl->am_assmbl->fragments[i][j] = malloc(MAX_FRAGMENT_SIZE);
    }
  }
  return assmbl;
}

void mac_assmbl_destroy(MacAssmbl assmbl) {
  for (int i = 0; i < assmbl->um_assmbl->fragNr; i++) {
    free(assmbl->um_assmbl->fragments[i]);
  }
  for (int i = 0; i < MAX_SEQNR; i++) {
    for (int j = 0; j < MAX_SEQNR; j++) {
      free(assmbl->am_assmbl->fragments[i][j]);
    }
  }
  free(assmbl);
}

MacDataFrame mac_assmbl_reassemble_um(MacAssmblUM assmbl, MacMessage fragment) {
  MacDLdata *data = &fragment->hdr.DLdata;
  MacDataFrame frame;

  if (!assmbl->frame_open) {
    // no frame open yet, store the received sequence number
    if (data->fragNr == 0) {
      assmbl->seqNr = data->seqNr;
      assmbl->fragNr = 0;
      assmbl->frame_open = 1;
    } else {
      LOG(DEBUG,
          "[MAC ASSMBL UM] unexpected fragNr for new frame. "
          "Got %d Expect 0\n",
          data->fragNr);
      return NULL;
    }
  }

  // ensure that the sequence number and fragment number matches
  // TODO implement unordered fragment reception
  if ((assmbl->seqNr == data->seqNr) && (assmbl->fragNr == data->fragNr)) {
    LOG(DEBUG, "[MAC ASSMBL UM] new matching fragment %d:%d\n", assmbl->seqNr,
        assmbl->fragNr);
    assmbl->fragments[assmbl->fragNr] = malloc(fragment->payload_len);
    memcpy(assmbl->fragments[assmbl->fragNr], fragment->data,
           fragment->payload_len);
    assmbl->fragments_len[assmbl->fragNr] = fragment->payload_len;
    assmbl->frame_len += fragment->payload_len;
    assmbl->fragNr++;
  } else {
    // reset reassembler state
    LOG(DEBUG,
        "[MAC ASSMBL UM] seq/frag Nr does not match: Got %d:%d, "
        "expected %d:%d\n",
        data->seqNr, data->fragNr, assmbl->seqNr, assmbl->fragNr);
    for (int i = 0; i < assmbl->fragNr; i++) {
      free(assmbl->fragments[i]);
    }
    // if fragnr of the frame is 0, we can use it
    // as a new start
    if (data->fragNr == 0) {
      assmbl->fragments[0] = malloc(fragment->payload_len);
      memcpy(assmbl->fragments[0], fragment->data, fragment->payload_len);
      assmbl->fragments_len[0] = fragment->payload_len;
      assmbl->fragNr = 1;
      assmbl->seqNr = data->seqNr;
      assmbl->frame_open = 1;
      assmbl->frame_len = fragment->payload_len;
    } else {
      // reset reassembler state
      assmbl->frame_open = 0;
      assmbl->frame_len = 0;
      assmbl->fragNr = 0;
      return NULL;
    }
  }

  if (data->final_flag) {
    frame = dataframe_create(assmbl->frame_len);
    frame->do_arq = 0;
    uint8_t *p = frame->data;
    for (int i = 0; i < assmbl->fragNr; i++) {
      memcpy(p, assmbl->fragments[i], assmbl->fragments_len[i]);
      p += assmbl->fragments_len[i];
      free(assmbl->fragments[i]);
    }
    assmbl->fragNr = 0;
    assmbl->frame_open = 0;
    assmbl->frame_len = 0;
    return frame;
  } else {
    // no complete frame received yet
    return NULL;
  }
}

MacDataFrame mac_assmbl_reassemble_am(MacAssmblAM assmbl, MacMessage fragment) {
  MacDataFrame frame;
  MacDLdata hdr = fragment->hdr.DLdata;
  uint8_t seqnr = hdr.seqNr;
  uint8_t fragnr = hdr.fragNr;

  // store fragment in buffer
  memcpy(assmbl->fragments[seqnr][fragnr], fragment->data,
         fragment->payload_len);
  assmbl->fragments_len[seqnr][fragnr] = fragment->payload_len;
  assmbl->frag_rcv_bitmask[seqnr][fragnr] = 1;
  if (hdr.final_flag) {
    assmbl->seq_got_final[seqnr] = 1;
    assmbl->num_fragments[seqnr] = fragnr + 1;
  }

  LOG(DEBUG, "[MAC FRAG AM] got fragment %d:%d\n", seqnr, fragnr);
  // check if we have all fragments to release a frame
  if (assmbl->seq_got_final[seqnr]) {
    uint frag_cnt = 0;
    for (int i = 0; i < MAX_FRAGNR; i++) {
      frag_cnt += assmbl->frag_rcv_bitmask[seqnr][i];
    }
    if (frag_cnt == assmbl->num_fragments[seqnr]) {
      // we got all fragments and can release the frame
      uint frame_len = 0;
      for (int i = 0; i < assmbl->num_fragments[seqnr]; i++) {
        frame_len += assmbl->fragments_len[seqnr][i];
      }
      frame = dataframe_create(frame_len);
      frame->do_arq = 1;
      uint8_t *p = frame->data;
      for (int i = 0; i < assmbl->num_fragments[seqnr]; i++) {
        memcpy(p, assmbl->fragments[seqnr][i], assmbl->fragments_len[seqnr][i]);
        p += assmbl->fragments_len[seqnr][i];
        assmbl->frag_rcv_bitmask[seqnr][i] = 0;
      }
      assmbl->seq_got_final[seqnr] = 0;
      assmbl->num_fragments[seqnr] = 0;
      return frame;
    }
  }
  return NULL;
}

MacDataFrame mac_assmbl_reassemble(MacAssmbl assmbl, MacMessage fragment) {
  if (fragment->hdr.DLdata.do_ack) {
    return mac_assmbl_reassemble_am(assmbl->am_assmbl, fragment);
  } else {
    return mac_assmbl_reassemble_um(assmbl->um_assmbl, fragment);
  }
}