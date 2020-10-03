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

#ifndef MAC_MAC_FRAGMENTATION_H_
#define MAC_MAC_FRAGMENTATION_H_

#include "mac_common.h"
#include "mac_messages.h"
#include <stddef.h>

//// Fragmenter / Reassembler struct declarations ////
struct MacFragmenter_s;
struct MacReassembler_s;

typedef struct MacFragmenter_s *MacFrag;
typedef struct MacReassembler_s *MacAssmbl;

//// MAC Fragmenter methods ////

/**
 * Initialize the fragmenter structure
 *
 * @param subframe_ptr Pointer to the subframe counter in a MAC object.
 *                     This is used to get a common time instance in order to
 *                     retransmit fragments after a while
 * @returns A MacFragmenter struct if the initialization was successful
 */
MacFrag mac_frag_init(long long unsigned int *subframe_ptr);
/**
 * Destroy a MacFragmenter object
 *
 * @param frag MacFragmenter object to be destroyed
 */
void mac_frag_destroy(MacFrag frag);

/**
 * Add a frame to the MAC queue
 *
 * @param frag MacFragmenter object.
 * @returns 1 if the frame has been successfully added to the queue
 */
int mac_frag_add_frame(MacFrag frag, MacDataFrame frame);

/**
 * Check whether the fragmenter has some data in the queue
 *
 * @param frag MacFragmenter object
 * @returns 1 if the fragmenter is able to return a fragment, else 0
 */
int mac_frag_has_fragment(MacFrag frag);

/**
 * Check whether the fragmenter queue is full
 *
 * @param frag MacFragmenter object
 * @returns 1 if no further frames can be added to the fragmenter queue, else 0
 */
int mac_frag_queue_full(MacFrag frag);

/**
 * Get the number of bytes that are currently buffered in the Fragmenter object
 *
 * @param frag MacFragmenter object
 * @returns Number of bytes that are buffered (and thus available for
 * transmission)
 */
int mac_frag_get_buffersize(MacFrag frag);

/**
 * Fetch a fragment from the frame queue with a specified maximum fragment size
 *
 * This function first checks, whether any fragment that has already been sent
 * has to be retransmitted because no ACK was received. If not, it will select
 * the next frame from the buffer.
 * If there is currently a frame being fragmented, the next fragment of this
 * frame will be returned If there is no open frame available, a new frame from
 * the Fragmenter queue will be fetched and the fragmentation process for this
 * frame will start.
 *
 * @param frag MacFragmenter object
 * @param max_frag_size maximum fragment size in bytes
 * @param is_uplink 1 if the request is made for an Uplink transmission
 * @returns An uplink or a downlink data message. If no fragment is available,
 *          NULL is returned.
 */
MacMessage mac_frag_get_fragment(MacFrag frag, uint max_frag_size,
                                 uint is_uplink);

/**
 * Callback to acknowledge a fragment that has been sent using Ack mode
 *
 * This function will search for the sequence number and fragment id in
 * the send_window. If the pair is found, the corresponding fragment
 * is removed from the window
 *
 * @param frag MacFragmenter object
 * @param ack the dl_data_ack or ul_data_ack message that has been received
 */
void mac_frag_ack_fragment(MacFrag frag, MacMessage ack);

//// MAC Reassembler methods ////

/**
 * Initialize the reassembler structure
 *
 * @returns A Mac Reassembler object if the initialization was successful
 */
MacAssmbl mac_assmbl_init();

/**
 * Destroy the Mac Reassembler object
 *
 * @param assmbl Mac Reassembler object
 */
void mac_assmbl_destroy(MacAssmbl assmbl);

/**
 * Add a new received fragment to the Reassembler buffer
 *
 * After the fragment is added to the receive buffer, the reassembler checks
 * whether a frame reception is completed. In this case, the frame will be
 * returned.
 *
 * @param assmbl Reassembler object
 * @param fragment the received fragment
 * @returns A mac data frame, if the reassembler was able to reassemble
 *          a complete frame. Otherwise NULL is returned
 */
MacDataFrame mac_assmbl_reassemble(MacAssmbl assmbl, MacMessage fragment);

#endif /* MAC_MAC_FRAGMENTATION_H_ */
