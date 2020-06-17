/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library;
 * if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301 USA
 */

#ifndef MAC_MAC_FRAGMENTATION_H_
#define MAC_MAC_FRAGMENTATION_H_

#include <stddef.h>
#include "mac_messages.h"
#include "mac_common.h"


//// Fragmenter / Reassembler struct declarations ////
struct MacFragmenter_s;
struct MacReassembler_s;

typedef struct MacFragmenter_s* MacFrag;
typedef struct MacReassembler_s* MacAssmbl;


//// MAC Fragmenter methods ////

// Initialize the fragmenter structure
MacFrag mac_frag_init();
void mac_frag_destroy(MacFrag frag);

// Add a frame to the MAC queue
int mac_frag_add_frame(MacFrag frag, MacDataFrame frame);

// Check whether the fragmenter has some data in the queue
int mac_frag_has_fragment(MacFrag frag);

// Check whether the fragmenter queue is full
int mac_frag_queue_full(MacFrag frag);

// Get the number of bytes that are currently buffered,
// i.e. bytes that could be sent
int mac_frag_get_buffersize(MacFrag frag);

// Fetch a fragment from the frame queue with a specified maximum
// fragment size
MacMessage mac_frag_get_fragment(MacFrag frag, uint max_frag_size, uint is_uplink);


//// MAC Reassembler methods ////

// Initialize the reassembler structure
MacAssmbl mac_assmbl_init();
void mac_assmbl_destroy(MacAssmbl assmbl);

// add a new fragment to the reassembler buffer
// returns a MAC frame if reception of a open frame was completed
MacDataFrame mac_assmbl_reassemble(MacAssmbl assmbl, MacMessage fragment);

#endif /* MAC_MAC_FRAGMENTATION_H_ */
