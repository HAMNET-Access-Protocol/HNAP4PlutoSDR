/*
 * mac_fragmentation.h
 *
 *  Created on: Jan 13, 2020
 *      Author: lukas
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

// Add a frame to the MAC queue
int mac_frag_add_frame(MacFrag frag, MacDataFrame frame);

// Check whether the fragmenter has some data in the queue
int mac_frag_has_fragment(MacFrag frag);

// Get the number of bytes that are currently buffered,
// i.e. bytes that could be sent
int mac_frag_get_buffersize(MacFrag frag);

// Fetch a fragment from the frame queue with a specified maximum
// fragment size
MacMessage mac_frag_get_fragment(MacFrag frag, uint max_frag_size, uint is_uplink);


//// MAC Reassembler methods ////

// Initialize the reassembler structure
MacAssmbl mac_assmbl_init();

// add a new fragment to the reassembler buffer
// returns a MAC frame if reception of a open frame was completed
MacDataFrame mac_assmbl_reassemble(MacAssmbl assmbl, MacMessage fragment);

#endif /* MAC_MAC_FRAGMENTATION_H_ */
