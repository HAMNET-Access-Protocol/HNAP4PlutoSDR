/*
 * mac_bs.h
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_BS_H_
#define MAC_MAC_BS_H_

#include "mac_config.h"
#include "mac_fragmentation.h"
#include "mac_common.h"

#include "../util/ringbuf.h"
#include <liquid/liquid.h>
#include "../phy/phy_bs.h"


typedef struct {
	ofdmframesync fs;			// framesync object. Stores freq offset etc.
	uint8_t userid;
	uint ul_queue;
	ringbuf msg_control_queue;
	MacFrag fragmenter;
	MacAssmbl reassembler;

	uint8_t dl_mcs;				// The mcs schemes used for the user
	uint8_t ul_mcs;
}user_s;

//forward declaration of phy struct that is needed in mac struct
struct PhyBS_s;

struct MacBS_s {
	ringbuf broadcast_ctrl_queue;
	MacFrag broadcast_data_fragmenter;
	user_s* UE[MAX_USER];

	uint8_t ul_ctrl_assignments[MAC_ULCTRL_SLOTS];
	uint8_t ul_data_assignments[MAC_DLDATA_SLOTS];
	uint8_t dl_data_assignments[MAC_ULDATA_SLOTS];

	struct PhyBS_s* phy;
};

typedef struct MacBS_s* MacBS;

MacBS mac_bs_init();
void mac_bs_destroy(MacBS mac);

void mac_bs_add_new_ue(MacBS mac, uint8_t rachuserid, ofdmframesync fs);
int mac_bs_add_txdata(MacBS mac, uint8_t destUserID, MacDataFrame frame);
void mac_bs_run_scheduler(MacBS mac);

#endif /* MAC_MAC_BS_H_ */
