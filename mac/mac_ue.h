/*
 * mac_ue.h
 *
 *  Created on: Dec 10, 2019
 *      Author: lukas
 */

#ifndef MAC_MAC_UE_H_
#define MAC_MAC_UE_H_

#include "mac_channels.h"

void mac_ue_rec_channel(LogicalChannel chan);
int mac_ue_get_mcs_dl();

#endif /* MAC_MAC_UE_H_ */
