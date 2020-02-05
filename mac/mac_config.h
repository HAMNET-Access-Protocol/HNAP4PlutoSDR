/*
 * mac_config.h
 *
 *  Created on: Jan 10, 2020
 *      Author: lukas
 */

#ifndef MAC_MAC_CONFIG_H_
#define MAC_MAC_CONFIG_H_

#include "../config.h"

// Number of control messages that can be enqueued
#define MAC_CTRL_MSG_BUF_SIZE 32
// Number of data frames that can be enqueued
#define MAC_DATA_BUF_SIZE 32

// Maximum allowed response time for control messages sent by BS
// Unit: number of subframes
#define MAX_RESPONSE_TIME 32

// Maximum number of users. Fixed and should not be changed
#define MAX_USER 16

// enable MAC testing
#ifdef USE_SIM
#define MAC_TEST_DELAY
#endif

// Number of data/control slots per subframe
#define MAC_DLDATA_SLOTS 4 //TODO double definition of the same variable here and in phy_config
#define MAC_ULDATA_SLOTS 4
#define MAC_ULCTRL_SLOTS 2

// set the userID which is reserved as a broadcast identifier
#define USER_BROADCAST 1
// userID that is used to indicate a disabled/unused slot
#define USER_UNUSED 0


#endif /* MAC_MAC_CONFIG_H_ */
