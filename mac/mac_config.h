/*
 * mac_config.h
 *
 *  Created on: Jan 10, 2020
 *      Author: lukas
 */

#ifndef MAC_MAC_CONFIG_H_
#define MAC_MAC_CONFIG_H_

#define MAC_MSG_BUF_SIZE 8
#define MAX_USER 16

// enable MAC testing
//#define MAC_TEST_DELAY

// Number of data/control slots per subframe
#define MAC_DLDATA_SLOTS 4 //TODO double definition of the same variable here and in phy_config
#define MAC_ULDATA_SLOTS 4
#define MAC_ULCTRL_SLOTS 2

// set the userID which is reserved as a broadcast identifier
#define USER_BROADCAST 1
// userID that is used to indicate a disabled/unused slot
#define USER_UNUSED 0


#endif /* MAC_MAC_CONFIG_H_ */
