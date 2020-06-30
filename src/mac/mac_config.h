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

#ifndef MAC_MAC_CONFIG_H_
#define MAC_MAC_CONFIG_H_

// Number of control messages that can be enqueued
#define MAC_CTRL_MSG_BUF_SIZE 32
// Number of data frames that can be enqueued
#define MAC_DATA_BUF_SIZE 32

// Maximum allowed response time for control messages sent by BS
// Unit: number of subframes
#define MAX_RESPONSE_TIME 32

// Inactivity timer for removing unresponsive/disconnected users
// Unit: number of subframes
#define TMR_USER_INACTIVE 200

// Maximum number of users. Fixed and should not be changed
#define MAX_USER 16

// MAC MTU size. Should be larger than 1514 in order to support ether
// forwarding. Max possible value depends on MCS and amount of control data
// theoretic max for MCS0: 32fragments*60bytes/fragment ~= 1900
#define MAC_MTU 1550

// enable MAC testing
#ifdef SIM_LOG_DELAY
#define MAC_TEST_DELAY
#endif

// Number of data/control slots per subframe
#define MAC_DLDATA_SLOTS                                                       \
  4 // TODO double definition of the same variable here and in phy_config
#define MAC_ULDATA_SLOTS 4
#define MAC_ULCTRL_SLOTS 2

// set the userID which is reserved as a broadcast identifier
#define USER_BROADCAST 1
// userID that is used to indicate a disabled/unused slot
#define USER_UNUSED 0

#endif /* MAC_MAC_CONFIG_H_ */
