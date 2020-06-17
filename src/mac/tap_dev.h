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

#ifndef MAC_TAP_DEV_H_
#define MAC_TAP_DEV_H_

#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>

#include <stdint.h>

// We want to be able to capture whole ethernet frames
// MTU at least = 1500(ether mtu) + 14(ether header)
// MTU_SIZE defines the size of the buffer TAP can write to
#define MTU_SIZE (1500+14)

struct tap_dev_s {
	char tap_name[IFNAMSIZ];
	int tapfd;

	uint8_t* buffer;
	unsigned int bytes_rec;
};

typedef struct tap_dev_s* tap_dev;

tap_dev tap_init(char* tap_name);
void tap_receive(tap_dev dev);
void tap_send(tap_dev dev, uint8_t* buffer, uint buflen);


#endif /* MAC_TAP_DEV_H_ */
