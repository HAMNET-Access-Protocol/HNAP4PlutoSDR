/*
 * tap_dev.h
 *
 *  Created on: Feb 11, 2020
 *      Author: lukas
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
