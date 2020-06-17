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

#include "tap_dev.h"

#include <linux/if.h>
#include <linux/if_tun.h>
#include <netinet/ether.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

#include "../util/log.h"

int tun_alloc(char *dev, int flags) {

  struct ifreq ifr;
  int fd, err;
  char *clonedev = "/dev/net/tun";

  /* Arguments taken by the function:
   *
   * char *dev: the name of an interface (or '\0'). MUST have enough
   *   space to hold the interface name if '\0' is passed
   * int flags: interface flags (eg, IFF_TUN etc.)
   */

   /* open the clone device */
   if( (fd = open(clonedev, O_RDWR)) < 0 ) {
     return fd;
   }

   /* preparation of the struct ifr, of type "struct ifreq" */
   memset(&ifr, 0, sizeof(ifr));

   ifr.ifr_flags = flags;   /* IFF_TUN or IFF_TAP, plus maybe IFF_NO_PI */

   if (*dev) {
     /* if a device name was specified, put it in the structure; otherwise,
      * the kernel will try to allocate the "next" device of the
      * specified type */
     strncpy(ifr.ifr_name, dev, IFNAMSIZ);
   }

   /* try to create the device */
   if( (err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0 ) {
     close(fd);
     return err;
   }

  /* if the operation was successful, write back the name of the
   * interface to the variable "dev", so the caller can know
   * it. Note that the caller MUST reserve space in *dev (see calling
   * code below) */
  strcpy(dev, ifr.ifr_name);

  return fd;
}

// initialize the tap device structure
tap_dev tap_init(char* tap_name)
{
	tap_dev dev = calloc(sizeof(struct tap_dev_s),1);
	strcpy(dev->tap_name,tap_name);
	dev->tapfd = tun_alloc(dev->tap_name, IFF_TAP | IFF_NO_PI);
	if (dev->tapfd==-1) {
		LOG(ERR,"Could not create TAP device!\n");
		free(dev);
		exit(EXIT_FAILURE);
		return NULL;
	}

	dev->buffer = malloc(MTU_SIZE);
	return dev;
}

// try to receive from the tap device
void tap_receive(tap_dev dev)
{
	int nread = read(dev->tapfd, dev->buffer, MTU_SIZE);
	if (nread < 0) {
		LOG(ERR,"[TAP DEV] could not read TAP device!\n");
		nread = 0;
	}
	dev->bytes_rec = nread;
}

// push a buffer to the tap device
void tap_send(tap_dev dev, uint8_t* buffer, uint buflen)
{
	int nwrite  = write(dev->tapfd, buffer, buflen);
	if (nwrite != buflen) {
		LOG(ERR, "[TAP DEV] could not write to TAP device!\n");
	}
}

