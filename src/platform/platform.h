/*
 * HNAP4PlutoSDR - HAMNET Access Protocol implementation for the Adalm Pluto SDR
 *
 * Copyright (C) 2020 Lukas Ostendorf <lukas.ostendorf@gmail.com>
 *                    and the project contributors
 *
 * This library is free software; you can redistribute it and/or modify it under
 *the terms of the GNU Lesser General Public License as published by the Free
 *Software Foundation; version 3.0.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 *ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 *details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 *along with this library; if not, write to the Free Software Foundation, Inc.,
 *51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 *
 *	Platform abstraction.
 *
 *	Every platform has to support 5 basic functions:
 *
 *	init_platform(uint buf_size):
 *			initialize the platform
 *			using the specified buffer size.
 *
 *	tx_push(platform):
 *			release the TX samples
 *
 *	tx_prep(platform, float complex*, uint offset, num_samples):
 *			prepare the tx buf by writing num_samples samples
 *starting at the specified offset to the buffer
 *
 *	platform_rx(platform, float complex*):
 *			fetch new samples and pass them to the specified buffer
 *
 *	end(platform):
 *			do necessary operations for shutdown
 *
 *	Optional a manual PTT signal can be generated, e.g. at a GPIO pin
 *	to use this feature, implement handlers for
 *	ptt_set_tx(platform)
 *	ptt_set_rx(platform)
 *
 *	If you do not want to use this, implement dummy functions for these
 *handlers.
 */

#ifndef PLATFORM_PLATFORM_H_
#define PLATFORM_PLATFORM_H_

#include <complex.h>

struct platform_s {
  int (*platform_tx_push)(struct platform_s *);
  int (*platform_tx_prep)(struct platform_s *, float complex *,
                          unsigned int offset, unsigned int num_samples);
  int (*platform_rx)(struct platform_s *, float complex *);
  void (*end)(struct platform_s *);
  void (*ptt_set_tx)(struct platform_s *);
  void (*ptt_set_rx)(struct platform_s *);
  void *data; // Pointer to store some data if necessary for some platform
};

typedef struct platform_s *platform;

#endif /* PLATFORM_PLATFORM_H_ */
