/*
 * platform.h
 *
 *  Created on: Jan 16, 2020
 *      Author: lukas
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
 *			prepare the tx buf by writing num_samples samples starting
 *			at the specified offset to the buffer
 *
 *	platform_rx(platform, float complex*):
 *			fetch new samples and pass them to the specified buffer
 *
 *	end(platform):
 *			do necessary operations for shutdown
 */

#ifndef PLATFORM_PLATFORM_H_
#define PLATFORM_PLATFORM_H_

#include <complex.h>

struct platform_s {
	int (*platform_tx_push)(struct platform_s*);
	int (*platform_tx_prep)(struct platform_s*, float complex*, unsigned int offset, unsigned int num_samples);
	int (*platform_rx)(struct platform_s*, float complex*);
	void (*end)(struct platform_s*);
	void (*ptt_set_tx)(struct platform_s*);
	void (*ptt_set_rx)(struct platform_s*);
	void* data;	// Pointer to store some data if necessary for some platform
};

typedef struct platform_s* platform;


#endif /* PLATFORM_PLATFORM_H_ */
