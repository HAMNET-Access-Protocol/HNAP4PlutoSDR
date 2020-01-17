/*
 * ringbuf.h
 *
 *  Created on: Jan 10, 2020
 *      Author: lukas
 */

#ifndef UTIL_RINGBUF_H_
#define UTIL_RINGBUF_H_

#include <stdint.h>
#include <stddef.h>

// Ringbuf structure that stores pointers
// uses void pointers internally
// NOTE: the capacity of the buffer is size-1 items
//		 due to the specific implementation of the get and put functions


struct ringbuf_s;
typedef struct ringbuf_s* ringbuf;

// Initialize a ringbuf of given size
// returns the ringbuf object
ringbuf ringbuf_create(uint32_t size);

// Delete the buffer
void ringbuf_destroy(ringbuf buf);

// Get an item from the buffer;
// returns NULL if the buffer is empty
void* ringbuf_get(ringbuf buf);

// Add an item to the buffer
// returns 1 on success, 0 if the buffer is full
int ringbuf_put(ringbuf buf, void* item);

// Check if the buffer is full
int ringbuf_isfull(ringbuf buf);

// Check if the buffer is full
int ringbuf_isempty(ringbuf buf);

#endif /* UTIL_RINGBUF_H_ */
