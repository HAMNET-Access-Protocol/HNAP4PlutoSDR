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

#ifndef UTIL_RINGBUF_H_
#define UTIL_RINGBUF_H_

#include <stddef.h>
#include <stdint.h>

// Ringbuf structure that stores pointers
// uses void pointers internally
// NOTE: the capacity of the buffer is size-1 items
//		 due to the specific implementation of the get and put functions

struct ringbuf_s;
typedef struct ringbuf_s *ringbuf;

// Initialize a ringbuf of given size
// returns the ringbuf object
ringbuf ringbuf_create(uint32_t size);

// Delete the buffer
void ringbuf_destroy(ringbuf buf);

// Get an item from the buffer;
// returns NULL if the buffer is empty
void *ringbuf_get(ringbuf buf);

// Add an item to the buffer
// returns 1 on success, 0 if the buffer is full
int ringbuf_put(ringbuf buf, void *item);

// Check if the buffer is full
int ringbuf_isfull(ringbuf buf);

// Check if the buffer is full
int ringbuf_isempty(ringbuf buf);

#endif /* UTIL_RINGBUF_H_ */
