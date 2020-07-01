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

#include "ringbuf.h"
#include <stdlib.h>

struct ringbuf_s {
  void **data;
  uint32_t readpos;
  uint32_t writepos;
  uint32_t size;
};

ringbuf ringbuf_create(uint32_t size) {
  ringbuf buf = malloc(sizeof(struct ringbuf_s));
  buf->data = calloc(size * sizeof(void *), 1);
  buf->writepos = 0;
  buf->readpos = 0;
  buf->size = size;

  return buf;
}

void ringbuf_destroy(ringbuf buf) {
  while (!ringbuf_isempty(buf)) {
    void *p = ringbuf_get(buf);
    free(p);
  }
  free(buf->data);
  free(buf);
}

void *ringbuf_get(ringbuf buf) {
  void *item;
  if (ringbuf_isempty(buf)) {
    return NULL; // no element in buf
  } else {
    item = buf->data[buf->readpos];
    buf->readpos = (buf->readpos + 1) % buf->size;
    return item;
  }
}

int ringbuf_put(ringbuf buf, void *item) {
  if (ringbuf_isfull(buf)) {
    return 0;
  }
  buf->data[buf->writepos] = item;
  buf->writepos = (buf->writepos + 1) % buf->size;
  return 1;
}

int ringbuf_isfull(ringbuf buf) {
  if ((buf->writepos + 1) % buf->size == buf->readpos) {
    return 1; // buffer full
  } else {
    return 0;
  }
}

int ringbuf_isempty(ringbuf buf) {
  if (buf->readpos == buf->writepos) {
    return 1;
  } else {
    return 0;
  }
}
