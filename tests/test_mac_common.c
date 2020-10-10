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
 *
 * Unit tests for MAC_COMMON module
 *
 */

#include <mac_common.h>
#include <unity.h>

void test_packet_inspect_is_tcpip_nullptr(void) {
  uint8_t *buf = NULL;
  uint buflen = 42;
  TEST_ASSERT_EQUAL(-1, packet_inspect_is_tcpip(buf, buflen));
}

void test_packet_inspect_is_tcpip_shortframe(void) {
  uint8_t buf[] = {0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
                   0xee, 0xee, 0xee, 0x08, 0x00, 0x45, 0x00, 0x00, 0x34,
                   0xc1, 0x29, 0x40, 0x00, 0x40, 0x06, 0x68, 0x2c, 0xc0,
                   0xa8, 0x01, 0x01, 0xc0, 0xa8, 0x01, 0x02};
  uint buflen = 29;
  TEST_ASSERT_EQUAL(-1, packet_inspect_is_tcpip(buf, buflen));
}

void test_packet_inspect_is_tcpip_tcp(void) {
  // Ethernet header + IP header with TCP payload
  uint8_t buf[] = {0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
                   0xee, 0xee, 0xee, 0x08, 0x00, 0x45, 0x00, 0x00, 0x34,
                   0xc1, 0x29, 0x40, 0x00, 0x40, 0x06, 0x68, 0x2c, 0xc0,
                   0xa8, 0x01, 0x01, 0xc0, 0xa8, 0x01, 0x02};
  uint buflen = sizeof(buf);
  TEST_ASSERT_EQUAL(1, packet_inspect_is_tcpip(buf, buflen));
}

void test_packet_inspect_is_tcpip_udp(void) {
  // Ethernet header + IP header with UDP payload
  uint8_t buf[] = {0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
                   0xee, 0xee, 0xee, 0x08, 0x00, 0x45, 0x00, 0x00, 0x34,
                   0xc1, 0x29, 0x40, 0x00, 0x40, 0x11, 0x68, 0x2c, 0xc0,
                   0xa8, 0x01, 0x01, 0xc0, 0xa8, 0x01, 0x02};
  uint buflen = sizeof(buf);
  TEST_ASSERT_EQUAL(0, packet_inspect_is_tcpip(buf, buflen));
}
