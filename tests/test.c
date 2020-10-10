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
 * Main file for the Unit test suite
 *
 */

#include "test_mac_common.c"
#include "test_mac_messages.c"


void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

// not needed when using generate_test_runner.rb
int main(void) {
  UNITY_BEGIN();

  // MAC messsages
  RUN_TEST(test_mac_msg_get_hdrlen);
  RUN_TEST(test_mac_msg_parse_associate_response);
  RUN_TEST(test_mac_msg_parse_dldata);
  RUN_TEST(test_mac_msg_parse_uldata);
  RUN_TEST(test_mac_msg_parse_dl_data_ack);
  RUN_TEST(test_mac_msg_parse_ul_data_ack);

  // MAC common
  RUN_TEST(test_packet_inspect_is_tcpip_nullptr);
  RUN_TEST(test_packet_inspect_is_tcpip_shortframe);
  RUN_TEST(test_packet_inspect_is_tcpip_tcp);
  RUN_TEST(test_packet_inspect_is_tcpip_udp);
  return UNITY_END();
}