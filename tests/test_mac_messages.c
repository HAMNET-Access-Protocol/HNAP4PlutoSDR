#include <mac_messages.h>
#include <src/mac/mac_messages.h>
#include <unity.h>

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_mac_msg_get_hdrlen(void) {
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(associate_response), 3);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(dl_mcs_info), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(ul_mcs_info), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(timing_advance), 2);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(session_end), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(dl_data), 3);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(ul_req), 2);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(channel_quality), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(keepalive), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(control_ack), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(mcs_chance_req), 1);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(ul_data), 3);
  TEST_ASSERT_EQUAL_INT(mac_msg_get_hdrlen(255), -1);
}

void test_mac_msg_parse_associate_response(void) {
  // create message
  uint8_t userid = 2;
  uint8_t rachuserid = 5;
  uint8_t response = assoc_resp_success;
  uint8_t timingadvance = 123;
  MacMessage test_msg = mac_msg_create_associate_response(
      userid, rachuserid, response, timingadvance);

  // write it to buffer
  uint len = 128;
  uint8_t *buf = malloc(len);
  mac_msg_generate(test_msg, buf, len);

  // parse and check
  MacMessage parsed_msg = mac_msg_parse(buf, len, 0);
  TEST_ASSERT_EQUAL(associate_response, parsed_msg->type);
  TEST_ASSERT_EQUAL(userid, parsed_msg->hdr.AssociateResponse.userid);
  TEST_ASSERT_EQUAL(rachuserid, parsed_msg->hdr.AssociateResponse.rachuserid);
  TEST_ASSERT_EQUAL(response, parsed_msg->hdr.AssociateResponse.response);
  TEST_ASSERT_EQUAL(timingadvance,
                    parsed_msg->hdr.AssociateResponse.timing_advance);

  mac_msg_destroy(test_msg);
  mac_msg_destroy(parsed_msg);
}

void test_mac_msg_parse_dldata(void) {
  // create message
  uint8_t ack_mode = AM;
  uint8_t final_flag = 1;
  uint8_t fragnr = 31;
  uint8_t seqnr = 7;
  uint datalen = 531;
  uint8_t databuf[531] = {1, 124, 85, 213, 12, 90, 34, 0, 1, 255};
  MacMessage test_msg = mac_msg_create_dl_data(datalen, ack_mode, final_flag,
                                               seqnr, fragnr, databuf);

  // write it to buffer
  uint buflen = 1024;
  uint8_t *msgbuf = malloc(buflen);
  mac_msg_generate(test_msg, msgbuf, buflen);

  // parse and check
  MacMessage parsed_msg = mac_msg_parse(msgbuf, buflen, 0);
  TEST_ASSERT_EQUAL(dl_data, parsed_msg->type);
  TEST_ASSERT_EQUAL(datalen, parsed_msg->hdr.DLdata.data_length);
  TEST_ASSERT_EQUAL(ack_mode, parsed_msg->hdr.DLdata.do_ack);
  TEST_ASSERT_EQUAL(final_flag, parsed_msg->hdr.DLdata.final_flag);
  TEST_ASSERT_EQUAL(fragnr, parsed_msg->hdr.DLdata.fragNr);
  TEST_ASSERT_EQUAL(seqnr, parsed_msg->hdr.DLdata.seqNr);
  TEST_ASSERT_EQUAL_MEMORY(databuf, parsed_msg->data, datalen);
  mac_msg_destroy(test_msg);
  mac_msg_destroy(parsed_msg);
  free(msgbuf);
}

void test_mac_msg_parse_uldata(void) {
  // create message
  uint8_t ack_mode = AM;
  uint8_t final_flag = 1;
  uint8_t fragnr = 31;
  uint8_t seqnr = 7;
  uint datalen = 531;
  uint8_t databuf[531] = {1, 124, 85, 213, 12, 90, 34, 0, 1, 255};
  MacMessage test_msg = mac_msg_create_ul_data(datalen, ack_mode, final_flag,
                                               seqnr, fragnr, databuf);

  // write it to buffer
  uint buflen = 1024;
  uint8_t *msgbuf = malloc(buflen);
  mac_msg_generate(test_msg, msgbuf, buflen);

  // parse and check
  MacMessage parsed_msg = mac_msg_parse(msgbuf, buflen, 1);
  TEST_ASSERT_EQUAL(ul_data, parsed_msg->type);
  TEST_ASSERT_EQUAL(datalen, parsed_msg->hdr.DLdata.data_length);
  TEST_ASSERT_EQUAL(ack_mode, parsed_msg->hdr.DLdata.do_ack);
  TEST_ASSERT_EQUAL(final_flag, parsed_msg->hdr.DLdata.final_flag);
  TEST_ASSERT_EQUAL(fragnr, parsed_msg->hdr.DLdata.fragNr);
  TEST_ASSERT_EQUAL(seqnr, parsed_msg->hdr.DLdata.seqNr);
  TEST_ASSERT_EQUAL_MEMORY(databuf, parsed_msg->data, datalen);

  mac_msg_destroy(test_msg);
  mac_msg_destroy(parsed_msg);
  free(msgbuf);
}

void test_mac_msg_parse_dl_data_ack(void) {
  // create message
  uint8_t ack_type = ACK;
  uint8_t seqNr = 5;
  uint8_t fragNr = 13;
  MacMessage test_msg = mac_msg_create_dl_data_ack(ack_type, seqNr, fragNr);

  // write msg to buffer
  uint buflen = 128;
  uint8_t *buf = malloc(buflen);
  mac_msg_generate(test_msg, buf, buflen);

  // parse and check
  MacMessage parsed_msg = mac_msg_parse(buf, buflen, 1);
  TEST_ASSERT_EQUAL(dl_data_ack, parsed_msg->type);
  TEST_ASSERT_EQUAL(ack_type, parsed_msg->hdr.DLdataAck.ack_type);
  TEST_ASSERT_EQUAL(seqNr, parsed_msg->hdr.DLdataAck.seqNr);
  TEST_ASSERT_EQUAL(fragNr, parsed_msg->hdr.DLdataAck.fragNr);

  mac_msg_destroy(test_msg);
  mac_msg_destroy(parsed_msg);
  free(buf);
}

void test_mac_msg_parse_ul_data_ack(void) {
  // create message
  uint8_t ack_type = ACK;
  uint8_t seqNr = 5;
  uint8_t fragNr = 13;
  MacMessage test_msg = mac_msg_create_ul_data_ack(ack_type, seqNr, fragNr);

  // write msg to buffer
  uint buflen = 128;
  uint8_t *buf = malloc(buflen);
  mac_msg_generate(test_msg, buf, buflen);

  // parse and check
  MacMessage parsed_msg = mac_msg_parse(buf, buflen, 0);
  TEST_ASSERT_EQUAL(ul_data_ack, parsed_msg->type);
  TEST_ASSERT_EQUAL(ack_type, parsed_msg->hdr.ULdataAck.ack_type);
  TEST_ASSERT_EQUAL(seqNr, parsed_msg->hdr.ULdataAck.seqNr);
  TEST_ASSERT_EQUAL(fragNr, parsed_msg->hdr.ULdataAck.fragNr);

  mac_msg_destroy(test_msg);
  mac_msg_destroy(parsed_msg);
  free(buf);
}

// not needed when using generate_test_runner.rb
int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mac_msg_get_hdrlen);
  RUN_TEST(test_mac_msg_parse_associate_response);
  RUN_TEST(test_mac_msg_parse_dldata);
  RUN_TEST(test_mac_msg_parse_uldata);
  RUN_TEST(test_mac_msg_parse_dl_data_ack);
  RUN_TEST(test_mac_msg_parse_ul_data_ack);
  return UNITY_END();
}