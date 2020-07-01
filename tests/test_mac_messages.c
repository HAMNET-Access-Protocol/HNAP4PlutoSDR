#include <unity.h>
#include <mac_messages.h>

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


// not needed when using generate_test_runner.rb
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mac_msg_get_hdrlen);
    return UNITY_END();
}