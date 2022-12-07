/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_test, CONFIG_NET_WEBSOCKET_LOG_LEVEL);

#include <zephyr/ztest_assert.h>

#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>

#include "websocket_internal.h"

/* Generated by http://www.lipsum.com/
 * 2 paragraphs, 178 words, 1160 bytes of Lorem Ipsum
 */
static const char lorem_ipsum[] =
	"Lorem ipsum dolor sit amet, consectetur adipiscing elit. "
	"Vestibulum ultricies sapien tellus, ac viverra dolor bibendum "
	"lacinia. Vestibulum et nisl tristique tellus finibus gravida "
	"vitae sit amet nunc. Suspendisse maximus justo mi, vitae porta "
	"risus suscipit vitae. Curabitur ut fringilla velit. Donec ac nisi "
	"in dui semper lobortis sed nec ante. Sed nec luctus dui. Sed ut "
	"ante nisi. Mauris congue euismod felis, et maximus ex pellentesque "
	"nec. Proin nibh nisl, semper at nunc in, mattis pharetra metus. Nam "
	"turpis risus, pulvinar sit amet varius ac, pellentesque quis purus."
	" "
	"Nam consequat purus in lacinia fringilla. Morbi volutpat, tellus "
	"nec tempus dapibus, ante sem aliquam dui, eu feugiat libero diam "
	"at leo. Sed suscipit egestas orci in ultrices. Integer in elementum "
	"ligula, vel sollicitudin velit. Nullam sit amet eleifend libero. "
	"Proin sit amet consequat tellus, vel vulputate arcu. Curabitur quis "
	"lobortis lacus. Sed faucibus vestibulum enim vel elementum. Vivamus "
	"enim nunc, auctor in purus at, aliquet pulvinar eros. Cras dapibus "
	"nec quam laoreet sagittis. Quisque dictum ante odio, at imperdiet "
	"est convallis a. Morbi mattis ut orci vitae volutpat."
	"\n";

#define MAX_RECV_BUF_LEN 256
static uint8_t recv_buf[MAX(sizeof(lorem_ipsum), MAX_RECV_BUF_LEN)];

/* We need to allocate bigger buffer for the websocket data we receive so that
 * the websocket header fits into it.
 */
#define EXTRA_BUF_SPACE 30

static uint8_t temp_recv_buf[MAX_RECV_BUF_LEN + EXTRA_BUF_SPACE];
static uint8_t feed_buf[MAX_RECV_BUF_LEN + EXTRA_BUF_SPACE];
static size_t test_msg_len;

struct test_data {
	uint8_t *input_buf;
	size_t input_len;
	struct websocket_context *ctx;
};

static int test_recv_buf(uint8_t *feed_buf, size_t feed_len,
			 struct websocket_context *ctx,
			 uint32_t *msg_type, uint64_t *remaining,
			 uint8_t *recv_buf, size_t recv_len)
{
	static struct test_data test_data;
	int ctx_ptr;

	test_data.ctx = ctx;
	test_data.input_buf = feed_buf;
	test_data.input_len = feed_len;

	ctx_ptr = POINTER_TO_INT(&test_data);

	return websocket_recv_msg(ctx_ptr, recv_buf, recv_len,
				  msg_type, remaining, 0);
}

/* Websocket frame, header is 6 bytes, FIN bit is set, opcode is text (1),
 * payload length is 12, masking key is e17e8eb9,
 * unmasked data is "test message"
 */
static const unsigned char frame1[] = {
	0x81, 0x8c, 0xe1, 0x7e, 0x8e, 0xb9, 0x95, 0x1b,
	0xfd, 0xcd, 0xc1, 0x13, 0xeb, 0xca, 0x92, 0x1f,
	0xe9, 0xdc
};

static const unsigned char frame1_msg[] = {
	/* Null added for printing purposes */
	't', 'e', 's', 't', ' ', 'm', 'e', 's', 's', 'a', 'g', 'e', '\0'
};

/* The frame2 has frame1 + frame1. The idea is to test a case where we
 * read full frame1 and then part of second frame
 */
static const unsigned char frame2[] = {
	0x81, 0x8c, 0xe1, 0x7e, 0x8e, 0xb9, 0x95, 0x1b,
	0xfd, 0xcd, 0xc1, 0x13, 0xeb, 0xca, 0x92, 0x1f,
	0xe9, 0xdc,
	0x81, 0x8c, 0xe1, 0x7e, 0x8e, 0xb9, 0x95, 0x1b,
	0xfd, 0xcd, 0xc1, 0x13, 0xeb, 0xca, 0x92, 0x1f,
	0xe9, 0xdc
};

#define FRAME1_HDR_SIZE (sizeof(frame1) - (sizeof(frame1_msg) - 1))

static void test_recv(int count)
{
	struct websocket_context ctx;
	uint32_t msg_type = -1;
	uint64_t remaining = -1;
	int total_read = 0;
	int ret, i, left;

	memset(&ctx, 0, sizeof(ctx));

	ctx.tmp_buf = temp_recv_buf;
	ctx.tmp_buf_len = sizeof(temp_recv_buf);
	ctx.tmp_buf_pos = 0;

	memcpy(feed_buf, &frame1, sizeof(frame1));

	NET_DBG("Reading %d bytes at a time, frame %zd hdr %zd", count,
		sizeof(frame1), FRAME1_HDR_SIZE);

	/* We feed the frame N byte(s) at a time */
	for (i = 0; i < sizeof(frame1) / count; i++) {
		ret = test_recv_buf(&feed_buf[i * count], count,
				    &ctx, &msg_type, &remaining,
				    recv_buf + total_read,
				    sizeof(recv_buf) - total_read);
		if (count < 7 && (i * count) < FRAME1_HDR_SIZE) {
			zassert_equal(ret, -EAGAIN,
				      "[%d] Header parse failed (ret %d)",
				      i * count, ret);
		} else {
			total_read += ret;
		}
	}

	/* Read any remaining data */
	left = sizeof(frame1) % count;
	if (left > 0) {
		/* Some leftover bytes are still there */
		ret = test_recv_buf(&feed_buf[sizeof(frame1) - left], left,
				    &ctx, &msg_type, &remaining,
				    recv_buf + total_read,
				    sizeof(recv_buf) - total_read);
		zassert_true(ret <= (sizeof(recv_buf) - total_read),
			     "Invalid number of bytes read (%d)", ret);
		total_read += ret;
		zassert_equal(total_read, sizeof(frame1) - FRAME1_HDR_SIZE,
			      "Invalid amount of data read (%d)", ret);

	} else if (total_read < (sizeof(frame1) - FRAME1_HDR_SIZE)) {
		/* We read the whole message earlier, but we have parsed
		 * only part of the message. Parse the reset of the message
		 * here.
		 */
		ret = test_recv_buf(&feed_buf[FRAME1_HDR_SIZE + total_read],
			    sizeof(frame1) - FRAME1_HDR_SIZE - total_read,
				    &ctx, &msg_type, &remaining,
				    recv_buf + total_read,
				    sizeof(recv_buf) - total_read);
		total_read += ret;
		zassert_equal(total_read, sizeof(frame1) - FRAME1_HDR_SIZE,
			      "Invalid amount of data read (%d)", ret);
	}

	zassert_mem_equal(recv_buf, frame1_msg, sizeof(frame1_msg) - 1,
			  "Invalid message, should be '%s' was '%s'",
			  frame1_msg, recv_buf);

	zassert_equal(remaining, 0, "Msg not empty");
}

static void test_recv_1_byte(void)
{
	test_recv(1);
}

static void test_recv_2_byte(void)
{
	test_recv(2);
}

static void test_recv_3_byte(void)
{
	test_recv(3);
}

static void test_recv_6_byte(void)
{
	test_recv(6);
}

static void test_recv_7_byte(void)
{
	test_recv(7);
}

static void test_recv_8_byte(void)
{
	test_recv(8);
}

static void test_recv_9_byte(void)
{
	test_recv(9);
}

static void test_recv_10_byte(void)
{
	test_recv(10);
}

static void test_recv_12_byte(void)
{
	test_recv(12);
}

static void test_recv_whole_msg(void)
{
	test_recv(sizeof(frame1));
}

static void test_recv_2(int count)
{
	struct websocket_context ctx;
	uint32_t msg_type = -1;
	uint64_t remaining = -1;
	int total_read = 0;
	int ret;

	memset(&ctx, 0, sizeof(ctx));

	ctx.tmp_buf = temp_recv_buf;
	ctx.tmp_buf_len = sizeof(temp_recv_buf);

	memcpy(feed_buf, &frame2, sizeof(frame2));

	NET_DBG("Reading %d bytes at a time, frame %zd hdr %zd", count,
		sizeof(frame2), FRAME1_HDR_SIZE);

	total_read = test_recv_buf(&feed_buf[0], count, &ctx, &msg_type,
				   &remaining, recv_buf, sizeof(recv_buf));

	zassert_mem_equal(recv_buf, frame1_msg, sizeof(frame1_msg) - 1,
			  "Invalid message, should be '%s' was '%s'",
			  frame1_msg, recv_buf);

	zassert_equal(remaining, 0, "Msg not empty");

	/* Then read again, now we should get EAGAIN as the second message
	 * header is partially read.
	 */
	ret = test_recv_buf(&feed_buf[sizeof(frame1)], count, &ctx, &msg_type,
			    &remaining, recv_buf, sizeof(recv_buf));

	zassert_equal(ret, sizeof(frame1_msg) - 1,
		      "2nd header parse failed (ret %d)", ret);

	zassert_equal(remaining, 0, "Msg not empty");
}

static void test_recv_two_msg(void)
{
	test_recv_2(sizeof(frame1) + FRAME1_HDR_SIZE / 2);
}

int verify_sent_and_received_msg(struct msghdr *msg, bool split_msg)
{
	static struct websocket_context ctx;
	uint32_t msg_type = -1;
	uint64_t remaining = -1;
	size_t split_len = 0, total_read = 0;
	int ret;

	memset(&ctx, 0, sizeof(ctx));

	ctx.tmp_buf = temp_recv_buf;
	ctx.tmp_buf_len = sizeof(temp_recv_buf);

	/* Read first the header */
	ret = test_recv_buf(msg->msg_iov[0].iov_base,
			    msg->msg_iov[0].iov_len,
			    &ctx, &msg_type, &remaining,
			    recv_buf, sizeof(recv_buf));
	zassert_equal(ret, -EAGAIN, "Msg header not found");

	/* Then the first split if it is enabled */
	if (split_msg) {
		split_len = msg->msg_iov[1].iov_len / 2;

		ret = test_recv_buf(msg->msg_iov[1].iov_base,
				    split_len,
				    &ctx, &msg_type, &remaining,
				    recv_buf, sizeof(recv_buf));
		zassert_true(ret > 0, "Cannot read data (%d)", ret);

		total_read = ret;
	}

	/* Then the data */
	while (remaining > 0) {
		ret = test_recv_buf((uint8_t *)msg->msg_iov[1].iov_base +
								total_read,
				    msg->msg_iov[1].iov_len - total_read,
				    &ctx, &msg_type, &remaining,
				    recv_buf, sizeof(recv_buf));
		zassert_true(ret > 0, "Cannot read data (%d)", ret);

		if (memcmp(recv_buf, lorem_ipsum + total_read, ret) != 0) {
			LOG_HEXDUMP_ERR(lorem_ipsum + total_read, ret,
					"Received message should be");
			LOG_HEXDUMP_ERR(recv_buf, ret, "but it was instead");
			zassert_true(false, "Invalid received message "
				     "after %d bytes", total_read);
		}

		total_read += ret;
	}

	zassert_equal(total_read, test_msg_len,
		      "Msg body not valid, received %d instead of %zd",
		      total_read, test_msg_len);

	NET_DBG("Received %zd header and %zd body",
		msg->msg_iov[0].iov_len, total_read);

	return msg->msg_iov[0].iov_len + total_read;
}

static void test_send_and_recv_lorem_ipsum(void)
{
	static struct websocket_context ctx;
	int ret;

	memset(&ctx, 0, sizeof(ctx));

	ctx.tmp_buf = temp_recv_buf;
	ctx.tmp_buf_len = sizeof(temp_recv_buf);

	test_msg_len = sizeof(lorem_ipsum) - 1;

	ret = websocket_send_msg(POINTER_TO_INT(&ctx),
				 lorem_ipsum, test_msg_len,
				 WEBSOCKET_OPCODE_DATA_TEXT, true, true,
				 SYS_FOREVER_MS);
	zassert_equal(ret, test_msg_len,
		      "Should have sent %zd bytes but sent %d instead",
		      test_msg_len, ret);
}

static void test_recv_two_large_split_msg(void)
{
	static struct websocket_context ctx;
	int ret;

	memset(&ctx, 0, sizeof(ctx));

	ctx.tmp_buf = temp_recv_buf;
	ctx.tmp_buf_len = sizeof(temp_recv_buf);

	test_msg_len = sizeof(lorem_ipsum) - 1;

	ret = websocket_send_msg(POINTER_TO_INT(&ctx), lorem_ipsum,
				 test_msg_len, WEBSOCKET_OPCODE_DATA_TEXT,
				 false, true, SYS_FOREVER_MS);
	zassert_equal(ret, test_msg_len,
		      "1st should have sent %zd bytes but sent %d instead",
		      test_msg_len, ret);
}

void test_main(void)
{
	k_thread_system_pool_assign(k_current_get());

	ztest_test_suite(websocket,
			 ztest_unit_test(test_recv_1_byte),
			 ztest_unit_test(test_recv_2_byte),
			 ztest_unit_test(test_recv_3_byte),
			 ztest_unit_test(test_recv_6_byte),
			 ztest_unit_test(test_recv_7_byte),
			 ztest_unit_test(test_recv_8_byte),
			 ztest_unit_test(test_recv_9_byte),
			 ztest_unit_test(test_recv_10_byte),
			 ztest_unit_test(test_recv_12_byte),
			 ztest_unit_test(test_recv_whole_msg),
			 ztest_unit_test(test_recv_two_msg),
			 ztest_unit_test(test_send_and_recv_lorem_ipsum),
			 ztest_unit_test(test_recv_two_large_split_msg)
		);

	ztest_run_test_suite(websocket);
}
