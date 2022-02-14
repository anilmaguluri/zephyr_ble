/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <ztest.h>
#include <ztress.h>
#include <sys/ring_buffer.h>
#include <sys/mutex.h>
#include <random/rand32.h>

/**
 * @defgroup lib_ringbuffer_tests Ringbuffer
 * @ingroup all_tests
 * @{
 * @}
 */

#define STACKSIZE (512 + CONFIG_TEST_EXTRA_STACKSIZE)

#define RINGBUFFER			256
#define LENGTH				64
#define VALUE				0xb
#define TYPE				0xc

static ZTEST_BMEM SYS_MUTEX_DEFINE(mutex);
RING_BUF_ITEM_DECLARE(ringbuf, RINGBUFFER);
static uint32_t output[LENGTH];
static uint32_t databuffer1[LENGTH];
static uint32_t databuffer2[LENGTH];

static void data_write(uint32_t *input)
{
	sys_mutex_lock(&mutex, K_FOREVER);
	int ret = ring_buf_item_put(&ringbuf, TYPE, VALUE,
				   input, LENGTH);
	zassert_equal(ret, 0, NULL);
	sys_mutex_unlock(&mutex);
}

static void data_read(uint32_t *output)
{
	uint16_t type;
	uint8_t value, size32 = LENGTH;
	int ret;

	sys_mutex_lock(&mutex, K_FOREVER);
	ret = ring_buf_item_get(&ringbuf, &type, &value, output, &size32);
	sys_mutex_unlock(&mutex);

	zassert_equal(ret, 0, NULL);
	zassert_equal(type, TYPE, NULL);
	zassert_equal(value, VALUE, NULL);
	zassert_equal(size32, LENGTH, NULL);
	if (output[0] == 1) {
		zassert_equal(memcmp(output, databuffer1, size32), 0, NULL);
	} else {
		zassert_equal(memcmp(output, databuffer2, size32), 0, NULL);
	}
}

static bool user_handler(void *user_data, uint32_t iter_cnt, bool last, int prio)
{
	uintptr_t id = (uintptr_t)user_data;
	uint32_t *buffer = id ? databuffer2 : databuffer1;

	if (iter_cnt == 0) {
		for (int i = 0; i < LENGTH; i++) {
			buffer[i] = 1;
		}
	}

	/* Try to write data into the ringbuffer */
	data_write(buffer);
	/* Try to get data from the ringbuffer and check */
	data_read(output);

	return true;
}

/**
 * @brief Test that prevent concurrent writing
 * operations by using a mutex
 *
 * @details Define a ring buffer and a mutex,
 * and then spawn two threads to read and
 * write the same buffer at the same time to
 * check the integrity of data reading and writing.
 *
 * @ingroup lib_ringbuffer_tests
 */
void test_ringbuffer_concurrent(void)
{
	ztress_set_timeout(K_MSEC(1000));
	ZTRESS_EXECUTE(ZTRESS_THREAD(user_handler, (void *)0, 0, 0, Z_TIMEOUT_TICKS(20)),
		       ZTRESS_THREAD(user_handler, (void *)1, 0, 10, Z_TIMEOUT_TICKS(20)));
}

static bool produce_cpy(void *user_data, uint32_t iter_cnt, bool last, int prio)
{
	static int cnt;
	uint8_t buf[3];
	uint32_t len;

	if (iter_cnt == 0) {
		cnt = 0;
	}

	for (int i = 0; i < sizeof(buf); i++) {
		buf[i] = (uint8_t)cnt++;
	}

	len = ring_buf_put(&ringbuf, buf, sizeof(buf));
	cnt -= (sizeof(buf) - len);

	return true;
}

static bool consume_cpy(void *user_data, uint32_t iter_cnt, bool last, int prio)
{
	static int cnt;
	uint8_t buf[3];
	uint32_t len;

	if (iter_cnt == 0) {
		cnt = 0;
	}

	len = ring_buf_get(&ringbuf, buf, sizeof(buf));
	for (int i = 0; i < len; i++) {
		zassert_equal(buf[i], (uint8_t)cnt, NULL);
		cnt++;
	}

	return true;
}

static bool produce_item(void *user_data, uint32_t cnt, bool last, int prio)
{
	int err;
	static uint32_t pcnt;
	uint32_t buf[2];

	if (cnt == 0) {
		pcnt = 0;
	}

	err = ring_buf_item_put(&ringbuf, (uint16_t)pcnt, VALUE, buf, 2);
	if (err == 0) {
		pcnt++;
	}

	return true;
}

static bool consume_item(void *user_data, uint32_t cnt, bool last, int prio)
{
	int err;
	static uint32_t pcnt;
	uint32_t data[2];
	uint16_t type;
	uint8_t value;
	uint8_t size32 = ARRAY_SIZE(data);

	if (cnt == 0) {
		pcnt = 0;
	}

	err = ring_buf_item_get(&ringbuf, &type, &value, data, &size32);
	if (err == 0) {
		zassert_equal(value, VALUE, NULL);
		zassert_equal(type, (uint16_t)pcnt, NULL);
		pcnt++;
	} else if (err == -EMSGSIZE) {
		zassert_true(false, NULL);
	}

	return true;
}

static bool produce(void *user_data, uint32_t iter_cnt, bool last, int prio)
{
	static int cnt;
	static int wr = 8;
	uint32_t len;
	uint8_t *data;

	if (iter_cnt == 0) {
		cnt = 0;
	}

	len = ring_buf_put_claim(&ringbuf, &data, wr);
	if (len == 0) {
		len = ring_buf_put_claim(&ringbuf, &data, wr);
	}

	if (len == 0) {
		return true;
	}

	for (uint32_t i = 0; i < len; i++) {
		data[i] = cnt++;
	}

	wr++;
	if (wr == 15) {
		wr = 8;
	}

	int err = ring_buf_put_finish(&ringbuf, len);

	zassert_equal(err, 0, "cnt: %d", cnt);

	return true;
}

static bool consume(void *user_data, uint32_t iter_cnt, bool last, int prio)
{
	static int rd = 8;
	static int cnt;
	uint32_t len;
	uint8_t *data;

	if (iter_cnt == 0) {
		cnt = 0;
	}

	len = ring_buf_get_claim(&ringbuf, &data, rd);
	if (len == 0) {
		len = ring_buf_get_claim(&ringbuf, &data, rd);
	}

	if (len == 0) {
		return true;
	}

	for (uint32_t i = 0; i < len; i++) {
		zassert_equal(data[i], (uint8_t)cnt,
			      "Got %02x, exp: %02x", data[i], (uint8_t)cnt);
		cnt++;
	}

	rd++;
	if (rd == 15) {
		rd = 8;
	}

	int err = ring_buf_get_finish(&ringbuf, len);

	zassert_equal(err, 0, NULL);

	return true;
}

static void test_ztress(ztress_handler high_handler,
			ztress_handler low_handler,
			bool item_mode)
{
	uint8_t buf[32];
	uint32_t buf32[32];
	k_timeout_t timeout;

	if (item_mode) {
		ring_buf_item_init(&ringbuf, ARRAY_SIZE(buf32), buf32);
	} else {
		ring_buf_init(&ringbuf, ARRAY_SIZE(buf), buf);
	}

	/* Timeout after 5 seconds. */
	timeout =  (CONFIG_SYS_CLOCK_TICKS_PER_SEC < 10000) ? K_MSEC(1000) : K_MSEC(10000);

	ztress_set_timeout(timeout);
	ZTRESS_EXECUTE(ZTRESS_THREAD(high_handler, NULL, 0, 0, Z_TIMEOUT_TICKS(20)),
		       ZTRESS_THREAD(low_handler, NULL, 0, 2000, Z_TIMEOUT_TICKS(20)));
}

void test_ringbuffer_stress(ztress_handler produce_handler,
			    ztress_handler consume_handler,
			    bool item_mode)
{
	PRINT("Producing interrupts consuming\n");
	test_ztress(produce_handler, consume_handler, item_mode);

	PRINT("Consuming interrupts producing\n");
	test_ztress(consume_handler, produce_handler, item_mode);
}

/* Zero-copy API. Test is validating single producer, single consumer from
 * different priorities.
 */
void test_ringbuffer_zerocpy_stress(void)
{
	test_ringbuffer_stress(produce, consume, false);
}

/* Copy API. Test is validating single producer, single consumer from
 * different priorities.
 */
void test_ringbuffer_cpy_stress(void)
{
	test_ringbuffer_stress(produce_cpy, consume_cpy, false);
}

/* Item API. Test is validating single producer, single consumer from
 * different priorities.
 */
void test_ringbuffer_item_stress(void)
{
	test_ringbuffer_stress(produce_item, consume_item, true);
}
