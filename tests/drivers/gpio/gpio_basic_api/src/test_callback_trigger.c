/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_gpio_basic_api
 * @{
 * @defgroup t_gpio_callback_trigger test_gpio_callback_trigger
 * @brief TestPurpose: verify zephyr gpio callback triggered
 * under different INT modes
 * @}
 */

#include "test_gpio.h"

static struct drv_data data;
static int cb_cnt;

static int pin_num(u32_t pins)
{
	int ret = 0;

	while (pins >>= 1) {
		ret++;
	}
	return ret;
}

static void callback(struct device *dev,
		     struct gpio_callback *gpio_cb, u32_t pins)
{
	const struct drv_data *dd = CONTAINER_OF(gpio_cb,
						 struct drv_data, gpio_cb);

	/*= checkpoint: pins should be marked with correct pin number bit =*/
	zassert_true(pin_num(pins) == PIN_IN, NULL);
	++cb_cnt;
	TC_PRINT("callback triggered: %d\n", cb_cnt);
	if ((cb_cnt == 1)
	    && (dd->mode == GPIO_INT_EDGE_BOTH)) {
		gpio_pin_toggle(dev, PIN_OUT);
	}
	if (cb_cnt >= MAX_INT_CNT) {
		gpio_pin_set(dev, PIN_OUT, 0);
		gpio_pin_interrupt_configure(dev, PIN_IN, GPIO_INT_DISABLE);
	}
}

static int test_callback(int mode)
{
	struct device *dev = device_get_binding(DEV_NAME);
	struct drv_data *drv_data = &data;

	gpio_pin_interrupt_configure(dev, PIN_IN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(dev, PIN_OUT, GPIO_INT_DISABLE);

	/* 1. set PIN_OUT to logical initial state inactive */
	u32_t out_flags = GPIO_OUTPUT_LOW;

	if ((mode & GPIO_INT_LOW_0) != 0) {
		out_flags = GPIO_OUTPUT_HIGH | GPIO_ACTIVE_LOW;
	}

	int rc = gpio_pin_configure(dev, PIN_OUT, out_flags);

	if (rc != 0) {
		TC_ERROR("PIN_OUT config fail: %d", rc);
		return TC_FAIL;
	}

	/* 2. configure PIN_IN callback and trigger condition */
	rc = gpio_pin_configure(dev, PIN_IN, GPIO_INPUT);
	if (rc != 0) {
		TC_ERROR("config PIN_IN fail: %d\n", rc);
		goto err_exit;
	}

	drv_data->mode = mode;
	gpio_init_callback(&drv_data->gpio_cb, callback, BIT(PIN_IN));
	rc = gpio_add_callback(dev, &drv_data->gpio_cb);
	if (rc == -ENOTSUP) {
		TC_PRINT("interrupts not supported\n");
		return TC_PASS;
	} else if (rc != 0) {
		TC_ERROR("set PIN_IN callback fail: %d\n", rc);
		return TC_FAIL;
	}

	/* 3. enable callback, trigger PIN_IN interrupt by operate PIN_OUT */
	cb_cnt = 0;
	rc = gpio_pin_interrupt_configure(dev, PIN_IN, mode | GPIO_INT_DEBOUNCE);
	if (rc == -ENOTSUP) {
		TC_PRINT("Mode %x not supported\n", mode);
		goto pass_exit;
	} else if (rc != 0) {
		TC_ERROR("config PIN_IN interrupt fail: %d\n", rc);
		goto err_exit;
	}
	k_sleep(100);
	gpio_pin_set(dev, PIN_OUT, 1);
	k_sleep(1000);
	(void)gpio_pin_interrupt_configure(dev, PIN_IN, GPIO_INT_DISABLE);

	/*= checkpoint: check callback is triggered =*/
	TC_PRINT("OUT init %x, IN cfg %x, cnt %d\n", out_flags, mode, cb_cnt);
	if (mode == GPIO_INT_EDGE_BOTH) {
		if (cb_cnt != 2) {
			TC_ERROR("double edge not detected\n");
			goto err_exit;
		}
		goto pass_exit;
	}
	if ((mode & GPIO_INT_EDGE) == GPIO_INT_EDGE) {
		if (cb_cnt != 1) {
			TC_ERROR("not trigger callback correctly\n");
			goto err_exit;
		}
		goto pass_exit;
	}

	if ((mode & GPIO_INT_EDGE) == GPIO_INT_LEVEL) {
		if (cb_cnt != MAX_INT_CNT) {
			TC_ERROR("not trigger callback correctly\n");
			goto err_exit;
		}
	}

pass_exit:
	gpio_remove_callback(dev, &drv_data->gpio_cb);
	return TC_PASS;

err_exit:
	gpio_remove_callback(dev, &drv_data->gpio_cb);
	return TC_FAIL;
}

/* export test cases */
void test_gpio_callback_variants(void)
{
	zassert_equal(test_callback(GPIO_INT_EDGE_FALLING), TC_PASS,
		      "falling edge failed");
	zassert_equal(test_callback(GPIO_INT_EDGE_RISING), TC_PASS,
		      "rising edge failed");
	zassert_equal(test_callback(GPIO_INT_EDGE_TO_ACTIVE), TC_PASS,
		      "edge active failed");
	zassert_equal(test_callback(GPIO_INT_EDGE_TO_INACTIVE), TC_PASS,
		      "edge inactive failed");
	zassert_equal(test_callback(GPIO_INT_LEVEL_HIGH), TC_PASS,
		      "level high failed");
	zassert_equal(test_callback(GPIO_INT_LEVEL_LOW), TC_PASS,
		      "level low failed");
	zassert_equal(test_callback(GPIO_INT_LEVEL_ACTIVE), TC_PASS,
		      "level active failed");
	zassert_equal(test_callback(GPIO_INT_LEVEL_INACTIVE), TC_PASS,
		      "level inactive failed");
	zassert_equal(test_callback(GPIO_INT_EDGE_BOTH), TC_PASS,
		      "edge both failed");
}
