/*
 * Copyright (c) 2024 Daikin Comfort Technologies North America, Inc.
 * Copyright (c) 2021 Sateesh Kotapati
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_SOC_GECKO_DEV_INIT
#include "em_cmu.h"
#endif

LOG_MODULE_REGISTER(sparkfun_thing_plus_mgm240p,
CONFIG_BOARD_SPARKFUN_THING_PLUS_MATTER_MGM240P_LOG_LEVEL);

static int sparkfun_thing_plus_mgm240p_init_clocks(void);

void board_late_init_hook(void)
{
#ifdef CONFIG_SOC_GECKO_DEV_INIT
	sparkfun_thing_plus_mgm240p_init_clocks();
#endif
	static struct gpio_dt_spec wake_up_gpio_dev =
		GPIO_DT_SPEC_GET(DT_NODELABEL(wake_up_trigger), gpios);


	if (!gpio_is_ready_dt(&wake_up_gpio_dev)) {
		LOG_ERR("Wake-up GPIO device was not found!");
	}
	ret = gpio_pin_configure_dt(&wake_up_gpio_dev, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure wake-up GPIO device!");
	}
}

#ifdef CONFIG_SOC_GECKO_DEV_INIT
static int sparkfun_thing_plus_mgm240p_init_clocks(void)
{
	CMU_ClockSelectSet(cmuClock_SYSCLK, cmuSelect_HFRCODPLL);
#if defined(_CMU_EM01GRPACLKCTRL_MASK)
	CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFRCODPLL);
#endif
#if defined(_CMU_EM01GRPBCLKCTRL_MASK)
	CMU_ClockSelectSet(cmuClock_EM01GRPBCLK, cmuSelect_HFRCODPLL);
#endif
	CMU_ClockSelectSet(cmuClock_EM23GRPACLK, cmuSelect_LFRCO);
	CMU_ClockSelectSet(cmuClock_EM4GRPACLK, cmuSelect_LFRCO);
#if defined(RTCC_PRESENT)
	CMU_ClockSelectSet(cmuClock_RTCC, cmuSelect_LFRCO);
#endif
#if defined(SYSRTC_PRESENT)
	CMU_ClockSelectSet(cmuClock_SYSRTC, cmuSelect_LFRCO);
#endif
	CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_LFRCO);
#if WDOG_COUNT > 1
	CMU_ClockSelectSet(cmuClock_WDOG1, cmuSelect_LFRCO);
#endif

	return 0;
}
#endif
