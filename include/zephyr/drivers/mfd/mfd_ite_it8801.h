/*
 * Copyright (c) 2024 ITE Corporation. All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT8801_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT8801_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IC clock and power management controller register fields
 */
/* 0xf9: Gather interrupt status register */
#define IT8801_REG_GISR             0xf9
#define IT8801_REG_MASK_GISR_GKSIIS BIT(6)
/* 0xfb: Gather interrupt enable control register */
#define IT8801_REG_GIECR            0xfb
#define IT8801_REG_MASK_GKSIIE      BIT(3)

/*
 * General control register fields
 */
#define IT8801_REG_LBVIDR 0xfe
#define IT8801_REG_HBVIDR 0xff

struct it8801_vendor_id_t {
	uint8_t chip_id;
	uint8_t reg;
};

static const struct it8801_vendor_id_t it8801_id_verify[] = {
	{0x12, IT8801_REG_HBVIDR},
	{0x83, IT8801_REG_LBVIDR},
};

/*
 * SMbus interface register fields
 */
/* 0xfa: SMBus control register */
#define IT8801_REG_SMBCR    0xfa
#define IT8801_REG_MASK_ARE BIT(4)

/*
 * GPIO register fields
 */
#define IT8801_GPIOAFS_FUN1     0x0
#define IT8801_GPIOAFS_FUN2     0x01
#define IT8801_GPIOAFS_FUN3     0x02
/* GPIO control register */
/* GPIO direction */
#define IT8801_GPIODIR          BIT(5)
/* GPIO input and output type */
#define IT8801_GPIOIOT_OD       BIT(4)
#define IT8801_GPIOIOT_INT_FALL BIT(4)
#define IT8801_GPIOIOT_INT_RISE BIT(3)
/* GPIO polarity */
#define IT8801_GPIOPOL          BIT(2)
/* GPIO pull-down enable */
#define IT8801_GPIOPDE          BIT(1)
/* GPIO pull-up enable */
#define IT8801_GPIOPUE          BIT(0)

/*
 * Keyboard matrix scan controller register fields
 */
/* 0x40: Keyboard scan out mode control register */
#define IT8801_REG_MASK_KSOSDIC BIT(7)
#define IT8801_REG_MASK_KSE     BIT(6)
#define IT8801_REG_MASK_AKSOSC  BIT(5)

/*
 * PWM register fields
 */
#define PWM_IT8801_FREQ             32895
/* Control push-pull flag */
#define PWM_IT8801_PUSH_PULL        BIT(8)
/* 0x5f: PWM output open-drain disable register */
#define IT8801_REG_PWMODDSR         0x5f
/* PWM mode control register */
#define IT8801_PWMMCR_MCR_MASK      GENMASK(1, 0)
#define IT8801_PWMMCR_MCR_OFF       0
#define IT8801_PWMMCR_MCR_BLINKING  1
#define IT8801_PWMMCR_MCR_BREATHING 2
#define IT8801_PWMMCR_MCR_ON        3

/* Define the IT8801 MFD interrupt callback function handler */
typedef void (*it8801_callback_handler_t)(const struct device *dev);
/* Register the interrupt of IT8801 MFD callback function */
void mfd_it8801_register_interrupt_callback(const struct device *mfd, const struct device *child,
					    it8801_callback_handler_t cb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_ITE_IT8801_H_ */
