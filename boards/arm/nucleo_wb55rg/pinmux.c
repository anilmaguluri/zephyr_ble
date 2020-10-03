/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for NUCLEO-WB55RG board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay) && CONFIG_SERIAL
	{STM32_PIN_PB7, STM32WBX_PINMUX_FUNC_PB7_USART1_RX},
	{STM32_PIN_PB6, STM32WBX_PINMUX_FUNC_PB6_USART1_TX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart1), okay) && CONFIG_SERIAL
	{STM32_PIN_PA2, STM32WBX_PINMUX_FUNC_PA2_LPUART1_TX},
	{STM32_PIN_PA3, STM32WBX_PINMUX_FUNC_PA3_LPUART1_RX},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	{STM32_PIN_PB8, STM32WBX_PINMUX_FUNC_PB8_I2C1_SCL},
	{STM32_PIN_PB9, STM32WBX_PINMUX_FUNC_PB9_I2C1_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) && CONFIG_I2C
	{STM32_PIN_PC0, STM32WBX_PINMUX_FUNC_PC0_I2C3_SCL},
	{STM32_PIN_PC1, STM32WBX_PINMUX_FUNC_PC1_I2C3_SDA},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && CONFIG_SPI
#ifdef CONFIG_SPI_STM32_USE_HW_SS
	{STM32_PIN_PA4, STM32WBX_PINMUX_FUNC_PA4_SPI1_NSS},
#endif /* CONFIG_SPI_STM32_USE_HW_SS */
	{STM32_PIN_PA5, STM32WBX_PINMUX_FUNC_PA5_SPI1_SCK},
	{STM32_PIN_PA6, STM32WBX_PINMUX_FUNC_PA6_SPI1_MISO},
	{STM32_PIN_PA7, STM32WBX_PINMUX_FUNC_PA7_SPI1_MOSI},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay) && CONFIG_ADC
	{STM32_PIN_PC2, STM32WBX_PINMUX_FUNC_PC2_ADC1_IN3},
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usb), okay)
	{STM32_PIN_PA11, STM32WBX_PINMUX_FUNC_PA11_USB_DM},
	{STM32_PIN_PA12, STM32WBX_PINMUX_FUNC_PA12_USB_DP},
#endif
};

static int pinmux_stm32_init(const struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
