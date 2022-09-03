/*
 * Copyright (c) 2022 TOKITA Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32V30X_CLOCKS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32V30X_CLOCKS_H_

#include "gd32-clocks-common.h"

/**
 * @name Register offsets
 * @{
 */

#define CH32_APB1PCEN_OFFSET       0x1CU
#define CH32_APB2PCEN_OFFSET       0x18U
#define CH32_AHBPCEN_OFFSET        0x14U

#define GD32_APB1PCEN_OFFSET      CH32_APB1PCEN_OFFSET
#define GD32_APB2PCEN_OFFSET      CH32_APB2PCEN_OFFSET
#define GD32_AHBPCEN_OFFSET       CH32_AHBPCEN_OFFSET

/** @} */

/**
 * @name Clock enable/disable definitions for peripherals
 * @{
 */

/* APB1 peripherals */
#define CH32_CLOCK_TIMER2     GD32_CLOCK_CONFIG(APB1PCEN, 0U)
#define CH32_CLOCK_TIMER3     GD32_CLOCK_CONFIG(APB1PCEN, 1U)
#define CH32_CLOCK_TIMER4     GD32_CLOCK_CONFIG(APB1PCEN, 2U)
#define CH32_CLOCK_TIMER5     GD32_CLOCK_CONFIG(APB1PCEN, 3U)
#define CH32_CLOCK_TIMER6     GD32_CLOCK_CONFIG(APB1PCEN, 4U)
#define CH32_CLOCK_TIMER7     GD32_CLOCK_CONFIG(APB1PCEN, 5U)
#define CH32_CLOCK_UART6      GD32_CLOCK_CONFIG(APB1PCEN, 6U)
#define CH32_CLOCK_UART7      GD32_CLOCK_CONFIG(APB1PCEN, 7U)
#define CH32_CLOCK_UART8      GD32_CLOCK_CONFIG(APB1PCEN, 8U)
/* Reserved 9 */
/* Reserved 10 */
#define CH32_CLOCK_WWDGT      GD32_CLOCK_CONFIG(APB1PCEN, 11U)
/* Reserved 12 */
/* Reserved 13 */
#define CH32_CLOCK_SPI1       GD32_CLOCK_CONFIG(APB1PCEN, 14U)
#define CH32_CLOCK_SPI2       GD32_CLOCK_CONFIG(APB1PCEN, 15U)
#define CH32_CLOCK_USART1     GD32_CLOCK_CONFIG(APB1PCEN, 17U)
#define CH32_CLOCK_USART2     GD32_CLOCK_CONFIG(APB1PCEN, 18U)
#define CH32_CLOCK_USART3     GD32_CLOCK_CONFIG(APB1PCEN, 19U)
#define CH32_CLOCK_UART4      GD32_CLOCK_CONFIG(APB1PCEN, 20U)
#define CH32_CLOCK_I2C1       GD32_CLOCK_CONFIG(APB1PCEN, 21U)
#define CH32_CLOCK_I2C2       GD32_CLOCK_CONFIG(APB1PCEN, 22U)
#define CH32_CLOCK_USBD       GD32_CLOCK_CONFIG(APB1PCEN, 23U)
/* Reserved 24 */
#define CH32_CLOCK_CAN1       GD32_CLOCK_CONFIG(APB1PCEN, 25U)
#define CH32_CLOCK_CAN2       GD32_CLOCK_CONFIG(APB1PCEN, 26U)
#define CH32_CLOCK_BKP        GD32_CLOCK_CONFIG(APB1PCEN, 27U)
#define CH32_CLOCK_PMU        GD32_CLOCK_CONFIG(APB1PCEN, 28U)
#define CH32_CLOCK_DAC        GD32_CLOCK_CONFIG(APB1PCEN, 29U)

/* APB2 peripherals */
#define CH32_CLOCK_AFIO       GD32_CLOCK_CONFIG(APB2PCEN, 0U)
/* Reserved 1 */
#define CH32_CLOCK_GPIOA      GD32_CLOCK_CONFIG(APB2PCEN, 2U)
#define CH32_CLOCK_GPIOB      GD32_CLOCK_CONFIG(APB2PCEN, 3U)
#define CH32_CLOCK_GPIOC      GD32_CLOCK_CONFIG(APB2PCEN, 4U)
#define CH32_CLOCK_GPIOD      GD32_CLOCK_CONFIG(APB2PCEN, 5U)
#define CH32_CLOCK_GPIOE      GD32_CLOCK_CONFIG(APB2PCEN, 6U)
/* Reserved 7 */
/* Reserved 8 */
#define CH32_CLOCK_ADC1       GD32_CLOCK_CONFIG(APB2PCEN, 9U)
#define CH32_CLOCK_ADC2       GD32_CLOCK_CONFIG(APB2PCEN, 10U)
#define CH32_CLOCK_TIMER1     GD32_CLOCK_CONFIG(APB2PCEN, 11U)
#define CH32_CLOCK_SPI1       GD32_CLOCK_CONFIG(APB2PCEN, 12U)
#define CH32_CLOCK_TIMER8     GD32_CLOCK_CONFIG(APB2PCEN, 13U)
#define CH32_CLOCK_USART1     GD32_CLOCK_CONFIG(APB2PCEN, 14U)
/* Reserved 15 */
/* Reserved 16 */
/* Reserved 17 */
/* Reserved 18 */
#define CH32_CLOCK_TIMER9     GD32_CLOCK_CONFIG(APB2PCEN, 19U)
#define CH32_CLOCK_TIMER10    GD32_CLOCK_CONFIG(APB2PCEN, 20U)

/* AHB peripherals */
#define CH32_CLOCK_DMA0       GD32_CLOCK_CONFIG(AHBPCEN, 0U)
#define CH32_CLOCK_DMA1       GD32_CLOCK_CONFIG(AHBPCEN, 1U)
#define CH32_CLOCK_SRAMSP     GD32_CLOCK_CONFIG(AHBPCEN, 2U)
/* Reserved 3 */
#define CH32_CLOCK_CRC        GD32_CLOCK_CONFIG(AHBPCEN, 6U)
/* Reserved 7 */
#define CH32_CLOCK_FSMC       GD32_CLOCK_CONFIG(AHBPCEN, 8U)
#define CH32_CLOCK_RNG        GD32_CLOCK_CONFIG(AHBPCEN, 9U)
#define CH32_CLOCK_SDIO       GD32_CLOCK_CONFIG(AHBPCEN, 10U)
#define CH32_CLOCK_USBHS      GD32_CLOCK_CONFIG(AHBPCEN, 11U)
#define CH32_CLOCK_OTG        GD32_CLOCK_CONFIG(AHBPCEN, 12U)
#define CH32_CLOCK_DVP        GD32_CLOCK_CONFIG(AHBPCEN, 13U)
#define CH32_CLOCK_ETHMAC     GD32_CLOCK_CONFIG(AHBPCEN, 14U)
#define CH32_CLOCK_ETHMACTX   GD32_CLOCK_CONFIG(AHBPCEN, 15U)
#define CH32_CLOCK_ETHMACRX   GD32_CLOCK_CONFIG(AHBPCEN, 16U)
#define CH32_CLOCK_BLEC       GD32_CLOCK_CONFIG(AHBPCEN, 16U) /* alias */
#define CH32_CLOCK_BLES       GD32_CLOCK_CONFIG(AHBPCEN, 17U)

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_CH32V30X_CLOCKS_H_ */
