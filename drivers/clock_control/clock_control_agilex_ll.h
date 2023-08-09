/*
 * Copyright (c) 2019-2023, Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Intel Agilex clock manager low layer driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_CLOCK_AGILEX_LL_H_
#define ZEPHYR_INCLUDE_DRIVERS_CLOCK_AGILEX_LL_H_

#include <stdint.h>

#include <zephyr/sys/sys_io.h>

/* Clock manager registers offset */
#define CLKMGR_CTRL				0x00
#define CLKMGR_STAT				0x04
#define CLKMGR_INTRCLR				0x14

/* Clock manager main PLL group register offsets */
#define CLKMGR_MAINPLL_OFFSET			0x24
#define CLKMGR_MAINPLL_EN			0x00
#define CLKMGR_MAINPLL_BYPASS			0x0C
#define CLKMGR_MAINPLL_MPUCLK			0x18
#define CLKMGR_MAINPLL_NOCCLK			0x1c
#define CLKMGR_MAINPLL_NOCDIV			0x20
#define CLKMGR_MAINPLL_PLLGLOB			0x24
#define CLKMGR_MAINPLL_FDBCK			0x28
#define CLKMGR_MAINPLL_MEM			0x2C
#define CLKMGR_MAINPLL_MEMSTAT			0x30
#define CLKMGR_MAINPLL_PLLC0			0x34
#define CLKMGR_MAINPLL_PLLC1			0x38
#define CLKMGR_MAINPLL_VCOCALIB			0x3c
#define CLKMGR_MAINPLL_PLLC2			0x40
#define CLKMGR_MAINPLL_PLLC3			0x44
#define CLKMGR_MAINPLL_PLLM			0x48
#define CLKMGR_MAINPLL_LOSTLOCK			0x54

/* Clock manager peripheral group register offsets */
#define CLKMGR_PERPLL_OFFSET			0x7C
#define CLKMGR_PERPLL_EN			0x00
#define CLKMGR_PERPLL_BYPASS			0x0C
#define CLKMGR_PERPLL_EMACCTL			0x18
#define CLKMGR_PERPLL_GPIODIV			0x1C
#define CLKMGR_PERPLL_PLLGLOB			0x20
#define CLKMGR_PERPLL_FDBCK			0x24
#define CLKMGR_PERPLL_MEM			0x28
#define CLKMGR_PERPLL_MEMSTAT			0x2C
#define CLKMGR_PERPLL_PLLC0			0x30
#define CLKMGR_PERPLL_PLLC1			0x34
#define CLKMGR_PERPLL_VCOCALIB			0x38
#define CLKMGR_PERPLL_PLLC2			0x3c
#define CLKMGR_PERPLL_PLLC3			0x40
#define CLKMGR_PERPLL_PLLM			0x44
#define CLKMGR_PERPLL_LOSTLOCK			0x50

/* Clock manager control/Intel group register offsets */
#define CLKMGR_INTEL_OFFSET			0xD0
#define CLKMGR_INTEL_JTAG			0x00
#define CLKMGR_INTEL_EMACACTR			0x04
#define CLKMGR_INTEL_EMACBCTR			0x08
#define CLKMGR_INTEL_EMACPTPCTR			0x0C
#define CLKMGR_INTEL_GPIODBCTR			0x10
#define CLKMGR_INTEL_SDMMCCTR			0x14
#define CLKMGR_INTEL_S2FUSER0CTR		0x18
#define CLKMGR_INTEL_S2FUSER1CTR		0x1c
#define CLKMGR_INTEL_PSIREFCTR			0x20
#define CLKMGR_INTEL_EXTCNTRST			0x24

/* Membus */
#define CLKMGR_MEM_REQ				BIT(24)
#define CLKMGR_MEM_WR				BIT(25)
#define CLKMGR_MEM_ERR				BIT(26)
#define CLKMGR_MEM_WDAT_OFFSET			16

/* Clock manager macros */
#define CLKMGR_CTRL_BOOTMODE_SET_MSK		0x00000001
#define CLKMGR_STAT_BUSY_E_BUSY			0x1
#define CLKMGR_STAT_BUSY(x)			(((x) & 0x00000001U) >> 0)
#define CLKMGR_STAT_MAINPLLLOCKED(x)		(((x) & 0x00000100U) >> 8)
#define CLKMGR_STAT_PERPLLLOCKED(x)		(((x) & 0x00010000U) >> 16)
#define CLKMGR_INTRCLR_MAINLOCKLOST_SET_MSK	0x00000004U
#define CLKMGR_INTRCLR_PERLOCKLOST_SET_MSK	0x00000008U
#define CLKMGR_INTOSC_HZ			460000000U

/* Clock manager main PLL macros */
#define CLKMGR_MAINPLL_EN_RESET			0x000000FFU
#define CLKMGR_MAINPLL_L4SPDIV(x)		(((x) >> 16) & 0x3)

/* Clock manager peripheral PLL macros */
#define CLKMGR_PERPLL_EN_RESET			0x00000FFFU
#define CLKMGR_PERPLL_EN_SDMMCCLK		BIT(5)
#define CLKMGR_PERPLL_GPIODIV_GPIODBCLK_SET(x)	(((x) << 0) & 0x0000FFFFU)

/* Clock manager control/Intel macros */
#define CLKMGR_INTEL_EXTCNTRST_RESET		0xFF
#define CLKMGR_INTEL_SDMMC_CNT(x)		(((x) & 0x7FF) + 1)

/* Shared macros */
#define CLKMGR_PSRC(x)				(((x) & 0x00030000U) >> 16)
#define CLKMGR_PSRC_MAIN			0
#define CLKMGR_PSRC_PER				1

#define CLKMGR_PLLGLOB_PSRC_EOSC1		0x0
#define CLKMGR_PLLGLOB_PSRC_INTOSC		0x1
#define CLKMGR_PLLGLOB_PSRC_F2S			0x2

#define CLKMGR_PLLM_MDIV(x)			((x) & 0x000003FFU)
#define CLKMGR_PLLGLOB_PD_SET_MSK		0x00000001U
#define CLKMGR_PLLGLOB_RST_SET_MSK		0x00000002U

#define CLKMGR_PLLGLOB_REFCLKDIV(x)		(((x) & 0x00003F00U) >> 8)
#define CLKMGR_PLLGLOB_AREFCLKDIV(x)		(((x) & 0x00000F00U) >> 8)
#define CLKMGR_PLLGLOB_DREFCLKDIV(x)		(((x) & 0x00003000U) >> 12)

#define CLKMGR_VCOCALIB_HSCNT_SET(x)		(((x) << 0) & 0x000003FFU)
#define CLKMGR_VCOCALIB_MSCNT_SET(x)		(((x) << 16) & 0x00FF0000U)

#define CLKMGR_PLLC_DIV(x)			((x) & 0x7FF)

/**
 *  @brief  Initialize the low layer clock control driver
 *
 *  @param  base_addr    : Clock control device MMIO base address
 *
 *  @return void
 */
void clock_agilex_ll_init(mm_reg_t base_addr);

/**
 *  @brief  Get MPU(Micro Processor Unit) clock value
 *
 *  @param  void
 *
 *  @return returns MPU clock value
 */
uint32_t get_mpu_clk(void);

/**
 *  @brief  Get Watchdog peripheral clock value
 *
 *  @param  void
 *
 *  @return returns Watchdog clock value
 */
uint32_t get_wdt_clk(void);

/**
 *  @brief  Get UART peripheral clock value
 *
 *  @param  void
 *
 *  @return returns UART clock value
 */
uint32_t get_uart_clk(void);

/**
 *  @brief  Get MMC peripheral clock value
 *
 *  @param  void
 *
 *  @return returns MMC clock value
 */
uint32_t get_mmc_clk(void);

/**
 *  @brief  Get Timer peripheral clock value
 *
 *  @param  void
 *
 *  @return returns Timer clock value
 */
uint32_t get_timer_clk(void);

/**
 *  @brief  Get QSPI peripheral clock value
 *
 *  @param  void
 *
 *  @return returns QSPI clock value
 */
uint32_t get_qspi_clk(void);

#endif /* ZEPHYR_INCLUDE_DRIVERS_CLOCK_AGILEX_LL_H_ */
