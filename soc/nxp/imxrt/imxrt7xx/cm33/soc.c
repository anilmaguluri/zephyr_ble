/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for NXP RT7XX platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the RT7XX platforms.
 */

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/linker/sections.h>
#include <soc.h>
#include "fsl_power.h"
#include "fsl_clock.h"

#ifdef CONFIG_HAS_MCUX_CACHE
#include <fsl_cache.h>
#endif

#ifdef CONFIG_FLASH_MCUX_XSPI_XIP
#include "flash_clock_setup.h"
#endif

#define SYSOSC_SETTLING_US 220U      /*!< System oscillator settling time in us */
#define XTAL_SYS_CLK_HZ    24000000U /*!< xtal frequency in Hz */

const clock_main_pll_config_t g_mainPllConfig_clock_init = {
	.main_pll_src = kCLOCK_MainPllOscClk, /* OSC clock */
	.numerator = 0,   /* Numerator of the SYSPLL0 fractional loop divider is 0 */
	.denominator = 1, /* Denominator of the SYSPLL0 fractional loop divider is 1 */
	.main_pll_mult = kCLOCK_MainPllMult22 /* Divide by 22 */
};

const clock_audio_pll_config_t g_audioPllConfig_clock_init = {
	.audio_pll_src = kCLOCK_AudioPllOscClk, /* OSC clock */
	.numerator = 5040,    /* Numerator of the Audio PLL fractional loop divider is 0 */
	.denominator = 27000, /* Denominator of the Audio PLL fractional loop divider is 1 */
	.audio_pll_mult = kCLOCK_AudioPllMult22, /* Divide by 22 */
	.enableVcoOut = true};

void soc_reset_hook(void)
{
	SystemInit();
}

void soc_early_init_hook(void)
{
#if CONFIG_SOC_MIMXRT798S_CPU0
	const clock_fro_config_t froAutotrimCfg = {
		.targetFreq = 300000000U,
		.range = 50U,
		.trim1DelayUs = 15U,
		.trim2DelayUs = 15U,
		.refDiv = 1U,
		.enableInt = 0U,
		.coarseTrimEn = true,
	};

#ifndef CONFIG_IMXRT7XX_CODE_CACHE
	CACHE64_DisableCache(CACHE64_CTRL0);
#endif

	POWER_DisablePD(kPDRUNCFG_PD_LPOSC);

	/* Power up OSC */
	POWER_DisablePD(kPDRUNCFG_PD_SYSXTAL);
	/* Enable system OSC */
	CLOCK_EnableSysOscClk(true, true, SYSOSC_SETTLING_US);
	/* Sets external XTAL OSC freq */
	CLOCK_SetXtalFreq(XTAL_SYS_CLK_HZ);

	/* Make sure FRO1 is enabled. */
	POWER_DisablePD(kPDRUNCFG_PD_FRO1);

	/* Switch to FRO1 for safe configure. */
	CLOCK_AttachClk(kFRO1_DIV1_to_COMPUTE_BASE);
	CLOCK_AttachClk(kCOMPUTE_BASE_to_COMPUTE_MAIN);
	CLOCK_SetClkDiv(kCLOCK_DivCmptMainClk, 1U);
	CLOCK_AttachClk(kFRO1_DIV1_to_RAM);
	CLOCK_SetClkDiv(kCLOCK_DivComputeRamClk, 1U);
	CLOCK_AttachClk(kFRO1_DIV1_to_COMMON_BASE);
	CLOCK_AttachClk(kCOMMON_BASE_to_COMMON_VDDN);
	CLOCK_SetClkDiv(kCLOCK_DivCommonVddnClk, 1U);

#if CONFIG_FLASH_MCUX_XSPI_XIP
	/* Change to common_base clock(Sourced by FRO1). */
	flexspi_clock_safe_config();
#endif

	/* Ungate all FRO clock. */
	POWER_DisablePD(kPDRUNCFG_GATE_FRO0);
	/* Use close loop mode. */
	CLOCK_EnableFroClkFreqCloseLoop(FRO0, &froAutotrimCfg, kCLOCK_FroAllOutEn);
	/* Enable FRO0 MAX clock for all domains.*/
	CLOCK_EnableFro0ClkForDomain(kCLOCK_AllDomainEnable);

	CLOCK_InitMainPll(&g_mainPllConfig_clock_init);
	CLOCK_InitMainPfd(kCLOCK_Pfd0, 20U); /* 475 MHz */
	CLOCK_InitMainPfd(kCLOCK_Pfd1, 24U); /* 396 MHz */
	CLOCK_InitMainPfd(kCLOCK_Pfd2, 18U); /* 528 MHz */
	/* Main PLL kCLOCK_Pfd3 (528 * 18 / 19) = 500 MHz -need 2 div  -> 250 MHz*/
	CLOCK_InitMainPfd(kCLOCK_Pfd3, 19U);

	CLOCK_EnableMainPllPfdClkForDomain(kCLOCK_Pfd0, kCLOCK_AllDomainEnable);
	CLOCK_EnableMainPllPfdClkForDomain(kCLOCK_Pfd1, kCLOCK_AllDomainEnable);
	CLOCK_EnableMainPllPfdClkForDomain(kCLOCK_Pfd2, kCLOCK_AllDomainEnable);
	CLOCK_EnableMainPllPfdClkForDomain(kCLOCK_Pfd3, kCLOCK_AllDomainEnable);

	CLOCK_SetClkDiv(kCLOCK_DivCmptMainClk, 2U);
	CLOCK_AttachClk(kMAIN_PLL_PFD0_to_COMPUTE_MAIN); /* Switch to PLL 237.5 MHz */

	CLOCK_SetClkDiv(kCLOCK_DivMediaMainClk, 2U);
	CLOCK_AttachClk(kMAIN_PLL_PFD0_to_MEDIA_MAIN); /* Switch to PLL 237.5 MHz */

	CLOCK_SetClkDiv(kCLOCK_DivMediaVddnClk, 2U);
	CLOCK_AttachClk(kMAIN_PLL_PFD0_to_MEDIA_VDDN); /* Switch to PLL 237.5 MHz */

	CLOCK_SetClkDiv(kCLOCK_DivComputeRamClk, 2U);
	CLOCK_AttachClk(kMAIN_PLL_PFD0_to_RAM); /* Switch to PLL 237.5 MHz */

	CLOCK_SetClkDiv(kCLOCK_DivCommonVddnClk, 2U);
	CLOCK_AttachClk(kMAIN_PLL_PFD3_to_COMMON_VDDN); /* Switch to 250MHZ */

	/* Configure Audio PLL clock source. */
	CLOCK_InitAudioPll(&g_audioPllConfig_clock_init); /* 532.48MHZ */
	CLOCK_InitAudioPfd(kCLOCK_Pfd1, 24U);             /* 399.36MHz */
	CLOCK_InitAudioPfd(kCLOCK_Pfd3, 26U); /* Enable Audio PLL PFD3 clock to 368.64MHZ */
	CLOCK_EnableAudioPllPfdClkForDomain(kCLOCK_Pfd1, kCLOCK_AllDomainEnable);
	CLOCK_EnableAudioPllPfdClkForDomain(kCLOCK_Pfd3, kCLOCK_AllDomainEnable);

#if CONFIG_FLASH_MCUX_XSPI_XIP
	/* Call function flexspi_setup_clock() to set user configured clock for XSPI. */
	flexspi_setup_clock(XSPI0, 3U, 1U); /* Main PLL PDF1 DIV1. */
#endif                                      /* CONFIG_FLASH_MCUX_XSPI_XIP */

#elif CONFIG_SOC_MIMXRT798S_CPU1
	/* Power up OSC in case it's not enabled. */
	POWER_DisablePD(kPDRUNCFG_PD_SYSXTAL);
	/* Enable system OSC */
	CLOCK_EnableSysOscClk(true, true, SYSOSC_SETTLING_US);
	/* Sets external XTAL OSC freq */
	CLOCK_SetXtalFreq(XTAL_SYS_CLK_HZ);

	CLOCK_AttachClk(kFRO1_DIV3_to_SENSE_BASE);
	CLOCK_SetClkDiv(kCLOCK_DivSenseMainClk, 1);
	CLOCK_AttachClk(kSENSE_BASE_to_SENSE_MAIN);

	POWER_DisablePD(kPDRUNCFG_GATE_FRO2);
	CLOCK_EnableFroClkFreq(FRO2, 300000000U, kCLOCK_FroAllOutEn);

	CLOCK_EnableFro2ClkForDomain(kCLOCK_AllDomainEnable);

	CLOCK_AttachClk(kFRO2_DIV3_to_SENSE_BASE);
	CLOCK_SetClkDiv(kCLOCK_DivSenseMainClk, 1);
	CLOCK_AttachClk(kSENSE_BASE_to_SENSE_MAIN);
#endif /* CONFIG_SOC_MIMXRT798S_CPU0 */
}
