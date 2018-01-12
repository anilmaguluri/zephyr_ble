/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <xtensa_api.h>
#include <xtensa/xtruntime.h>
#include <logging/sys_log.h>
#include <board.h>
#include <irq_nextlevel.h>
#include <xtensa/hal.h>

void _soc_irq_enable(u32_t irq)
{
	struct device *dev_cavs, *dev_ictl;

	switch (XTENSA_IRQ_NUMBER(irq)) {
	case CAVS_ICTL_0_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_0_NAME);
		break;
	case CAVS_ICTL_1_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_1_NAME);
		break;
	case CAVS_ICTL_2_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_2_NAME);
		break;
	case CAVS_ICTL_3_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_3_NAME);
		break;
	default:
		/* regular interrupt */
		_xtensa_irq_enable(XTENSA_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_cavs) {
		SYS_LOG_DBG("board: CAVS device binding failed\n");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in either CAVS interrupt logic or DW interrupt controller
	 */
	_xtensa_irq_enable(XTENSA_IRQ_NUMBER(irq));

	switch (CAVS_IRQ_NUMBER(irq)) {
	case DW_ICTL_IRQ_CAVS_OFFSET:
		dev_ictl = device_get_binding(CONFIG_DW_ICTL_NAME);
		break;
	default:
		/* The source of the interrupt is in CAVS interrupt logic */
		irq_enable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_ictl) {
		SYS_LOG_DBG("board: DW intr_control device binding failed\n");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in DW interrupt controller
	 */
	irq_enable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

	/* Manipulate the relevant bit in the interrupt controller
	 * register as needed
	 */
	irq_enable_next_level(dev_ictl, INTR_CNTL_IRQ_NUM(irq));
}

void _soc_irq_disable(u32_t irq)
{
	struct device *dev_cavs, *dev_ictl;

	switch (XTENSA_IRQ_NUMBER(irq)) {
	case CAVS_ICTL_0_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_0_NAME);
		break;
	case CAVS_ICTL_1_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_1_NAME);
		break;
	case CAVS_ICTL_2_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_2_NAME);
		break;
	case CAVS_ICTL_3_IRQ:
		dev_cavs = device_get_binding(CONFIG_CAVS_ICTL_3_NAME);
		break;
	default:
		/* regular interrupt */
		_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		return;
	}

	if (!dev_cavs) {
		SYS_LOG_DBG("board: CAVS device binding failed\n");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in either CAVS interrupt logic or DW interrupt controller
	 */

	switch (CAVS_IRQ_NUMBER(irq)) {
	case DW_ICTL_IRQ_CAVS_OFFSET:
		dev_ictl = device_get_binding(CONFIG_DW_ICTL_NAME);
		break;
	default:
		/* The source of the interrupt is in CAVS interrupt logic */
		irq_disable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

		/* Disable the parent IRQ if all children are disabled */
		if (!irq_is_enabled_next_level(dev_cavs)) {
			_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		}
		return;
	}

	if (!dev_ictl) {
		SYS_LOG_DBG("board: DW intr_control device binding failed\n");
		return;
	}

	/* If the control comes here it means the specified interrupt
	 * is in DW interrupt controller.
	 * Manipulate the relevant bit in the interrupt controller
	 * register as needed
	 */
	irq_disable_next_level(dev_ictl, INTR_CNTL_IRQ_NUM(irq));

	/* Disable the parent IRQ if all children are disabled */
	if (!irq_is_enabled_next_level(dev_ictl)) {
		irq_disable_next_level(dev_cavs, CAVS_IRQ_NUMBER(irq));

		if (!irq_is_enabled_next_level(dev_cavs)) {
			_xtensa_irq_disable(XTENSA_IRQ_NUMBER(irq));
		}
	}
}

/* Setup DMA ownership registers */
void setup_ownership_dma0(void)
{
	*(volatile u16_t *)CAVS_DMA0_OWNERSHIP_REG = 0x80FF;
}

void setup_ownership_dma1(void)
{
	*(volatile u16_t *)CAVS_DMA1_OWNERSHIP_REG = 0x80FF;
}

void setup_ownership_dma2(void)
{
	*(volatile u16_t *)CAVS_DMA2_OWNERSHIP_REG = 0x80FF;
}

void dcache_writeback_region(void *addr, size_t size)
{
	xthal_dcache_region_writeback(addr, size);
}
