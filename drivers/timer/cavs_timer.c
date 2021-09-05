/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <cavs-idc.h>
#include <cavs-shim.h>

/**
 * @file
 * @brief CAVS DSP Wall Clock Timer driver
 *
 * The CAVS DSP on Intel SoC has a timer with one counter and two compare
 * registers that is external to the CPUs. This timer is accessible from
 * all available CPU cores and provides a synchronized timer under SMP.
 */

#define TIMER		0
#define TIMER_IRQ	DSP_WCT_IRQ(TIMER)
#define CYC_PER_TICK	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
			/ CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_CYC		0xFFFFFFFFUL
#define MAX_TICKS	((MAX_CYC - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY	(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 100000)

BUILD_ASSERT(MIN_DELAY < CYC_PER_TICK);
BUILD_ASSERT(TIMER >= 0 && TIMER <= 1);

static struct k_spinlock lock;
static uint64_t last_count;

static void set_compare(uint64_t time)
{
	/* Disarm the comparator to prevent spurious triggers */
	CAVS_SHIM.dspwctcs &= ~DSP_WCT_CS_TA(TIMER);

	if (TIMER == 0) {
		CAVS_SHIM.dspwct0c_lo = (uint32_t)time;
		CAVS_SHIM.dspwct0c_hi = (uint32_t)(time >> 32);
	} else {
		CAVS_SHIM.dspwct1c_lo = (uint32_t)time;
		CAVS_SHIM.dspwct1c_hi = (uint32_t)(time >> 32);
	}

	/* Arm the timer */
	CAVS_SHIM.dspwctcs |= DSP_WCT_CS_TA(TIMER);
}

static uint64_t count(void)
{
	/* The count register is 64 bits, but we're a 32 bit CPU that
	 * can only read four bytes at a time, so a bit of care is
	 * needed to prevent racing against a wraparound of the low
	 * word.  Wrap the low read between two reads of the high word
	 * and make sure it didn't change.
	 */
	uint32_t hi0, hi1, lo;

	do {
		hi0 = CAVS_SHIM.dspwc_hi;
		lo = CAVS_SHIM.dspwc_lo;
		hi1 = CAVS_SHIM.dspwc_hi;
	} while (hi0 != hi1);

	return (((uint64_t)hi0) << 32) | lo;
}

static uint32_t count32(void)
{
	return CAVS_SHIM.dspwc_lo;
}

static void compare_isr(const void *arg)
{
	ARG_UNUSED(arg);
	uint64_t curr;
	uint32_t dticks;

	k_spinlock_key_t key = k_spin_lock(&lock);

	curr = count();

#ifdef CONFIG_SMP
	/* If we are too soon since last_count,
	 * this interrupt is likely the same interrupt
	 * event but being processed by another CPU.
	 * Since it has already been processed and
	 * ticks announced, skip it.
	 */
	if ((count32() - (uint32_t)last_count) < MIN_DELAY) {
		k_spin_unlock(&lock, key);
		return;
	}
#endif

	dticks = (uint32_t)((curr - last_count) / CYC_PER_TICK);

	/* Clear the triggered bit */
	CAVS_SHIM.dspwctcs |= DSP_WCT_CS_TT(TIMER);

	last_count += dticks * CYC_PER_TICK;

#ifndef CONFIG_TICKLESS_KERNEL
	uint64_t next = last_count + CYC_PER_TICK;

	if ((int64_t)(next - curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}
	set_compare(next);
#endif

	k_spin_unlock(&lock, key);

	sys_clock_announce(dticks);
}

/* Runs on core 0 only */
int sys_clock_driver_init(const struct device *dev)
{
	uint64_t curr = count();

	IRQ_CONNECT(TIMER_IRQ, 0, compare_isr, 0, 0);
	set_compare(curr + CYC_PER_TICK);
	last_count = curr;
	irq_enable(TIMER_IRQ);
	return 0;
}

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#ifdef CONFIG_TICKLESS_KERNEL
	ticks = ticks == K_TICKS_FOREVER ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t curr = count();
	uint64_t next;
	uint32_t adj, cyc = ticks * CYC_PER_TICK;

	/* Round up to next tick boundary */
	adj = (uint32_t)(curr - last_count) + (CYC_PER_TICK - 1);
	if (cyc <= MAX_CYC - adj) {
		cyc += adj;
	} else {
		cyc = MAX_CYC;
	}
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;
	next = last_count + cyc;

	if (((uint32_t)next - (uint32_t)curr) < MIN_DELAY) {
		next += CYC_PER_TICK;
	}

	set_compare(next);
	k_spin_unlock(&lock, key);
#endif
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = (count32() - (uint32_t)last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return count32();
}

/* Runs on secondary cores */
void smp_timer_init(void)
{
	/* This enables the Timer 0 (or 1) interrupt for CPU n.
	 *
	 * FIXME: Done in this way because we don't have an API
	 * to enable interrupts per CPU.
	 */
	CAVS_INTCTRL[arch_curr_cpu()->id].l2.clear = CAVS_L2_DWCT0;
	irq_enable(TIMER_IRQ);
}
