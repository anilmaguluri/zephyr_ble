/*
 * Copyright (c) 1997-2010, 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>

/**
 * @brief Return the kernel version of the present build
 *
 * The kernel version is a four-byte value, whose format is described in the
 * file "version.h".
 *
 * @return kernel version
 */
uint32_t sys_kernel_version_get(void)
{
	return KERNELVERSION;
}
