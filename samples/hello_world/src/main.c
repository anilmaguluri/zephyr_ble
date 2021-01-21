/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

void main(void)
{
    printk("Hello from zephyr! %s %s %s\n", CONFIG_BOARD, __DATE__, __TIME__);
}
