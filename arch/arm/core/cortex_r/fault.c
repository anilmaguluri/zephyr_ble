/*
 * Copyright (c) 2018 Lexmark International, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>

/**
 *
 * @brief Fault handler
 *
 * This routine is called when fatal error conditions are detected by hardware
 * and is responsible only for reporting the error. Once reported, it then
 * invokes the user provided routine _SysFatalErrorHandler() which is
 * responsible for implementing the error handling policy.
 *
 * This is a stub for more exception handling code to be added later.
 */
void _Fault(const NANO_ESF *esf)
{
	z_SysFatalErrorHandler(_NANO_ERR_HW_EXCEPTION, esf);
}

void z_FaultInit(void)
{
}
