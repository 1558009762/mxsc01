/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>

#define DMU_CRU_RESET       0x1803f200

void reset_cpu(ulong ignored)
{
	/* Reset both iproc and switch. */
    *(unsigned int *)(DMU_CRU_RESET) = 0;

	while (1)
		;	/* loop forever till reset */
}
