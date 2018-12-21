/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/socregs.h>

#define EARLY_ACC_BUF_SIZE 512

DECLARE_GLOBAL_DATA_PTR;

/* Find sub string 't' in string 's'. If found, return index within 's' pointing
 * to the last matching character. If not found, return -1
 */
static int find_str_indx(char *s, size_t len, char *t)
{
	int i = 0, j = 0;

	do {
		j = 0;
		while (s[i] == t[j]) {
			i++;
			j++;
		}
		if (t[j] == '\0') {
			/* var found */
			return(i - 1);
		}
		i++;
	} while (i < len); /* (s[i] != '\0'); */

	return -1;
}

/*
 * Note about EARLY_ACC_BUF_SIZE used in this function. Since this function
 * is called very early in the init sequence, the amount of stack is limited,
 * so we limit the amount of data read from flash to EARLY_ACC_BUF_SIZE, that
 * is, we do not read the entire sector from flash (on helix4 its 64K, as per
 * CONFIG_ENV_SIZE). Since this function is written to get baudrate, which is
 * always towards beginning of uboot env list, reading 1st 1K (or even less
 * amount) is sufficient. If this function needs to be used to read some other
 * env var, the 'env_ptr' can be adjusted by doing some approximation, depending
 * on the name of env var (uboot stores env vars in sorted order), so that
 * a large buffer size need not be allocated simply to find a env var
 */
int early_access_env_vars(void)
{
#ifdef CONFIG_ENV_IS_NOWHERE

	return -1;

#else /* CONFIG_ENV_IS_NOWHERE */
	volatile uint8_t *env_ptr;
	int i;
	char buf[EARLY_ACC_BUF_SIZE];
	unsigned long baud;
	unsigned long bauds[] = CONFIG_SYS_BAUDRATE_TABLE;
	int baud_arr_size = sizeof(bauds) / sizeof(bauds[0]);

#if defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND
    env_ptr = (volatile uint8_t *)(IPROC_NAND_MEM_BASE + CONFIG_ENV_OFFSET);
#elif defined (CONFIG_ENV_IS_IN_FLASH)
    env_ptr = (volatile uint8_t *)(IPROC_NOR_MEM_BASE + CONFIG_ENV_OFFSET);
#else
    env_ptr = (volatile uint8_t *)(IPROC_QSPI_MEM_BASE + CONFIG_ENV_OFFSET);
#endif

	for (i = 0; i < EARLY_ACC_BUF_SIZE; i++) {
		buf[i] = *(env_ptr + i);
	}
	baud = 0;
	i = find_str_indx(buf, sizeof(buf),  "baudrate=");
	if (i != -1) {
		/* Convert to integer value */
		while ((buf[i + 1] >= '0') && (buf[i + 1] <= '9')) {
			baud = baud * 10 + (buf[i + 1] - '0');
			i++;
		}
		/* Verify that the baudrate is a valid and a supported value */
		for (i = 0; i < baud_arr_size; i++) {
			if (baud == bauds[i]) {
				break;
			}
		}
		if (i < baud_arr_size) {
			/* Baud rate value is valid */
			gd->baudrate = baud;
		} else {
			return -1;
		}
	} else {
		return -1;
	}
	return 0;

#endif /* CONFIG_ENV_IS_NOWHERE */
}

__weak void reset_cpu(ulong ignored)
{
	/* Reset both iproc and switch. */
	*(unsigned int *)(DMU_CRU_RESET) = 0;

	while (1)
	;	/* loop forever till reset */
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache if not yet. I-cache is already enabled in start.S */
	if (!dcache_status()) {
		printf("Enabling D-cache\n");
		dcache_enable();
	}
}
#endif
