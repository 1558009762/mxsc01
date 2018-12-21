/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef _SDIO_PLATFORM_H
#define _SDIO_PLATFORM_H

/*
 * SDIO device type
 */
enum sdio_devtype {
	SDIO_DEV_TYPE_SDMMC = 0,
	SDIO_DEV_TYPE_WIFI,
	SDIO_DEV_TYPE_EMMC,

	/* used for internal array indexing, DO NOT modify */
	SDIO_DEV_TYPE_MAX,
};

/*
 * SDIO WiFi GPIO configuration
 */
struct sdio_wifi_gpio_cfg {
	int reset;
	int shutdown;
	int reg;
	int host_wake;
};

struct sdio_platform_cfg {
	/* specify which SDIO device */
	unsigned id;

	/*
	* For boards without the SDIO pullup registers, data_pullup needs to set
	* to 1
	*/
	unsigned int data_pullup;

	/* for devices with 8-bit lines */
	int is_8bit;

	/* card detection GPIO, required for SD/MMC */
	int cd_gpio;
	enum sdio_devtype devtype;

	/* clocks */
	char *peri_clk_name;
	char *ahb_clk_name;
	char *sleep_clk_name;
	unsigned long peri_clk_rate;

	struct sdio_wifi_gpio_cfg wifi_gpio;
};

#endif  /* SDIO_PLATFORM_H */
