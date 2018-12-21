/*****************************************************************************
* Copyright 2001 - 2014 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
#ifndef __IPROC_SMC35x_LCD_H
#define __IPROC_SMC35x_LCD_H
#include <linux/mm_types.h>
ssize_t smc35x_lcd_io_write(const char *buf, size_t len);
ssize_t smc35x_lcd_io_read(char *buf, size_t len);
int smc35x_lcd_io_init(void);
void smc35x_lcd_io_cleanup(void);
#endif
