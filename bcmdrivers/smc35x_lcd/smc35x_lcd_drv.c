/*
 * smc35x_lcd.c
 *
 * Copyright 2008 - 2014 Broadcom Corporation.  All rights reserved.
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
 *
 *
 * This code is only meant to show how to set up the SRAM controller, and select the
 * proper I/O muxing and etc....  While it is presented in a driver form, it is not
 * necessarily how customers will use it.
 *
 */

#include "smc35x_lcd.h"		/* SMC register definition */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#include <linux/config.h>
#else
#include <linux/platform_device.h>
#endif

#include <linux/module.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/sysfs.h>
#include <linux/bitops.h>
#include <linux/err.h>
//#include <mach/platform.h>
#include <mach/irqs.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <asm/uaccess.h>	/* for copy_from_user */
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
//#include <mach/reg_gpio.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
#include <linux/iproc.h>
#else
#endif
/*
 *  Module Parameters - Set default 
 *       8b data Bus at 8b
 *       240 x320 LCD 
 */
static char devname[] = "LCD-SRAM";

/*
 *  Function Prototypes
 */

static int smc35x_lcd_open(struct inode *inode, struct file *fp);
static int smc35x_lcd_close(struct inode *inode, struct file *fp);
static ssize_t smc35x_lcd_write(struct file *fp, const char *buf,
				size_t len, loff_t * f_pos);
static ssize_t smc35x_lcd_read(struct file *fp, char *buf, size_t len,
			       loff_t * f_pos);
static const struct file_operations smc35x_fops = {
	.owner = THIS_MODULE,
	.open = smc35x_lcd_open,
	.release = smc35x_lcd_close,
	.read = smc35x_lcd_read,
	.write = smc35x_lcd_write
};

static struct miscdevice smc35x_lcd_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = devname,
	.fops = &smc35x_fops,
};

/*
 */
static int
smc35x_lcd_open(struct inode *inode, struct file *fp)
{
	return 0;
}

/*
 */
static int
smc35x_lcd_close(struct inode *inode, struct file *fp)
{
	return 0;
}

/*
 * Output data to LCD
 */

static void
smc35x_lcd_print(uint8_t * dbuf, int n)
{
	return;
}

static ssize_t
smc35x_lcd_write(struct file *fp, const char *buf, size_t len,
		 loff_t * f_pos)
{
	/* 
	 * artificial code here to populate the SRAM with bogus data
	 */

	smc35x_lcd_io_write(buf, len);
	printk("func <%s>\t line <%d>\n", __func__, __LINE__);
	return len;
}

/*
 * Read data from LCD
 */

static ssize_t
smc35x_lcd_read(struct file *filp, char *buf, size_t len, loff_t * f_pos)
{
	smc35x_lcd_io_read(buf, len);
	return len;
}

/*
 * Initialize the SMC controller -
 * 
 * I/O map system control,  SMC control and memory space for LCD
 * While we allocate 64MB in [E8000000, EBFF_FFFF] for SRAM, we only map 4KB because
 * it is all we need.
 *
 * Set the timing parameter of SMC
 */

static __devinit int
smc35x_lcd_init(void)
{
	int ret = 0;
	int maxsz;

	//if (register_chrdev(major, devname, &smc35x_fops) < 0) {
	if (misc_register(&smc35x_lcd_miscdev)) {
		printk("register smc35x char dev failed\n");
	}
	smc35x_lcd_io_init();
	return ret;
}

static void
smc35x_lcd_cleanup(void)
{

	misc_deregister(&smc35x_lcd_miscdev);
	smc35x_lcd_io_cleanup();
}

module_init(smc35x_lcd_init);
module_exit(smc35x_lcd_cleanup);

MODULE_DESCRIPTION("Broadcom LCD Char Drivr for SRAM interface");
MODULE_AUTHOR("Your Name <youremail@broadcom.com>");
MODULE_LICENSE("GPL");
