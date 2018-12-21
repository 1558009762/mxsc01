/*
* Copyright (C) 2013, Broadcom Corporation
* All Rights Reserved.
* 
* This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
* the contents of this file may not be disclosed to third parties, copied
* or duplicated in any form, in whole or in part, without the prior
* written permission of Broadcom Corporation.
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <mach/socregs-cygnus.h>
#include <plat/shm.h>

#define DEBUG
#define DTE_MAJOR 250
#define DTE_GPIO_INT

#define DTE_FIFO_SIZE	(4 * 1024)
#define DTE_TIMER_INTEV (125000)
#define DTE_N_FIFOEMPTY (5000)

#ifdef pr_info
#undef pr_info
#endif

#define pr_info(fmt, ...) do { \
	if (debug_on) \
		printk(KERN_INFO fmt, ##__VA_ARGS__); \
} while(0)

#define DTE_WRITEL(v, b, o) \
	writel((v), regbase.b + (o))

#define DTE_READL(b, o) \
	readl(regbase.b + (o))

#define DTE_DELAY(n) do { \
	if (n < 2000) \
		udelay(n); \
	else \
		mdelay(n/1000); \
} while(0)

struct dte_reg {
	void * __iomem crmudru;	
	void * __iomem chipdru;	
	void * __iomem gpio;	
	void * __iomem audio;	
	void * __iomem asiutop;	
	void * __iomem audioeav;	
};

struct bcm5830x_dte_dev {
	struct device *dev;
	struct cdev dte_cdev;
	struct class *dte_class;
	uint32_t users;
	int dteirq;
	int gpio14irq;
	int gpio15irq;
	int gpio22irq;
	uint32_t fifoof;
	uint32_t fifouf;
	uint32_t kfifoof;
	uint32_t mode;
};


/* use "insmod dte.ko debug=1" to enable debug */
static uint32_t debug_on = 0;
module_param_named(debug, debug_on, uint, 0644);
MODULE_PARM_DESC(debug, "debug info enable or not (default: 0)");

static struct kfifo recv_fifo;
static struct dte_reg regbase;
static DEFINE_MUTEX(dte_users_lock);
static DECLARE_WAIT_QUEUE_HEAD(dte_read_wq);

static void dte_audio_sw_reset(int bit)
{
	uint32_t value;

	/* ASIU SW RESET */
	value = DTE_READL(asiutop, ASIU_TOP_SW_RESET_CTRL_BASE);
	value &= ~(1 << bit);
	DTE_WRITEL(value, asiutop, ASIU_TOP_SW_RESET_CTRL_BASE);
	value |= 1 << bit;
	DTE_WRITEL(value, asiutop, ASIU_TOP_SW_RESET_CTRL_BASE);
	udelay(1000);
}

static void dte_asiu_audio_clock_gating_disable(int enable)
{
	uint32_t value;

	value = DTE_READL(asiutop, ASIU_TOP_CLK_GATING_CTRL_BASE);
	if (enable) {
		value |= 1 << ASIU_TOP_CLK_GATING_CTRL__AUD_CLK_GATE_EN;
	} else {
		value &= ~(1 << ASIU_TOP_CLK_GATING_CTRL__AUD_CLK_GATE_EN);
	}

	DTE_WRITEL(value, asiutop, ASIU_TOP_CLK_GATING_CTRL_BASE);
}

static void dte_asiu_audio_pad_enable(int enable)
{
	uint32_t value;

	value =  DTE_READL(asiutop, ASIU_TOP_PAD_CTRL_0_BASE);
	if (enable) {
		value |= 1 << ASIU_TOP_PAD_CTRL_0__aud_pad_oe_ctrl;
	} else {
		value &= ~(1 << ASIU_TOP_PAD_CTRL_0__aud_pad_oe_ctrl);
	}
	DTE_WRITEL(value, asiutop, ASIU_TOP_PAD_CTRL_0_BASE);
}

static void dte_asiu_iomux_select_external_inputs(int enable_audio)
{
	uint32_t val;
	uint32_t mask = 0x3;
	uint32_t v = 0;

	/* config gpio14 gpio15 gpio22 as AUDIO_DTE */
	if (enable_audio)
		v = 0x2;
	else
		v = 0;

	val = DTE_READL(chipdru, CRMU_IOMUX_CTRL1_BASE);
	val &= ~(mask << CRMU_IOMUX_CTRL1__CORE_TO_IOMUX_GPIO14_SEL_R);
	val |= v << CRMU_IOMUX_CTRL1__CORE_TO_IOMUX_GPIO14_SEL_R;
	val &= ~(mask << CRMU_IOMUX_CTRL1__CORE_TO_IOMUX_GPIO15_SEL_R);
	val |= v << CRMU_IOMUX_CTRL1__CORE_TO_IOMUX_GPIO15_SEL_R;
	DTE_WRITEL(val, chipdru, CRMU_IOMUX_CTRL1_BASE);

	val = DTE_READL(chipdru, CRMU_IOMUX_CTRL2_BASE);
	val &= ~(mask << CRMU_IOMUX_CTRL2__CORE_TO_IOMUX_GPIO22_SEL_R);
	val |= v << CRMU_IOMUX_CTRL2__CORE_TO_IOMUX_GPIO22_SEL_R;
	DTE_WRITEL(val, chipdru, CRMU_IOMUX_CTRL2_BASE);

	DTE_WRITEL(1<<15 | 1<<14 | 1<<22, gpio, ChipcommonG_GP_INT_CLR_BASE);
}

static void dte_asiu_audio_gen_pll_pwr_on(uint32_t crmu_pll_pwr_on)
{
	uint32_t rd_data = 0;
	uint32_t mask = 0;

	if (crmu_pll_pwr_on)
	{
		mask = 1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_BG;
		rd_data = (DTE_READL(crmudru, CRMU_PLL_AON_CTRL_BASE) & (~mask)) | mask;
		DTE_WRITEL(rd_data, crmudru, CRMU_PLL_AON_CTRL_BASE);
		DTE_DELAY(10000);

		mask = (1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_LDO)
			| (1<< CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_PLL);
		rd_data = (DTE_READL(crmudru, CRMU_PLL_AON_CTRL_BASE) & (~mask)) | mask;
		DTE_WRITEL(rd_data, crmudru, CRMU_PLL_AON_CTRL_BASE);
		DTE_DELAY(10000);

		mask = 1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_ISO_IN;
		rd_data = DTE_READL(crmudru, CRMU_PLL_AON_CTRL_BASE) & (~mask);
		DTE_WRITEL(rd_data, crmudru, CRMU_PLL_AON_CTRL_BASE);
		DTE_DELAY(2000);
	}
}

void dte_asiu_audio_gen_pll_group_id_config(uint32_t user_macro,
	uint32_t audio_ext_test_clock_en)
{
	uint32_t wr_data_user =0;
	uint32_t wr_data =0;
	uint32_t reset_val = 0;
	uint32_t rd_data = 0;
	uint32_t timeout = 120;

	if ( audio_ext_test_clock_en != 1)
	{
		/* User Macro select */
		wr_data_user = user_macro << AUD_FMM_IOP_PLL_0_MACRO_REG__MACRO_SELECT_R;
		DTE_WRITEL(wr_data_user, audio, AUD_FMM_IOP_PLL_0_MACRO_REG_BASE);

		/* resetb & post_resetb Assertion */
		reset_val = (1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD)
			|   (1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA);
		DTE_WRITEL(reset_val, audio, AUD_FMM_IOP_PLL_0_RESET_REG_BASE);
		DTE_DELAY(1000);

		/* resetb de-assertion */
		reset_val = (1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD)
			| (0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA);
		DTE_WRITEL(reset_val, audio, AUD_FMM_IOP_PLL_0_RESET_REG_BASE);
		DTE_DELAY(1000);

		/* post_resetb de-assertion */
		reset_val = (0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD)
			| (0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA);
		DTE_WRITEL(reset_val, audio, AUD_FMM_IOP_PLL_0_RESET_REG_BASE);

		do {
			rd_data = DTE_READL(audio, AUD_FMM_IOP_PLL_0_LOCK_STATUS_REG_BASE);
			timeout--;
		} while ((rd_data  != 0x00000001) && (timeout > 0));

		if ((rd_data  == 0x00000001) && (timeout!=0))
			pr_info("\nPLL Lock Is Asserted\n");
		else if (timeout == 0)
			pr_info("\nERROR: PLL Lock Timeout\n");
	}

	//Enable Audio As Secure Master
	rd_data = DTE_READL(audio, ASIU_TOP_AUD_AXI_SB_CTRL_BASE);
	wr_data = rd_data & (~(0x3f << ASIU_TOP_AUD_AXI_SB_CTRL__AUD_AWPROT_R));
	DTE_WRITEL(wr_data, audio, ASIU_TOP_AUD_AXI_SB_CTRL_BASE);
	pr_info("ASIU_TOP_AUD_AXI_SB_CTRL= %0x\n", wr_data);
}

static void dte_asiu_audio_dte_lts_div_1312 (uint32_t lts_ext_3_div,
	uint32_t lts_ext_4_div)
{
	uint32_t wr_data = 0;

	wr_data = (lts_ext_3_div << 16) | (lts_ext_4_div << 0);

	pr_info("EAV_DTE_DTE_LTS_DIV_1312_REG= %0x\n", wr_data);
	DTE_WRITEL(wr_data, audioeav, EAV_DTE_DTE_LTS_DIV_1312_REG_BASE);
}

static void dte_asiu_audio_dte_lts_div_1514 (uint32_t lts_ext_5_div,
	uint32_t lts_ext_6_div)
{
	uint32_t wr_data = 0;

	wr_data = (lts_ext_5_div << 16) | (lts_ext_6_div << 0);

	pr_info("EAV_DTE_DTE_LTS_DIV_14_REG= %0x\n", wr_data);
	DTE_WRITEL(wr_data, audioeav, EAV_DTE_DTE_LTS_DIV_14_REG_BASE);
}

static void dte_asiu_audio_dte_lts_src_en (uint32_t src_en_data,
	uint32_t audio_dte_both_edge_en )
{
	uint32_t wr_data = 0;
	uint32_t rd_data = 0;
	uint32_t rd_status;
	uint32_t i = 0;
	uint32_t mask = 0x3;

	pr_info("EAV_DTE_DTE_LTS_SRC_EN_REG= %0x\n", src_en_data);
	DTE_WRITEL(src_en_data, audioeav, EAV_DTE_DTE_LTS_SRC_EN_REG_BASE);

	/* empty fifo */
	do
	{
		if (i++ >= 16)
			break;
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		rd_status = DTE_READL(audioeav, EAV_DTE_DTE_LTS_CSR_REG_BASE);
	} while ((rd_status & 0x00000010) == 0) ;

	rd_data = DTE_READL(asiutop, ASIU_TOP_MISC_BASE);

	if (audio_dte_both_edge_en)
	{
		wr_data = rd_data | (mask << ASIU_TOP_MISC__dte_trig_both_edge_R);
	} else {
		wr_data = rd_data & (~(mask << ASIU_TOP_MISC__dte_trig_both_edge_R));
	}
	DTE_WRITEL(wr_data, asiutop, ASIU_TOP_MISC_BASE);
	pr_info("ASIU_TOP_MISC= %0x\n", wr_data);
}

static void dte_asiu_audio_dte_intr_config(uint32_t soi,
	uint32_t interval_length)
{
	uint32_t rd_data = 0;
	uint32_t wr_data = 0;

	pr_info("EAV_DTE_DTE_ILEN= %0x\n", interval_length);
	DTE_WRITEL(interval_length, audioeav, EAV_DTE_DTE_ILEN_REG_BASE);

	rd_data = DTE_READL(audioeav, EAV_DTE_DTE_NCO_TIME_REG_BASE);
	pr_info("EAV_DTE_DTE_NCO_TIME_REG_BASE = %0x\n", rd_data);

	wr_data = rd_data * 16 + soi;
	DTE_WRITEL(wr_data, audioeav, EAV_DTE_DTE_NEXT_SOI_REG_BASE);
	pr_info("EAV_DTE_DTE_NEXT_SOI= %0x\n", wr_data);
}

static void dte_asiu_audio_dte_empty_fifo(void)
{
	uint32_t i = 0;
	uint32_t rd_data = 0;
	uint32_t rd_status;

	do
	{
		if (i++ >= 16)
			break;
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		rd_status = DTE_READL(audioeav, EAV_DTE_DTE_LTS_CSR_REG_BASE);
	} while ((rd_status & 0x00000010) == 0) ;
}

/* enable dte clock & function block */
static int dte_init(void)
{
	uint32_t src_en;

	dte_audio_sw_reset(ASIU_TOP_SW_RESET_CTRL__AUD_SW_RESET_N);
	dte_asiu_audio_clock_gating_disable(1);
	dte_asiu_audio_pad_enable(1);
#ifdef DTE_GPIO_INT
	dte_asiu_iomux_select_external_inputs(0);
#else
	dte_asiu_iomux_select_external_inputs(1);
#endif
	dte_asiu_audio_gen_pll_pwr_on(1);
	dte_asiu_audio_gen_pll_group_id_config(2, 0);

	/* Track every Edge of the signal */
	dte_asiu_audio_dte_lts_div_1312(1, 1);
	dte_asiu_audio_dte_lts_div_1514(1, 1);

	/* External Input is the source bit 12, 13, 14 */
	src_en = (1 << 12) | (1 << 13) | (1 << 14);
	dte_asiu_audio_dte_lts_src_en(src_en, 1);

	/* set interrupt time interval */
	dte_asiu_audio_dte_intr_config(1000, DTE_TIMER_INTEV);

	DTE_WRITEL(1<< EAV_DTE_DTE_CTRL_REG__Interrupt, audioeav,
		EAV_DTE_DTE_CTRL_REG_BASE);
	return 0;
}

/* SysFS callback when debug file is read */
static ssize_t debug_show(struct device *pDev,
		struct device_attribute *attr, char *buf)
{
	struct bcm5830x_dte_dev *dte_dev = dev_get_drvdata(pDev);
	int len = 0;


	len += sprintf(buf+len, "DTE mode: 1-GPIO 2-DTE : %08x\n", dte_dev->mode);

	len += sprintf(buf+len, "DTE fifo underflow : %08x\n", dte_dev->fifouf);
	len += sprintf(buf+len, "DTE fifo overflow  : %08x\n", dte_dev->fifoof);
	len += sprintf(buf+len, "kfifo overflow     : %08x\n", dte_dev->kfifoof);

	len += sprintf(buf+len, "\n AUD_FMM_IOP_PLL_0_LOCK_STATUS_REG : %08x\n",
		 DTE_READL(audio, AUD_FMM_IOP_PLL_0_LOCK_STATUS_REG_BASE));

	len += sprintf(buf+len, "\n EAV_DTE_DTE_CTRL_REG_BASE : %08x\n",
		 DTE_READL(audio, EAV_DTE_DTE_CTRL_REG_BASE));

	len += sprintf(buf+len, "\n ChipcommonG_GP_DATA_IN  : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_DATA_IN_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_INT_MSTAT: %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_INT_MSTAT_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_INT_STAT : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_INT_STAT_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_OUT_EN   : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_OUT_EN_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_INT_MSK  : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_INT_MSK_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_INT_TYPE : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_INT_TYPE_BASE));
	len += sprintf(buf+len, " ChipcommonG_GP_INT_DE   : %08x\n",
		DTE_READL(gpio, ChipcommonG_GP_INT_DE_BASE));

	return len;
}

/* SysFS callback when debug file is written */
static ssize_t debug_store(struct device *pDev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (strstr(buf, "init"))
		dte_init();
	else if (strstr(buf, "debug_on"))
		debug_on = 1;
	else if (strstr(buf, "debug_off"))
		debug_on = 0;
	else if (strstr(buf, "clearint")) {
		DTE_WRITEL(1<< EAV_DTE_DTE_CTRL_REG__Interrupt, audioeav,
			EAV_DTE_DTE_CTRL_REG_BASE);
	}

	return count;
}

/* table of sysfs files we will create/destroy */
static struct device_attribute sysfs_list[] = {
	__ATTR(debug, S_IWUSR | S_IRUGO, debug_show, debug_store),
};

int cdev_dte_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct bcm5830x_dte_dev *dte_dev = container_of(cdev,
		struct bcm5830x_dte_dev, dte_cdev);
	int minor = iminor(inode);

	pr_info("--> %s dte_dev %x minor %d \n", __FUNCTION__, (int)dte_dev, minor);

	mutex_lock(&dte_users_lock);
	if (!dte_dev->users) {
		dte_dev->users = 1;
	} else {
		pr_info("dte already opened \n");
		mutex_unlock(&dte_users_lock);
		return -EBUSY;
	}
	mutex_unlock(&dte_users_lock);

	return 0;
}

int cdev_dte_release(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct bcm5830x_dte_dev *dte_dev = container_of(cdev,
		struct bcm5830x_dte_dev, dte_cdev);

	pr_info("--> %s dte_dev %x \n", __FUNCTION__, (int)dte_dev);

	mutex_lock(&dte_users_lock);
	dte_dev->users--;
	mutex_unlock(&dte_users_lock);
	
	return 0;
}

ssize_t cdev_dte_read(struct file *filp, char __user *buf, size_t count,
		loff_t *offp)
{
	int rc = 0;
	int copied = 0;

	pr_info("--> %s count %d  fifo len %d \n", __FUNCTION__, count,
		kfifo_len(&recv_fifo));

	if (!kfifo_is_empty(&recv_fifo)) {
		rc = kfifo_to_user(&recv_fifo, buf, count, &copied);
		if (!rc)
			return copied;
		else
			return rc;
	}

#ifndef BLOCKMODE
	if (filp->f_flags & O_NONBLOCK) {
		pr_info("O_NONBLOCK \n");
		return -EAGAIN;
	} else {
		/* app choose block mode */
		rc = wait_event_interruptible(dte_read_wq, !kfifo_is_empty(&recv_fifo));
		pr_info("block mode rc %x \n", rc);
		if (rc)	{
			goto out;
		}

		/* we got the data */
		rc = kfifo_to_user(&recv_fifo, buf, count, &copied);
		if (!rc)
			return copied;
		else
			return rc;
	}
#endif

out:
	return rc;
}

/* this interface is used for select */
static unsigned int cdev_dte_poll(struct file *file, poll_table *wait)
{
	pr_info("--> %s  fifo len %d \n", __FUNCTION__,  kfifo_len(&recv_fifo));
	poll_wait(file, &dte_read_wq, wait);
	pr_info("<-- %s  poll end \n", __FUNCTION__);

	if (!kfifo_len(&recv_fifo)) {
		return POLLOUT;
	} else {
		return POLLIN | POLLRDNORM;
	}
}

static const struct file_operations dte_cdev_fops = {
	.owner = THIS_MODULE,
	.open = cdev_dte_open,
	.release = cdev_dte_release,
	.read = cdev_dte_read,
	.poll = cdev_dte_poll,
};

static int *resource_init_one(struct platform_device *pdev,
	char *name)
{
	void * __iomem reg_base;
	struct resource *res = pdev->resource;

	res = iproc_platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (res == NULL)
	{
		dev_err(&pdev->dev, "Failed to get DTE resource %s!\n", name);
		return NULL;
	}

	reg_base = ioremap_nocache(res->start, resource_size(res));
	if (reg_base == NULL)
	{
		dev_err(&pdev->dev, "Failed to ioremap DTE resource %s!\n", name);
		return NULL;
	}

	return reg_base;
}

void bcm5830x_dte_resource_release(struct platform_device *pdev)
{
	iounmap(regbase.crmudru);
	iounmap(regbase.chipdru);
	iounmap(regbase.gpio);
	iounmap(regbase.audio);
	iounmap(regbase.asiutop);
	iounmap(regbase.audioeav);
}

static int bcm5830x_dte_resource_init(struct platform_device *pdev)
{
	void * __iomem crmudru;
	void * __iomem chipdru;
	void * __iomem gpio;
	void * __iomem audio;
	void * __iomem asiutop;
	void * __iomem audioeav;

	crmudru = resource_init_one(pdev, "crmudru");
	chipdru =  resource_init_one(pdev, "chipdru");
	gpio = resource_init_one(pdev, "gpio");
	audio = resource_init_one(pdev, "audio");
	asiutop = resource_init_one(pdev, "asiutop");
	audioeav = resource_init_one(pdev, "audioeav");

	if (crmudru == NULL || chipdru == NULL || gpio == NULL || audio == NULL
		|| asiutop == NULL || audioeav == NULL)
		return -ENXIO;

	regbase.crmudru = crmudru;
	regbase.chipdru = chipdru;
	regbase.gpio = gpio;
	regbase.audio = audio;
	regbase.asiutop = asiutop;
	regbase.audioeav = audioeav;

	pr_info("--> regbase: crmudru %x chipdru %x gpio %x audio %x asiutop %x\n",
		(uint32_t)crmudru, (uint32_t)chipdru, (uint32_t)gpio, (uint32_t)audio,
		(uint32_t)asiutop);

	return 0;
}

static irqreturn_t dte_isr(int irq, void *drv_ctx)
{
	uint32_t fifo_csr;
	uint32_t rd_data;
	uint32_t i = 0;
	int inlen;
	struct bcm5830x_dte_dev *dte_dev = drv_ctx;
	static uint32_t fifo_empty_cnt = 0;

	/* get data from dte fifo */
	do {
		fifo_csr = DTE_READL(audioeav, EAV_DTE_DTE_LTS_CSR_REG_BASE);
		/* fifo empty */
		if (fifo_csr & (1 << EAV_DTE_DTE_LTS_CSR_REG__Fifo_Empty)) {
			break;
		}
		/* overflow error */
		if (fifo_csr & (1 << EAV_DTE_DTE_LTS_CSR_REG__Fifo_Overflow))
			break;

		/* first event */
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		inlen = kfifo_in(&recv_fifo, &rd_data, sizeof(rd_data));
		if (inlen != sizeof(rd_data)) {
			dte_dev->kfifoof++;
		}

		/* then timestamp */
		rd_data = DTE_READL(audioeav, EAV_DTE_DTE_LTS_FIFO_REG_BASE);
		inlen = kfifo_in(&recv_fifo, &rd_data, sizeof(rd_data));
		if (inlen != sizeof(rd_data)) {
			dte_dev->kfifoof++;
		}
	} while (++i < 1000); // avoid dead loop

	if (fifo_csr & (1 << EAV_DTE_DTE_LTS_CSR_REG__Fifo_Underflow)) {
		dte_dev->fifouf++;
	}
	if (fifo_csr  & (1 << EAV_DTE_DTE_LTS_CSR_REG__Fifo_Overflow)) {
		dte_dev->fifoof++;
	}

	/* overflow/underflow error, reset fifo */
	if (fifo_csr & 0xc)
		DTE_WRITEL(0, audioeav, EAV_DTE_DTE_LTS_CSR_REG_BASE);
		
#ifdef DTE_GPIO_INT
	if (!i) {
		if (fifo_empty_cnt++ >= DTE_N_FIFOEMPTY) {
			disable_irq_nosync(dte_dev->dteirq);
			dte_asiu_iomux_select_external_inputs(0);
			fifo_empty_cnt = 0;
			dte_dev->mode = 1;
		}
	} else {
		fifo_empty_cnt = 0;
	}
#endif

	/* clear interrupt bit */
	DTE_WRITEL(1<< EAV_DTE_DTE_CTRL_REG__Interrupt, audioeav,
		EAV_DTE_DTE_CTRL_REG_BASE);

	return IRQ_HANDLED;
}

#ifdef DTE_GPIO_INT
static irqreturn_t gpio_isr(int irq, void *drv_ctx)
{
	struct bcm5830x_dte_dev *dte_dev = drv_ctx;

	if ((irq == dte_dev->gpio14irq) || (irq == dte_dev->gpio15irq) 
		|| (irq == dte_dev->gpio22irq)) {
		dte_asiu_iomux_select_external_inputs(1);
		dte_asiu_audio_dte_empty_fifo();
		DTE_WRITEL(1<< EAV_DTE_DTE_CTRL_REG__Interrupt, audioeav,
			EAV_DTE_DTE_CTRL_REG_BASE);
		if (dte_dev->mode != 2)
			enable_irq(dte_dev->dteirq);
		dte_dev->mode = 2;

		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}
#endif

static int __devinit bcm5830x_dte_probe(struct platform_device *pdev)
{
	struct bcm5830x_dte_dev *dte_dev;
	int rc = 0;
	dev_t devno;
	int id = 0;

	/* init dte_dev */
	dte_dev = kzalloc(sizeof(*dte_dev), GFP_KERNEL);
	if (!dte_dev) {
		goto err_release_region;
	}

	dte_dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, dte_dev);

	/* request resource */
	rc = bcm5830x_dte_resource_init(pdev);
	if (rc) {
		goto err_request_resource;
	}

	dte_dev->dteirq = iproc_platform_get_irq_byname(pdev, "dteirq");
	if (dte_dev->dteirq < 0) {
		rc = -ENODEV;
		goto err_request_resource;
	}

	/* init sysfs */
	for (id = 0; id < ARRAY_SIZE(sysfs_list); ++id) {
		rc = iproc_device_create_file(dte_dev->dev, &sysfs_list[id]);
		if (rc) {
			dev_err(dte_dev->dev, "Failed adding sysfs file\n");
			goto err_remove_sysfs_files;
		}
	}

	pr_info("--> %s dte_dev %x irqd %d \n", __FUNCTION__,
		(uint32_t)dte_dev, dte_dev->dteirq);


	/* create device */
	devno = MKDEV(DTE_MAJOR, 0);
	cdev_init(&dte_dev->dte_cdev, &dte_cdev_fops);
	rc = cdev_add(&dte_dev->dte_cdev, devno, 1);
	if (rc) {
		dev_err(dte_dev->dev, "Failed adding dte cdev file\n");
		goto err_remove_sysfs_files;
	}

	/* create class for mdev auto create node /dev/dte */
	dte_dev->dte_class = iproc_class_create(THIS_MODULE, "dte");
	if (IS_ERR(dte_dev->dte_class))
	{
		dev_err(dte_dev->dev, "Failed creating class\n");
		rc = PTR_ERR(dte_dev->dte_class);
		goto err_remove_static_cdev;
	}
	iproc_device_create(dte_dev->dte_class, NULL, devno, NULL, "dte");

	/* alloc fifo*/
	rc = kfifo_alloc(&recv_fifo, DTE_FIFO_SIZE, GFP_KERNEL);
	if (rc) {
		dev_err(dte_dev->dev, "Failed kfifo alloc \n");
		goto err_remove_class;
	}

	/* hw init */
	dte_init();

	/* init irq */
	rc = request_irq(dte_dev->dteirq, dte_isr, IRQF_DISABLED,
		pdev->name, dte_dev);
	if (rc) {
		dev_err(dte_dev->dev, "Failed request dteirq\n");
		goto err_free_kfifo;
	}

#ifdef DTE_GPIO_INT
	disable_irq_nosync(dte_dev->dteirq);

	dte_dev->gpio14irq = iproc_gpio_to_irq(14);
	rc = request_irq(dte_dev->gpio14irq, gpio_isr, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED,
		pdev->name, dte_dev);
	if (rc) {
		dev_err(dte_dev->dev, "Failed request gpio14 irq\n");
		goto err_free_irq;
	}
	dte_dev->gpio15irq = iproc_gpio_to_irq(15);
	rc = request_irq(dte_dev->gpio15irq, gpio_isr, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED,
		pdev->name, dte_dev);
	if (rc) {
		dev_err(dte_dev->dev, "Failed request gpio15 irq\n");
		goto err_free_irq;
	}
	dte_dev->gpio22irq = iproc_gpio_to_irq(22);
	rc = request_irq(dte_dev->gpio22irq, gpio_isr, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED,
		pdev->name, dte_dev);
	if (rc) {
		dev_err(dte_dev->dev, "Failed request gpio22 irq\n");
		goto err_free_irq;
	}
#endif

	return rc;

#ifdef DTE_GPIO_INT
err_free_irq:
	if (dte_dev->gpio14irq)
		free_irq(dte_dev->gpio14irq, dte_dev);
	if (dte_dev->gpio15irq)
		free_irq(dte_dev->gpio15irq, dte_dev);
	if (dte_dev->gpio22irq)
		free_irq(dte_dev->gpio22irq, dte_dev);
#endif
	
err_free_kfifo:
	kfifo_free(&recv_fifo);

err_remove_class:
	iproc_device_destroy(dte_dev->dte_class, MKDEV(DTE_MAJOR, 0));
	iproc_class_destroy(dte_dev->dte_class);

err_remove_static_cdev:
	cdev_del (&dte_dev->dte_cdev);

err_remove_sysfs_files:
	while (id-- > 0) {
		iproc_device_remove_file(&pdev->dev, &sysfs_list[id]);
	}

err_request_resource:
	kfree(dte_dev);

err_release_region:

	return rc;
}

static int __devexit bcm5830x_dte_remove(struct platform_device *pdev)
{
	int id = ARRAY_SIZE(sysfs_list);
	struct bcm5830x_dte_dev *dte_dev = platform_get_drvdata(pdev);

	pr_info("--> %s dte_dev %x\n", __FUNCTION__, (uint32_t)dte_dev);

	kfifo_free(&recv_fifo);

	bcm5830x_dte_resource_release(pdev);

	free_irq(dte_dev->dteirq, dte_dev);
#ifdef DTE_GPIO_INT
	free_irq(dte_dev->gpio14irq, dte_dev);
	free_irq(dte_dev->gpio15irq, dte_dev);
	free_irq(dte_dev->gpio22irq, dte_dev);
#endif

	iproc_device_destroy(dte_dev->dte_class, MKDEV(DTE_MAJOR, 0));
	iproc_class_destroy(dte_dev->dte_class);

	while (id-- > 0) {
		iproc_device_remove_file(&pdev->dev, &sysfs_list[id]);
	}

	cdev_del(&dte_dev->dte_cdev);

	kfree(dte_dev);

	return 0;
}

#ifdef CONFIG_PM
static int bcm5830x_dte_suspend(struct device *dev)
{
	pr_info("--> %s \n", __FUNCTION__);
	return 0;
}
static int bcm5830x_dte_resume(struct device *dev)
{
	pr_info("--> %s \n", __FUNCTION__);
	dte_init();
	return 0;
}
static SIMPLE_DEV_PM_OPS(bcm5830x_dte_pm, bcm5830x_dte_suspend,
	bcm5830x_dte_resume);
#endif

static struct platform_driver bcm5830x_dte_driver = {
	.driver = {
		.name = "bcm5830x-dte",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &bcm5830x_dte_pm,
#endif
	},
	.probe	= bcm5830x_dte_probe,
	.remove	= __devexit_p(bcm5830x_dte_remove),
};

static void platform_dte_release(struct device * dev)
{
	return ;
}

static struct resource dtedev_resources[] = {
	[0] = {
		.name   = "dteirq",
		.flags  = IORESOURCE_IRQ,
		.start  = CHIP_INTR1__ASIU_AUDIO_DTE_INTR + 161,
		.end    = CHIP_INTR1__ASIU_AUDIO_DTE_INTR + 161,
	},
	[1] = {
		.name   = "crmudru",
		.flags  = IORESOURCE_MEM,
		.start  = CRMU_XTAL_CHANNEL_CONTROL,
		.end	= CRMU_XTAL_CHANNEL_CONTROL + SZ_4K - 1,
	},
	[2] = {
		.name   = "chipdru",
		.flags  = IORESOURCE_MEM,
		.start  = CRMU_GENPLL_CONTROL0,
		.end	= CRMU_GENPLL_CONTROL0 + 0xbff,
	},
	[3] = {
		.name	= "gpio",
		.flags	= IORESOURCE_MEM,
		.start	= ChipcommonG_GP_DATA_IN,
		.end	= ChipcommonG_GP_DATA_IN + SZ_4K - 1,
	},
	[4] = {
		.name	= "audio",
		.flags	= IORESOURCE_MEM,
		.start	= AUD_MISC_REVISION_REG,
		.end	= AUD_MISC_REVISION_REG + SZ_4K - 1,
	},
	[5] = {
		.name	= "asiutop",
		.flags	= IORESOURCE_MEM,
		.start	= ASIU_INTR_STATUS,
		.end	= ASIU_INTR_STATUS + SZ_4K - 1,
	},
	[6] = {
		.name	= "audioeav",
		.flags	= IORESOURCE_MEM,
		.start	= 0x180af000,
		.end	= 0x180af000 + SZ_4K - 1,
	},
};

static struct platform_device dte_dev = {
	.name = "bcm5830x-dte",
	.id	= -1,
	.dev = {
		.release = platform_dte_release,
	},
	.resource = dtedev_resources,
	.num_resources	= ARRAY_SIZE(dtedev_resources),
};

static int __init bcm5830x_dte_init(void)
{
	pr_info("--> %s \n", __FUNCTION__);
	iproc_platform_device_register(&dte_dev);
	iproc_platform_driver_register(&bcm5830x_dte_driver);
	return 0;
}
module_init(bcm5830x_dte_init);

static void __exit bcm5830x_dte_exit(void)
{
	pr_info("--> %s \n", __FUNCTION__);
	iproc_platform_device_unregister(&dte_dev);
	iproc_platform_driver_unregister(&bcm5830x_dte_driver);
}
module_exit(bcm5830x_dte_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("DTE hardware driver for BCM5830X");
MODULE_LICENSE("Proprietary");
MODULE_ALIAS("platform:bcm5830x-dte");
