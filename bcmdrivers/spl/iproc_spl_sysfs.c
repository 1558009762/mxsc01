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
#include <asm/io.h>
#include <linux/platform_device.h>

#include "iproc_spl.h"
#include "socregs.h"
#include "../../kernel/linux-3.6.5/arch/arm/plat-iproc/include/plat/shm.h"

/////////////////////////////////////////////////////////////////////////
/* return 0 or 1 */
static inline unsigned char spl_test_bit(uint32_t val, unsigned char bitnum)
{
	if(bitnum>31)
	{
		iproc_err("Invalid bit position, must be [0~31]!\n");	
		return 0;
	}
	
	return val & (1<<bitnum) ? 1: 0;
}

/////////////////////////////////////////////////////////////////////////
#define spl_sysfs_error_prt(reason, usage, arg_num) \
	do{ \
		if(reason==0) \
			iproc_err("Wrong format, format:'%s'\n", usage); \
		else if(reason==1) \
			iproc_err("Out of range, format:'%s'\n", usage); \
	}while(0)

static ssize_t wdog_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t val = spl_reg32_read(SPL_WDOG);
	
	return sprintf(buf, "0x%08x\n", val);
}

static ssize_t wdog_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	uint32_t val=0;
	int err = 0;
	char *usage = "val(0x0~0xFFFFFFFF)";

	if((err=sscanf(buf, "0x%8x", &val)) != 1)
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}

	val &= 0xFFFFFFFF;

	iproc_dbg("val=0x%08x\n", val);
	
	spl_reg32_write(SPL_WDOG, val);
	
	return n;
}

static ssize_t ring_oscillator_calibrators_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t cal0 = spl_reg32_read(SPL_CALIB_0);
	uint32_t cal1 = spl_reg32_read(SPL_CALIB_1);
	uint32_t cal2 = spl_reg32_read(SPL_CALIB_2);
	
	return sprintf(buf, "[cal0 cal1 cal2]=0x%08x 0x%08x 0x%08x\n", cal0, cal1, cal2);
}


static ssize_t ring_oscillator_calibrators_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	int err=0;
	uint32_t cal0, cal1, cal2;
	char *usage = "cal0(0x0~0x3FFFFFFF) cal1(0x0~0x3FFFFFFF) cal2(0x0~0x7FFF)";
	
	if ((err=sscanf(buf, "0x%8x 0x%8x 0x%8x", &cal0, &cal1, &cal2)) != 3)
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}

	if(cal0>0x3FFFFFFF || cal1>0x3FFFFFFF || cal2>0x7FFF)
	{
		spl_sysfs_error_prt(1, usage, err);
		return -EINVAL;
	}
	
	iproc_dbg("cal0=0x%08x, cal1=0x%08x, cal2=0x%08x\n", cal0, cal1, cal2);

	spl_reg32_write(SPL_CALIB_0, cal0);
	spl_reg32_write(SPL_CALIB_1, cal1);
	spl_reg32_write(SPL_CALIB_2, cal2);
	
	return n;
}

static ssize_t freq_monitors_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cnt;
	
	uint32_t f0_en = spl_reg32_read(SPL_FREQ_MON_EN_0);
	uint32_t f0l_en = (f0_en & (1<<0))  ? 1:0;
	uint32_t f0h_en = (f0_en & (1<<16)) ? 1:0;
	uint32_t f0_mp = spl_reg32_read(SPL_MONITOR_POINT_0) & 0x0000ffff;
	uint32_t f0_lt = spl_reg32_read(SPL_LOW_THRESH_0)    & 0x0000ffff;
	uint32_t f0_ht = spl_reg32_read(SPL_HIGH_THRESH_0)   & 0x0000ffff;

	uint32_t f1_en = spl_reg32_read(SPL_FREQ_MON_EN_1);
	uint32_t f1l_en = (f1_en & (1<<0))  ? 1:0;
	uint32_t f1h_en = (f1_en & (1<<16)) ? 1:0;
	uint32_t f1_mp = spl_reg32_read(SPL_MONITOR_POINT_1) & 0x0000ffff;
	uint32_t f1_lt = spl_reg32_read(SPL_LOW_THRESH_1)    & 0x0000ffff;
	uint32_t f1_ht = spl_reg32_read(SPL_HIGH_THRESH_1)   & 0x0000ffff;

	uint32_t f2_en = spl_reg32_read(SPL_FREQ_MON_EN_2);
	uint32_t f2l_en = (f2_en & (1<<0))  ? 1:0;
	uint32_t f2h_en = (f2_en & (1<<16)) ? 1:0;
	uint32_t f2_mp = spl_reg32_read(SPL_MONITOR_POINT_2) & 0x0000ffff;
	uint32_t f2_lt = spl_reg32_read(SPL_LOW_THRESH_2)    & 0x0000ffff;
	uint32_t f2_ht = spl_reg32_read(SPL_HIGH_THRESH_2)   & 0x0000ffff;

	cnt = sprintf(buf, 
		"ID LowEn HighEn MonitorPoint LowThreshold HighThreshold\n"
		"0  %u     %u      0x%08x   0x%08x   0x%08x\n"
		"1  %u     %u      0x%08x   0x%08x   0x%08x\n"
		"2  %u     %u      0x%08x   0x%08x   0x%08x\n", 
		f0l_en, f0h_en, f0_mp, f0_lt, f0_ht,
		f1l_en, f1h_en, f1_mp, f1_lt, f1_ht,
		f2l_en, f2h_en, f2_mp, f2_lt, f2_ht);

	return cnt;
}

static ssize_t freq_monitors_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	int err=0;
	uint32_t en;
	uint32_t id, l_en, h_en, mp, lt, ht;
	uint32_t offset;
	char *usage = "ID(0~2) LowEn(0|1) HighEn(0|1) MonitorPoint(0x0~0xFFFF) LowThreshold(0x0~0xFFFF) HighThreshold(0x0~0xFFFF)";
	
	if ((err=sscanf(buf, "%u %u %u 0x%8x 0x%8x 0x%8x", &id, &l_en, &h_en, &mp, &lt, &ht)) != 6)
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}

	if(id>2 || l_en>1 || h_en>1 || mp>0xFFFF || lt>0xFFFF || ht>0xFFFF)
	{
		spl_sysfs_error_prt(1, usage, err);
		return -EINVAL;
	}

	en = (h_en<<16) | l_en;

	offset=(SPL_FREQ_MON_EN_1 - SPL_FREQ_MON_EN_0)*id;

	iproc_dbg("id=%u, en=%u, mp=0x%08x, lt=0x%08x, ht=0x%08x\n", id, en, mp, lt, ht);

	spl_reg32_write(SPL_FREQ_MON_EN_0   + offset, en);
	spl_reg32_write(SPL_MONITOR_POINT_0 + offset, mp);
	spl_reg32_write(SPL_LOW_THRESH_0    + offset, lt);
	spl_reg32_write(SPL_HIGH_THRESH_0   + offset, ht);

	return n;
}

static ssize_t reset_monitors_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cnt;
	
	uint32_t r0_en = spl_reg32_read(SPL_RST_MON_EN_0)     & 1 ? 1:0;
	uint32_t r0_ct = spl_reg32_read(SPL_RST_CNT_THRESH_0) & 0xff;
	uint32_t r0_wt = spl_reg32_read(SPL_WIN_CNT_THRESH_0);
	uint32_t r0_cv = spl_reg32_read(SPL_RST_CNT_VAL_0)    & 0xff;
	uint32_t r0_wv = spl_reg32_read(SPL_WIN_CNT_VAL_0);

	uint32_t r1_en = spl_reg32_read(SPL_RST_MON_EN_1)     & 1 ? 1:0;
	uint32_t r1_ct = spl_reg32_read(SPL_RST_CNT_THRESH_1) & 0xff;
	uint32_t r1_wt = spl_reg32_read(SPL_WIN_CNT_THRESH_1);
	uint32_t r1_cv = spl_reg32_read(SPL_RST_CNT_VAL_1)    & 0xff;
	uint32_t r1_wv = spl_reg32_read(SPL_WIN_CNT_VAL_1);

	uint32_t r2_en = spl_reg32_read(SPL_RST_MON_EN_2)     & 1 ? 1:0;
	uint32_t r2_ct = spl_reg32_read(SPL_RST_CNT_THRESH_2) & 0xff;
	uint32_t r2_wt = spl_reg32_read(SPL_WIN_CNT_THRESH_2);
	uint32_t r2_cv = spl_reg32_read(SPL_RST_CNT_VAL_2)    & 0xff;
	uint32_t r2_wv = spl_reg32_read(SPL_WIN_CNT_VAL_2);

	uint32_t r3_en = spl_reg32_read(SPL_RST_MON_EN_3)     & 1 ? 1:0;
	uint32_t r3_ct = spl_reg32_read(SPL_RST_CNT_THRESH_3) & 0xff;
	uint32_t r3_wt = spl_reg32_read(SPL_WIN_CNT_THRESH_3);
	uint32_t r3_cv = spl_reg32_read(SPL_RST_CNT_VAL_3)    & 0xff;
	uint32_t r3_wv = spl_reg32_read(SPL_WIN_CNT_VAL_3);


	cnt = sprintf(buf, 
		"ID En RstCntThresh RstWinCntThresh CurRstCntValue CurRstWinCntValue\n"
		"0  %u  0x%08x   0x%08x      0x%08x     0x%08x        (window unit:6.6666us)\n"
		"1  %u  0x%08x   0x%08x      0x%08x     0x%08x        (window unit:53.363us)\n"
		"2  %u  0x%08x   0x%08x      0x%08x     0x%08x        (window unit:213.63us)\n"
		"3  %u  0x%08x   0x%08x      0x%08x     0x%08x        (window unit:3413.3us)\n",
		r0_en, r0_ct, r0_wt, r0_cv, r0_wv,
		r1_en, r1_ct, r1_wt, r1_cv, r1_wv,
		r2_en, r2_ct, r2_wt, r2_cv, r2_wv,
		r3_en, r3_ct, r3_wt, r3_cv, r3_wv);

	return cnt;
}

static ssize_t reset_monitors_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	int err=0;
	uint32_t id, en, ct, wt;
	uint32_t offset;
	char *usage = "ID(0~3) En(0|1) RstCntThresh(0x0~0xFF) RstWindowCntThresh(0x0~0xFFFFFFFF)";
	
	if ((err=sscanf(buf, "%u %u 0x%8x 0x%8x", &id, &en, &ct, &wt)) != 4)
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}

	wt &= 0xffffffff;
	
	if(id>3 || en>1 || ct>0xFF)
	{
		spl_sysfs_error_prt(1, usage, err);
		return -EINVAL;
	}

	offset=(SPL_RST_MON_EN_1 - SPL_RST_MON_EN_0)*id;

	iproc_dbg("id=%u, en=%u, ct=0x%08x, wt=0x%08x\n", id, en, ct, wt);

	spl_reg32_write(SPL_RST_MON_EN_0     + offset, en);
	spl_reg32_write(SPL_RST_CNT_THRESH_0 + offset, ct);
	spl_reg32_write(SPL_WIN_CNT_THRESH_0 + offset, wt);

	return n;
}

static ssize_t osc_en_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t val = spl_reg32_read(OSC_EN) & 1;
	
	return sprintf(buf, "%u\n", val);
}

static ssize_t osc_en_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	uint32_t en=0;
	char *usage = "En(0|1)";

	if(0==strcmp(buf, "0\n"))
		en = 0;
	else if(0==strcmp(buf, "1\n"))
		en = 1;
	else
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}
	

	if(en!=1 && en!=0)
	{
		spl_sysfs_error_prt(1, usage, err);
		return -EINVAL;
	}

	iproc_dbg("en=%u\n", en);

	spl_reg32_write(OSC_EN, en);
	
	return n;
}

static ssize_t event_log_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t event;
	int cnt;
	
	event = spl_reg32_read(CRMU_SPL_EVENT_LOG);
	
	cnt = sprintf(buf, 
		"HighFreqEvent(0 1 2): %u %u %u\n"
		"LowFreqEvent (0 1 2): %u %u %u\n"
		"ResetEvent(0 1 2 3) : %u %u %u %u\n"
		"Write 1 to this file to clear LowFreqEvent.\n",
		spl_test_bit(event, 16), spl_test_bit(event, 17), spl_test_bit(event, 18), 
		spl_test_bit(event, 8),  spl_test_bit(event, 9),  spl_test_bit(event, 10), 
		spl_test_bit(event, 0),  spl_test_bit(event, 1),  spl_test_bit(event, 2), spl_test_bit(event, 3));
		
	return cnt;
}

static ssize_t event_log_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	uint32_t lowClear=0;
	int err = 0;
	char *usage = "LowEventClear(1)";
	
	if ((err=sscanf(buf, "%u", &lowClear)) != 1)
	{
		spl_sysfs_error_prt(0, usage, err);
		return -EINVAL;
	}

	if(lowClear!=1)
	{
		spl_sysfs_error_prt(1, usage, err);
		return -EINVAL;
	}

	iproc_dbg("LowEventClear=%u\n", lowClear);

	spl_reg32_write(CRMU_SPL_EVENT_LOG, spl_reg32_read(CRMU_SPL_EVENT_LOG) | (1<<31));
	
	return n;
}

/////////////////////////////////////////////////////////////////////////
#define IPROC_DEV_ATTR_RW(_name) 	DEVICE_ATTR(_name, 0644, _name##_show, _name##_store);
#define IPROC_DEV_ATTR_RO(_name) 	DEVICE_ATTR(_name, 0444, _name##_show, NULL);

static IPROC_DEV_ATTR_RW(wdog);
static IPROC_DEV_ATTR_RW(ring_oscillator_calibrators);
static IPROC_DEV_ATTR_RW(freq_monitors);
static IPROC_DEV_ATTR_RW(reset_monitors);
static IPROC_DEV_ATTR_RW(osc_en);
static IPROC_DEV_ATTR_RW(event_log);


static struct attribute *spl_dev_attrs[] = {
	&dev_attr_wdog.attr,
	&dev_attr_ring_oscillator_calibrators.attr,
	&dev_attr_freq_monitors.attr,
	&dev_attr_reset_monitors.attr,
	&dev_attr_osc_en.attr,
	&dev_attr_event_log.attr,
	NULL
};

static struct attribute_group spl_attr_group = {
	.name = DRIVER_NAME,
	.attrs = spl_dev_attrs,
};

int iproc_spl_sysfs_register(struct platform_device *pldev)
{
	return iproc_sysfs_create_group(&pldev->dev.kobj, &spl_attr_group);
}

void iproc_spl_sysfs_unregister(struct platform_device *pldev)
{
	iproc_sysfs_remove_group(&pldev->dev.kobj, &spl_attr_group);
}
