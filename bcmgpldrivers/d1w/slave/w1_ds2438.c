#include <asm/types.h>

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include "../../../kernel/linux-3.6.5/drivers/w1/w1.h"
#include "../../../kernel/linux-3.6.5/drivers/w1/w1_int.h"
#include "../../../kernel/linux-3.6.5/drivers/w1/w1_family.h"
#include "w1_ds2438.h"

typedef struct ds2438_devinfo {
	/* DS2438 data, valid after calling ds2438_battery_read_status() */
	unsigned long update_time;	/* jiffies when data read */
	char raw[DS2438_PAGE_SIZE];	/* raw DS2438 data */
	int voltage_uV;
	int current_uA;
	int accum_current_uAh;
	int temp_C;
	u8 init:1;
	u8 setup:1;
	u8 calibrate:1;
	u8 input_src:1;
	u8 ee_flg:1;
	u8 resv_bit:3;
	u8 threshold:8;
	u16 resv_bytes;
	u32 senser;

	struct device *w1_dev;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
}ds2438_dev_info;

#define DS2438_SENSER	25
#define to_ds2438_device_info(x) container_of((x), ds2438_dev_info, bat)

static unsigned int update_interval = 1000;
module_param(update_interval, uint, 0644);
MODULE_PARM_DESC(update_interval, "cache time in milliseconds");

static int ds2438_read_page(struct device *dev, u8 page, u8 *buf)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);
	if ((page >= DS2438_PAGE_NUM) || (buf == NULL))
		return -EINVAL;

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, W1_READ_SCRATCHPAD);
		w1_write_8(slave->master, page);
		w1_read_block(slave->master, buf, DS2438_PAGE_SIZE);
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}

static int ds2438_write_page(struct device *dev, u8 page, u8 *buf)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);
	if ((page >= DS2438_PAGE_NUM) || (buf == NULL))
		return -EINVAL;

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, DS2438_WRITE_SCRATCHPAD);
		w1_write_8(slave->master, page);
		w1_write_block(slave->master, buf, DS2438_PAGE_SIZE);
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}

static int ds2438_command(struct device *dev, u8 command, u8 data)
{
	struct w1_slave *slave = container_of(dev, struct w1_slave, dev);

	mutex_lock(&slave->master->mutex);
	if (!w1_reset_select_slave(slave)) {
		w1_write_8(slave->master, command);
		switch (command) {
		case DS2438_COPY_SCRATCHPAD:
		case DS2438_RECALL_MEMORY:
			w1_write_8(slave->master, data);
		}
	}
	mutex_unlock(&slave->master->mutex);
	return 0;
}

static int ds2438_drain_sram(struct device *dev, u8 page)
{
	return ds2438_command(dev, DS2438_COPY_SCRATCHPAD, page);
}

static int ds2438_load_sram(struct device *dev, u8 page)
{
	return ds2438_command(dev, DS2438_RECALL_MEMORY, page);
}

static ssize_t ds2438_show_status(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
    struct w1_slave *slave = container_of(dev, struct w1_slave, dev);
    ds2438_dev_info *di = (ds2438_dev_info *)dev_get_drvdata(&slave->dev);
    int len = 0;

    len = sprintf(buf, "voltage_uV=%08x\n", di->voltage_uV);
    len += sprintf(buf+len, "current_uA=%08x\n", di->current_uA);
    len += sprintf(buf+len, "accum_current_uAh=%08x\n", di->accum_current_uAh);
    len += sprintf(buf+len, "temp_C=%08x\n", di->temp_C);
	return len;
}

static struct device_attribute ds2438_dev_attr[] = {
	__ATTR(status,      0400, ds2438_show_status,   NULL),
};

static void ds2438_init(ds2438_dev_info *di)
{
    ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
    ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	if (di->init && di->setup) {
		if (di->ee_flg)
			di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_EE;
		else
			di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_EE;
		if (di->input_src)
			di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_AD;
		else
			di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_AD;
		di->raw[PAGE0_THRESHOLD] = di->threshold;
	} else {
		di->ee_flg = !!(di->raw[PAGE0_STAT_CTRL] & DS2438_CTRL_EE);
		di->input_src = !!(di->raw[PAGE0_STAT_CTRL] & DS2438_CTRL_AD);
		di->threshold = di->raw[PAGE0_THRESHOLD];
		di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD | DS2438_CTRL_CA;
	}
    ds2438_write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
    ds2438_drain_sram(di->w1_dev, PAGE0_CONTROL);
	if (!di->init) {
		di->calibrate = 1;
		di->init = 1;
	}
	di->setup = 0;
}

static void ds2438_calibrate_init(ds2438_dev_info *di)
{
	int current_raw;
	/* disable ICA */
	ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
	ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_IAD;
	ds2438_write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	ds2438_drain_sram(di->w1_dev, PAGE0_CONTROL);

	/* Zero offset */
	ds2438_load_sram(di->w1_dev, PAGE1_ETM);
	ds2438_read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ds2438_writew(di->raw + PAGE1_OFFSET_LSB, 0);
	ds2438_drain_sram(di->w1_dev, PAGE1_ETM_BYTE0);

	/* enable ICA & read current */
	ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
	ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD;
	ds2438_write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	ds2438_drain_sram(di->w1_dev, PAGE0_CONTROL);
	/*wait current convert about 36HZ */
	mdelay(30);
	/* disable ICA */
	ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
	ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] &= ~DS2438_CTRL_IAD;
	ds2438_write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	ds2438_drain_sram(di->w1_dev, PAGE0_CONTROL);
	/* read current value */
	current_raw = ds2438_readw(di->raw + PAGE0_CURRENT_LSB);
	/* write offset by current value */
	ds2438_load_sram(di->w1_dev, PAGE1_ETM);
	ds2438_read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ds2438_writew(di->raw + PAGE1_OFFSET_LSB, current_raw << 8);
	ds2438_write_page(di->w1_dev, PAGE1_ETM, di->raw);
	ds2438_drain_sram(di->w1_dev, PAGE1_ETM);

	/*enable ICA again */
	ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
	ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	di->raw[PAGE0_STAT_CTRL] |= DS2438_CTRL_IAD;
	ds2438_write_page(di->w1_dev, PAGE0_CONTROL, di->raw);
	ds2438_drain_sram(di->w1_dev, PAGE0_CONTROL);
	di->calibrate = 0;
}

/*
 * power supply temperture is in tenths of degree.
 */
static inline int ds2438_get_temp(u16 raw)
{
	int degree, s;
    
	s = !!(raw & 0x8000);

	if (s)
		raw = ((~raw & 0x7FFF) + 1);
	degree = ((raw >> 8) * 10) + (((raw & 0xFF) * 5) + 63) / 128;
    
	return s ? -degree : degree;
}

/*
 * power supply current is in uA.
 */
static inline int ds2438_get_current(u32 senser, u16 raw)
{
	int s, current_uA;
	s = !!(raw & 0xFC00);
	/* (x * 1000 * 1000)uA / (4096 * (Rsens / 1000)) */
	raw &= 0x3FF;
	current_uA = raw * 125 * 125 * 125;
	current_uA /= (senser << 3);
	return s ? -current_uA : current_uA;
}

/*
 * power supply current is in uAh.
 */
static inline int ds2438_get_ica(u32 senser, u8 raw)
{
	int charge_uAh;
	/* (x * 1000 * 1000)uA / (2048 * (Rsens / 1000)) */
	charge_uAh = (raw * 125 * 125 * 125) >> 4;
	charge_uAh /= (senser << 4);
	return charge_uAh;
}

static int ds2438_battery_update_page1(ds2438_dev_info *di)
{
	int ica_raw;
	ds2438_load_sram(di->w1_dev, PAGE1_ETM);
	ds2438_read_page(di->w1_dev, PAGE1_ETM, di->raw);
	ica_raw = di->raw[PAGE1_ICA];
	di->accum_current_uAh = ds2438_get_ica(di->senser, ica_raw);
	return 0;
}

static int ds2438_battery_read_status(ds2438_dev_info *di)
{
	u8 status;
	int temp_raw, voltage_raw, current_raw;

	if (!(di->init) || di->setup)
		ds2438_init(di);

	if (di->calibrate)
		ds2438_calibrate_init(di);

	if (di->update_time && time_before(jiffies, di->update_time +
					   msecs_to_jiffies(update_interval)))
		return 0;

    ds2438_load_sram(di->w1_dev, PAGE0_CONTROL);
    ds2438_read_page(di->w1_dev, PAGE0_CONTROL, di->raw);
    
	status = di->raw[PAGE0_STAT_CTRL];
	temp_raw = ds2438_readw(di->raw + PAGE0_TEMP_LSB);
	voltage_raw = ds2438_readw(di->raw + PAGE0_VOLTAGE_LSB);
	current_raw = ds2438_readw(di->raw + PAGE0_CURRENT_LSB);
	di->temp_C = ds2438_get_temp(temp_raw);
	di->voltage_uV = voltage_raw * 10000;
	di->current_uA = ds2438_get_current(di->senser, current_raw);

	ds2438_battery_update_page1(di);

	if (!(status & DS2438_STAT_TB))
    {   
        ds2438_command(di->w1_dev, DS2438_CONVERT_TEMP, 0);
    }
	if (!(status & DS2438_STAT_ADB))
    {   
        ds2438_command(di->w1_dev, DS2438_CONVERT_VOLT, 0);
    }
	di->update_time = jiffies;
	return 0;
}

static void ds2438_battery_status_read(struct work_struct *work)
{
	ds2438_dev_info *di = container_of(work,
						     ds2438_dev_info,
						     monitor_work.work);
	const int interval = HZ * 3;

	dev_dbg(di->w1_dev, "%s\n", __func__);
	ds2438_battery_read_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static int ds2438_add_slave(struct w1_slave *slave)
{
	int i, retval = 0;
	ds2438_dev_info *di;

	di = (ds2438_dev_info *)kzalloc(sizeof(ds2438_dev_info), GFP_KERNEL);
	if (!di) {
		retval = -ENOMEM;
		goto di_alloc_failed;
	}

	di->w1_dev = &slave->dev;
	di->senser = DS2438_SENSER;
    di->setup = 1;
    di->ee_flg = 1;
    di->input_src = 1;
    di->threshold = 0x40;
    di->setup = 1;
    di->init = 0;
    di->calibrate = 1;

	for (i = 0; i < ARRAY_SIZE(ds2438_dev_attr); i++) {
		if (device_create_file(&slave->dev, ds2438_dev_attr + i)) {
			printk(KERN_ERR "Customize attribute file fail!\n");
			break;
		}
	}

	if (i != ARRAY_SIZE(ds2438_dev_attr)) {
		for (; i >= 0; i--)
			device_remove_file(&slave->dev, ds2438_dev_attr + i);
		goto DS2438_INIT_failed;
	}
	INIT_DELAYED_WORK(&di->monitor_work, ds2438_battery_status_read);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&slave->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto DS2438_INIT_failed;
	}
	dev_set_drvdata(&slave->dev, di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ / 2);

	goto success;

DS2438_INIT_failed:
    kfree(di);
di_alloc_failed:
success:
	return retval;
}

static void ds2438_remove_slave(struct w1_slave *slave)
{
	ds2438_dev_info *di = dev_get_drvdata(&slave->dev);

    cancel_delayed_work_sync(&di->monitor_work);
	destroy_workqueue(di->monitor_wqueue);
}

static struct w1_family_ops w1_ds2438_fops = {
	.add_slave = ds2438_add_slave,
	.remove_slave = ds2438_remove_slave,
};

static struct w1_family w1_family_ds2438 = {
	.fid = W1_FAMILY_DS2438,
	.fops = &w1_ds2438_fops,
};

static int __init w1_ds2438_init(void)
{
	printk("DS2438 - Cygnus battery monitor\n");
	return w1_register_family(&w1_family_ds2438);
}

static void __exit w1_ds2438_fini(void)
{
	w1_unregister_family(&w1_family_ds2438);
}

late_initcall(w1_ds2438_init);
module_exit(w1_ds2438_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Inc");
MODULE_DESCRIPTION("DS2438 driver for Cygnus battery monitor");

