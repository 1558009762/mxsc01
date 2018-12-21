/*
 * A driver for SPI MSR.
 *
 * $Copyright (C) Broadcom Corporation $
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <plat/shm.h>
#include <asm/io.h>
#include <linux/amba/pl022.h>

#define MSR_MAJOR (241)
#define MSR_MINOR (0)
#define TRACK1_START_SEN_CHAR   0x25
#define TRACK2_START_SEN_CHAR   0x3B
#define TRACK3_START_SEN_CHAR   0x2B
#define TRACK_END_SEN_CHAR      0x3F
#define MSR_LOCK \
    do {spin_lock_irq(&msr_dev->msr_lock);} while (0)
#define MSR_UNLOCK \
    do {spin_unlock_irq(&msr_dev->msr_lock);} while (0)
#define msr_print(fmt, ...) do { \
    if (debug_on) \
        printk(KERN_INFO fmt, ##__VA_ARGS__); \
} while(0)

#define SSP_CR0(r)  ((r) + 0x000)
#define SSP_CR1(r)  ((r) + 0x004)
#define SSP_DR(r)   ((r) + 0x008)
#define SSP_SR(r)   ((r) + 0x00C)
#define SSP_CPSR(r) ((r) + 0x010)

#define CRMU_CONTROL(r) ((r) + 0x000)
#define CRMU_CTRL1(r)   ((r) + 0x010)
#define CRMU_CTRL4(r)   ((r) + 0x01C)

struct spi_msr_reg {
    void __iomem  *spireg;
    void __iomem  *crmureg;
    void __iomem  *gpio;
};

/* Structure for SPI MSR device  */
struct spi_msr_dev {
    struct spi_device *spi;
    struct device *dev;
    struct spi_msr_reg  reg;
    spinlock_t  msr_lock;
    int gpio15irq;
    wait_queue_head_t msr_queue;
    unsigned int flag;
    int user;
    unsigned char *buffer;
    unsigned buffer_size;
};

static struct spi_msr_dev *msr_dev;
static struct class *msr_class;
static unsigned bufsiz = 4096;
static unsigned delay = 1;

/* use "insmod iproc_msr.ko debug=1" to enable debug */
static uint32_t debug_on = 0;
module_param_named(debug, debug_on, uint, 0644);
MODULE_PARM_DESC(debug, "debug info enable or not (default: 0)");

static void _reg_map(void)
{
    msr_dev->reg.spireg = ioremap(ChipcommonG_SPI1_SSPCR0, 32);
    msr_dev->reg.crmureg = ioremap(CRMU_CHIP_IO_PAD_CONTROL, 32);
    msr_dev->reg.gpio   = ioremap(ChipcommonG_GP_DATA_IN, 4);
    return;
}

static void _reg_unmap(void)
{
    iounmap(msr_dev->reg.spireg);
    iounmap(msr_dev->reg.crmureg);
    iounmap(msr_dev->reg.gpio);
    return;
}

static void _reg_init(void)
{
    unsigned int value = 0;

    iowrite32(0, CRMU_CONTROL(msr_dev->reg.crmureg));

    value = ioread32(CRMU_CTRL1(msr_dev->reg.crmureg));
    value &= ~(0xF << CRMU_IOMUX_CTRL1__CORE_TO_IOMUX_GPIO15_SEL_R);
    iowrite32(value, CRMU_CTRL1(msr_dev->reg.crmureg));

    return;
}

#ifdef CONFIG_PM
static uint32_t msr_reg_1 = 0;
static uint32_t msr_reg_2 = 0;

static void _reg_save(void)
{
    msr_reg_1 = ioread32(CRMU_CONTROL(msr_dev->reg.crmureg));
    msr_reg_2 = ioread32(CRMU_CTRL1(msr_dev->reg.crmureg));
    return;
}

static void _reg_load(void)
{
    iowrite32(msr_reg_1, CRMU_CONTROL(msr_dev->reg.crmureg));
    iowrite32(msr_reg_2, CRMU_CTRL1(msr_dev->reg.crmureg));
    return;
}
#endif

static void _reg_exit(void)
{

    return;
}

static irqreturn_t msr_isr(int irq, void *drv_ctx)
{
    struct spi_msr_dev *msr_dev = drv_ctx;
    wake_up_interruptible(&msr_dev->msr_queue);
    msr_dev->flag = 1;
    return IRQ_HANDLED;
}

int spi_msr_open(struct inode *inode, struct file *filp)
{
    int ret = 0;

    MSR_LOCK;
    if (0 == msr_dev->user) {
        msr_dev->user++;
    }
    else {
        ret = -EBUSY;
    }
    MSR_UNLOCK;

    msr_dev->buffer_size = bufsiz;
    if ((0 == ret) && (NULL == msr_dev->buffer)) {
        msr_dev->buffer = kmalloc(msr_dev->buffer_size, GFP_KERNEL);
        if (!msr_dev->buffer) {
            msr_print("open/ENOMEM\n");
            return -ENOMEM;
        }
    }
    _reg_init();
    msr_dev->flag = 0;

    return ret;
}

int spi_msr_release(struct inode *inode, struct file *filp)
{
    int ret = 0;

    MSR_LOCK;
    msr_dev->user = 0;
    MSR_UNLOCK;

    kfree(msr_dev->buffer);
    msr_dev->buffer = NULL;
    _reg_exit();

    return ret;
}

static ssize_t _spi_read(struct spi_device *spi, void *buf, size_t len)
{
    int ret = 0;
    struct spi_transfer t = {
            .rx_buf     = buf,
            .len        = len,
        };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(spi, &m);
    if (0 == m.status) {
        ret = m.actual_length;
    }
    return ret;
}

static int _get_data(char *p, int count)
{
    unsigned char fdata[2] = {0};
    int ret = 0;

    // Skip 2 bytes, 0x0D, 0x0A
    _spi_read(msr_dev->spi, &fdata, 2);
    ret = _spi_read(msr_dev->spi, p, count);

    return ret;
}

ssize_t spi_msr_read(struct file *filp, char __user *buf, size_t count, loff_t *offp)
{
    int ret = 0, rc = 0;

    if (count > msr_dev->buffer_size) {
        count = msr_dev->buffer_size;
    }

    rc = wait_event_interruptible_timeout(msr_dev->msr_queue, (msr_dev->flag != 0), msecs_to_jiffies(delay*1000));
    if (1 == msr_dev->flag) {
        ret = _get_data(msr_dev->buffer, count);
        msr_dev->flag = 0;
        if (copy_to_user(buf, msr_dev->buffer, ret) != 0) {
            msr_print("copy_to_user failed\n");
            return -EFAULT;
        }
    }

    return ret;
}

/* this interface is used for select */
static unsigned int spi_msr_poll(struct file *file, poll_table *wait)
{
    msr_print("%s:  flag %d \n", __FUNCTION__,  msr_dev->flag);
    poll_wait(file, &msr_dev->msr_queue, wait);
    msr_print("%s:  poll end \n", __FUNCTION__);

    if (0 != msr_dev->flag) {
        return POLLIN | POLLRDNORM;
    }

    return 0;
}

static struct file_operations spi_msr_cdev_fops = {
    .owner = THIS_MODULE,
    .open = spi_msr_open,
    .release = spi_msr_release,
    .read = spi_msr_read,
    .poll = spi_msr_poll,
};

static int dump_status(char *buf, int len)
{
    unsigned int value = 0;

    value = ioread32(CRMU_CONTROL(msr_dev->reg.crmureg));
    len += sprintf(buf+len, "NONAME is 0x%X\n", value);
    value = ioread32(CRMU_CTRL1(msr_dev->reg.crmureg));
    len += sprintf(buf+len, "CRMU_IOMUX_CTRL1 is 0x%X\n", value);
    value = ioread32(CRMU_CTRL4(msr_dev->reg.crmureg));
    len += sprintf(buf+len, "CRMU_IOMUX_CTRL4 is 0x%X\n", value);
    return len;
}

static int dump_sp(char *buf, int len)
{
    unsigned int value = 0;

    value = ioread32(SSP_CR0(msr_dev->reg.spireg));
    len += sprintf(buf+len, "SPI1 CR0 is 0x%X\n", value);
    value = ioread32(SSP_CR1(msr_dev->reg.spireg));
    len += sprintf(buf+len, "SPI1 CR1 is 0x%X\n", value);
    value = ioread32(SSP_CPSR(msr_dev->reg.spireg));
    len += sprintf(buf+len, "SPI1 CPSR is 0x%X\n", value);
    value = ioread16(SSP_SR(msr_dev->reg.spireg));
    len += sprintf(buf+len, "SPI1 SR is 0x%X\n", value);
    return len;
}

/* SysFS callback when debug file is read */
static ssize_t debug_show(struct device *pDev,
        struct device_attribute *attr, char *buf)
{
    int len = 0;

    len += sprintf(buf+len, "#########################\n");
    len += dump_status(buf, len);
    len += sprintf(buf+len, "#########################\n");
    len += dump_sp(buf, len);
    len += sprintf(buf+len, "#########################\n");
    return len;
}

/* SysFS callback when debug file is written */
static ssize_t debug_store(struct device *pDev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    return 0;
}

/* SysFS callback when delay file is read */
static ssize_t delay_show(struct device *pDev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Read delay is %d second.\n", delay);
}

/* SysFS callback when delay file is written */
static ssize_t delay_store(struct device *pDev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    delay = simple_strtoul(buf, NULL, 10);
    return count;
}

/* SysFS callback when buffer file is read */
static ssize_t buffer_show(struct device *pDev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "Buffer size is %d.\n", bufsiz);
}

/* SysFS callback when buffer file is written */
static ssize_t buffer_store(struct device *pDev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    bufsiz = simple_strtoul(buf, NULL, 10);
    return count;
}

/* table of sysfs files we will create/destroy */
struct device_attribute sysfs_list[] = {
    __ATTR(debug, S_IWUSR | S_IRUGO, debug_show, debug_store),
    __ATTR(delay, S_IWUSR | S_IRUGO, delay_show, delay_store),
    __ATTR(buffer, S_IWUSR | S_IRUGO, buffer_show, buffer_store),
};

static int __devinit spi_msr_probe(struct spi_device *spi)
{
    int ret = 0;
    int devno, id;

    msr_print("spi_msr_probe!!!!!!!!!!!\n");

    msr_dev = kzalloc(sizeof(*msr_dev), GFP_KERNEL);
    if (!msr_dev) {
        msr_print("spi_msr: No memory\n");
        ret = -ENOMEM;
        goto ERROR_PROBE;
    }

    devno = MKDEV(MSR_MAJOR, MSR_MINOR);
    msr_dev->dev = device_create(msr_class, &spi->dev, devno, msr_dev, "spi_msr");
    ret = IS_ERR(msr_dev->dev) ? PTR_ERR(msr_dev->dev) : 0;
    if (0 != ret) {
        goto ERROR_PROBE_BUFFER;
    }

    msr_dev->spi = spi;
    spin_lock_init(&msr_dev->msr_lock);

    /* init sysfs */
    for (id = 0; id < ARRAY_SIZE(sysfs_list); ++id) {
        ret = iproc_device_create_file(msr_dev->dev, &sysfs_list[id]);
        if (ret) {
            goto ERROR_PROBE_SYSFS;
        }
    }

    init_waitqueue_head(&msr_dev->msr_queue);
    msr_dev->gpio15irq = gpio_to_irq(15);
    ret = request_irq(msr_dev->gpio15irq, msr_isr,
        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_DISABLED,
        NULL, msr_dev);
    if (ret) {
        printk("Can't add irq\n");
        goto ERROR_PROBE_IRQ;
    }

    _reg_map();
    return ret;

ERROR_PROBE_IRQ:
    if (msr_dev->gpio15irq)
        free_irq(msr_dev->gpio15irq, msr_dev);
ERROR_PROBE_SYSFS:
    while (id-- > 0) {
        iproc_device_remove_file(msr_dev->dev, &sysfs_list[id]);
    }
    device_destroy(msr_class, devno);
ERROR_PROBE_BUFFER:
    kfree(msr_dev);
    msr_dev = NULL;
ERROR_PROBE:
    return ret;
}

static int __devexit spi_msr_remove(struct spi_device *spi)
{
    int ret = 0;
    int devno = 0, id = 0;

    if (!msr_dev)
        return ret;

    _reg_unmap();

    if (msr_dev->gpio15irq)
        free_irq(msr_dev->gpio15irq, msr_dev);
    for (id = 0; id < ARRAY_SIZE(sysfs_list); ++id) {
        iproc_device_remove_file(msr_dev->dev, &sysfs_list[id]);
    }

    devno = MKDEV(MSR_MAJOR, MSR_MINOR);
    device_destroy(msr_class, devno);
    kfree(msr_dev);
    msr_dev = NULL;
    return ret;
}

#ifdef CONFIG_PM
static int spi_msr_suspend(struct device *dev)
{
    _reg_save();
    return 0;
}

static int spi_msr_resume(struct device *dev)
{
    msr_print("MSR resume \n");
    _reg_load();
    return 0;
}
static SIMPLE_DEV_PM_OPS(spi_msr_pm, spi_msr_suspend, spi_msr_resume);
#endif

static struct spi_driver msr_driver = {
    .driver = {
            .name   = "spi_msr",
            .owner  = THIS_MODULE,
#ifdef CONFIG_PM
            .pm    = &spi_msr_pm,
#endif
    },
    .probe      = spi_msr_probe,
    .remove     = __devexit_p(spi_msr_remove),
};

static int __init msr_init(void)
{
    int ret = 0;

    msr_print("MSR module loading\n");

    ret = register_chrdev(MSR_MAJOR, "spi_msr", &spi_msr_cdev_fops);
    if (ret) {
        msr_print("spi_msr: Failed adding cdev file\n");
        goto ERROR_INIT;
    }

    /* create class for mdev auto create node /dev/spi_msr */
    msr_class = iproc_class_create(THIS_MODULE, "spi_msr");
    if (IS_ERR(msr_class)) {
        msr_print("spi_msr: Failed create class\n");
        goto ERROR_INIT_CDEV;
    }

    ret = spi_register_driver(&msr_driver);
    if (ret) {
        msr_print("spi_msr: Failed to register driver\n");
        goto ERROR_INIT_CLASS;
    }
    return ret;

ERROR_INIT_CLASS:
    class_destroy(msr_class);
ERROR_INIT_CDEV:
    unregister_chrdev(MSR_MAJOR, msr_driver.driver.name);
ERROR_INIT:
    return ret;
}

static void msr_exit(void)
{
    msr_print("MSR module unloading\n");
    spi_unregister_driver(&msr_driver);
    class_destroy(msr_class);
    unregister_chrdev(MSR_MAJOR, msr_driver.driver.name);

    return;
}

module_init(msr_init);
module_exit(msr_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("MSR hardware driver for BCM5830X");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:msr");
