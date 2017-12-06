/*
 * $Copyright Open Broadcom Corporation$
 */
 
/*
 * DESCRIPTION: The Broadcom iProc RNG200 Driver
 */

#include <linux/hw_random.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

/* Registers for RNG */
#define	RNG_CTRL_OFFSET	0x00000000
#define	RNG_CTRL_RESERVED_MASK	0xF00000CC
#define	RNG_CTRL_COMBLK2_OSC_DIS_SHIFT	22
#define	RNG_CTRL_COMBLK2_OSC_DIS_MASK	0x0FC00000
#define	RNG_CTRL_COMBLK1_OSC_DIS_SHIFT	16
#define	RNG_CTRL_COMBLK1_OSC_DIS_MASK	0x003F0000
#define	RNG_CTRL_JCLK_BYP_DIV_CNT_SHIFT	8
#define	RNG_CTRL_JCLK_BYP_DIV_CNT_MASK	0x0000FF00
#define	RNG_CTRL_JCLK_BYP_SRC_SHIFT	5
#define	RNG_CTRL_JCLK_BYP_SRC_MASK	0x00000020
#define	RNG_CTRL_JCLK_BYP_SEL_SHIFT	4
#define	RNG_CTRL_JCLK_BYP_SEL_MASK	0x00000010
#define	RNG_CTRL_RBG2X_SHIFT	1
#define	RNG_CTRL_RBG2X_MASK	0x00000002
#define	RNG_CTRL_RBGEN_SHIFT	0
#define	RNG_CTRL_RBGEN_MASK	0x00000001

#define	RNG_STATUS_OFFSET	0x00000004
#define	RNG_STATUS_RESERVED_MASK	0x00F00000
#define	RNG_STATUS_RND_VAL_SHIFT	24
#define	RNG_STATUS_RND_VAL_MASK	0xFF000000
#define	RNG_STATUS_WARM_CNT_SHIFT	0
#define	RNG_STATUS_WARM_CNT_MASK	0x000FFFFF

#define	RNG_DATA_OFFSET	0x00000008
#define	RNG_DATA_RESERVED_MASK	0x00000000
#define	RNG_DATA_RNG_NUM_SHIFT	0
#define	RNG_DATA_RNG_NUM_MASK	0xFFFFFFFF

#define	RNG_FF_THRES_OFFSET	0x0000000C
#define	RNG_FF_THRES_RESERVED_MASK	0xFFFFFFE0
#define	RNG_FF_THRES_RNG_FF_THRESH_SHIFT	0
#define	RNG_FF_THRES_RNG_FF_THRESH_MASK	0x0000001F

#define	RNG_INT_MASK_OFFSET	0x00000010
#define	RNG_INT_MASK_RESERVED_MASK	0xFFFFFFFE
#define	RNG_INT_MASK_OFF_SHIFT	0
#define	RNG_INT_MASK_OFF_MASK	0x00000001

/* Registers for RNG200*/
#define RNG200_CTRL_OFFSET					0x00
#define RNG200_CTRL_RBGEN_MASK				0x00001FFF
#define RNG200_CTRL_RBGEN_ENABLE			0x00000001
#define RNG200_CTRL_RBGEN_DISABLE			0x00000000

#define RNG200_SOFT_RESET_OFFSET				0x04
#define RNG200_SOFT_RESET_MASK		0x00000001
#define RNG200_SOFT_RESET_ACTIVE		0x00000001
#define RNG200_SOFT_RESET_CLEAR		0x00000000

#define RBG_SOFT_RESET_OFFSET				0x08
#define RBG_RNG_SOFT_RESET_MASK		0x00000001
#define RBG_RNG_SOFT_RESET_ACTIVE		0x00000001
#define RBG_RNG_SOFT_RESET_CLEAR		0x00000000

#define RNG200_INT_STATUS_OFFSET				0x18
#define RNG200_INT_STATUS_MASTER_FAIL_LOCKOUT_IRQ_MASK	0x80000000
#define RNG200_INT_STATUS_STARTUP_TRANSITIONS_MET_IRQ_MASK	0x00020000
#define RNG200_INT_STATUS_NIST_FAIL_IRQ_MASK		0x00000020
#define RNG200_INT_STATUS_TOTAL_BITS_COUNT_IRQ_MASK	0x00000001

#define RNG200_FIFO_DATA_OFFSET				0x20

#define RNG200_FIFO_COUNT_OFFSET			0x24
#define RNG200_FIFO_COUNT_MASK		0x000000FF

static int rng_read(struct hwrng *rng, void *buf, size_t max,
			       bool wait)
{
	u32 num_words = 0;
	u32 num_remaining = max;

	#define MAX_IDLE_TIME	(1 * HZ)
	unsigned long idle_endtime = jiffies + MAX_IDLE_TIME;

	/* Retrieve HW RNG registers base address. */
	void __iomem *base_addr = (void __iomem *)rng->priv;

	while ((num_remaining > 0) && time_before(jiffies, idle_endtime)) {
		/* Are there any random numbers available? */
		num_words = (ioread32(base_addr + RNG_STATUS_OFFSET) &
			RNG_STATUS_RND_VAL_MASK) >> RNG_STATUS_RND_VAL_SHIFT;
		if (num_words > 0) {
			if (num_remaining >= sizeof(u32)) {
				/* Buffer has room to store entire word */
				*(u32 *)buf = ioread32(base_addr +
							RNG_DATA_OFFSET);
				buf += sizeof(u32);
				num_remaining -= sizeof(u32);
			} else {
				/* Buffer can only store partial word */
				u32 rnd_number = ioread32(base_addr +
							RNG_DATA_OFFSET);
				memcpy(buf, &rnd_number, num_remaining);
				buf += num_remaining;
				num_remaining = 0;
			}

			/* Reset the IDLE timeout */
			idle_endtime = jiffies + MAX_IDLE_TIME;
		} else if (!wait) {
			/* Cannot wait, return immediately */
			break;
		} else {
			/* Can wait, give others chance to run */
			cpu_relax();
		}
	}

	return max - num_remaining;
}

static struct hwrng rng_ops = {
	.name = "iproc-rng",
	.read = rng_read,
};

static int rng_probe(struct platform_device *pdev)
{
	int error;
	u32 val;
	struct device *dev = &pdev->dev;
	void __iomem *base_addr;
	struct device_node *node;

	pr_info("Broadcom IPROC RNG Driver\n");
	/* We only accept one device, and it must have an id of -1 */
	if (pdev->id != -1)
		return -ENODEV;

	node = pdev->dev.of_node;
	base_addr = of_iomap(node, 0);
	if (!base_addr) {
		dev_err(&pdev->dev, "can't iomap base_addr for rng\n");
		return -EIO;
	}
	rng_ops.priv = (unsigned long)base_addr;

	/* Start RNG block */
	val = ioread32(base_addr + RNG_CTRL_OFFSET);
	val |= RNG_CTRL_RBGEN_MASK;
	iowrite32(val, base_addr + RNG_CTRL_OFFSET);

	/* Enable RNG RBG2X */
	val = ioread32(base_addr + RNG_CTRL_OFFSET);
	val |= RNG_CTRL_RBG2X_MASK;
	iowrite32(val, base_addr + RNG_CTRL_OFFSET);

	/* Disable RNG INTERRUPT */
	val = ioread32(base_addr + RNG_INT_MASK_OFFSET);
	val |= RNG_INT_MASK_OFF_MASK;
	iowrite32(val, base_addr + RNG_INT_MASK_OFFSET);

	/* set warmup cycle 0xfff */
	iowrite32(RNG_STATUS_WARM_CNT_MASK -
		  (0xfff & RNG_STATUS_WARM_CNT_MASK),
		  base_addr + RNG_STATUS_OFFSET);
	while ((ioread32(base_addr + RNG_STATUS_OFFSET) &
		RNG_STATUS_WARM_CNT_MASK) != RNG_STATUS_WARM_CNT_MASK)
		cpu_relax();

	/* register to the Linux RNG framework */
	error = hwrng_register(&rng_ops);
	if (error) {
		dev_err(dev, "hwrng registration failed\n");
		iounmap(base_addr);
		return error;
	}
	dev_dbg(dev, "hwrng registered\n");

	return 0;
}

static int rng_remove(struct platform_device *pdev)
{
	u32 val;
	void __iomem *base_addr = (void __iomem *)rng_ops.priv;
	/* Unregister driver */
	hwrng_unregister(&rng_ops);

	if (base_addr) {
		/* Disable RNG hardware */
		val = ioread32(base_addr + RNG_CTRL_OFFSET);
		val &= ~RNG_CTRL_RBGEN_MASK;
		iowrite32(val, base_addr + RNG_CTRL_OFFSET);

		val = ioread32(base_addr + RNG_CTRL_OFFSET);
		val &= ~RNG_CTRL_RBG2X_MASK;
		iowrite32(val, base_addr + RNG_CTRL_OFFSET);

		iounmap(base_addr);
	}

	return 0;
}

static void iproc_rng200_restart(void __iomem *rng_base)
{
	u32 val;

	/* Disable RBG */
	val = ioread32(rng_base + RNG200_CTRL_OFFSET);
	val &= ~RNG200_CTRL_RBGEN_MASK;
	val |= RNG200_CTRL_RBGEN_DISABLE;
	iowrite32(val, rng_base + RNG200_CTRL_OFFSET);

	/* Clear all interrupt status */
	iowrite32(0xFFFFFFFFUL, rng_base + RNG200_INT_STATUS_OFFSET);

	/* Reset RNG and RBG */
	val = ioread32(rng_base + RBG_SOFT_RESET_OFFSET);
	val &= ~RBG_RNG_SOFT_RESET_MASK;
	val |= RBG_RNG_SOFT_RESET_ACTIVE;
	iowrite32(val, rng_base + RBG_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RNG200_SOFT_RESET_OFFSET);
	val &= ~RNG200_SOFT_RESET_MASK;
	val |= RNG200_SOFT_RESET_ACTIVE;
	iowrite32(val, rng_base + RNG200_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RNG200_SOFT_RESET_OFFSET);
	val &= ~RNG200_SOFT_RESET_MASK;
	val |= RNG200_SOFT_RESET_CLEAR;
	iowrite32(val, rng_base + RNG200_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RBG_SOFT_RESET_OFFSET);
	val &= ~RBG_RNG_SOFT_RESET_MASK;
	val |= RBG_RNG_SOFT_RESET_CLEAR;
	iowrite32(val, rng_base + RBG_SOFT_RESET_OFFSET);

	/* Enable RBG */
	val = ioread32(rng_base + RNG200_CTRL_OFFSET);
	val &= ~RNG200_CTRL_RBGEN_MASK;
	val |= RNG200_CTRL_RBGEN_ENABLE;
	iowrite32(val, rng_base + RNG200_CTRL_OFFSET);
}

static int iproc_rng200_read(struct hwrng *rng, void *buf, size_t max,
			       bool wait)
{
	u32 status;
	u32 rng_fifo;
	u32 num_remaining = max;

	#define MAX_RESETS_PER_READ	1
	u32 num_resets = 0;

	#define MAX_IDLE_TIME	(1 * HZ)
	unsigned long idle_endtime = jiffies + MAX_IDLE_TIME;

	/* Retrieve HW RNG registers base address. */
	void __iomem *rng_base = (void __iomem *)rng->priv;

	while ((num_remaining > 0) && time_before(jiffies, idle_endtime)) {

		/* Is RNG sane? If not, reset it. */
		status = ioread32(rng_base + RNG200_INT_STATUS_OFFSET);
		if ((status & (RNG200_INT_STATUS_MASTER_FAIL_LOCKOUT_IRQ_MASK |
			RNG200_INT_STATUS_NIST_FAIL_IRQ_MASK)) != 0) {

			if (num_resets >= MAX_RESETS_PER_READ)
				return max - num_remaining;

			iproc_rng200_restart(rng_base);
			num_resets++;
		}

		/* Are there any random numbers available? */
		rng_fifo = ioread32(rng_base + RNG200_FIFO_COUNT_OFFSET);
		if ((rng_fifo & RNG200_FIFO_COUNT_MASK) > 0) {

			if (num_remaining >= sizeof(u32)) {
				/* Buffer has room to store entire word */
				*(u32 *)buf = ioread32(rng_base +
						RNG200_FIFO_DATA_OFFSET);
				buf += sizeof(u32);
				num_remaining -= sizeof(u32);
			} else {
				/* Buffer can only store partial word */
				u32 rnd_number = ioread32(rng_base +
						RNG200_FIFO_DATA_OFFSET);
				memcpy(buf, &rnd_number, num_remaining);
				buf += num_remaining;
				num_remaining = 0;
			}

			/* Reset the IDLE timeout */
			idle_endtime = jiffies + MAX_IDLE_TIME;
		} else {
			if (!wait)
				/* Cannot wait, return immediately */
				break;

			/* Can wait, give others chance to run */
			cpu_relax();
		}
	}

	return max - num_remaining;
}

static struct hwrng iproc_rng200_ops = {
	.name	= "iproc-rng200",
	.read	= iproc_rng200_read,
};

static int iproc_rng200_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *rng_base;
	struct resource *res;
	u32 val;
	int err;

	pr_info("Broadcom IPROC RNG200 Driver\n");
	/* Map peripheral */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get rng resources");
		return -ENODEV;
	}

	rng_base = devm_ioremap_resource(dev, res);
	//rng_base = ioremap(res->start, res->end - res->start);
	if (!rng_base) {
		dev_err(dev, "failed to remap rng regs");
		return -ENODEV;
	}

	iproc_rng200_ops.priv = (unsigned long)rng_base;

	/* Setup RNG. */
	val = ioread32(rng_base + RNG200_CTRL_OFFSET);
	val &= ~RNG200_CTRL_RBGEN_MASK;
	val |= RNG200_CTRL_RBGEN_ENABLE;
	iowrite32(val, rng_base + RNG200_CTRL_OFFSET);

	/* Register driver */
	err = hwrng_register(&iproc_rng200_ops);
	if (err) {
		dev_err(dev, "hwrng registration failed\n");
		return err;
	}
	dev_dbg(dev, "hwrng registered\n");

	return 0;
}

static int iproc_rng200_remove(struct platform_device *pdev)
{
	u32 val;
	void __iomem *rng_base = (void __iomem *)iproc_rng200_ops.priv;

	/* Unregister driver */
	hwrng_unregister(&iproc_rng200_ops);
	if (rng_base) {
		/* Disable RNG hardware */
		val = ioread32(rng_base + RNG200_CTRL_OFFSET);
		val &= ~RNG200_CTRL_RBGEN_MASK;
		val |= RNG200_CTRL_RBGEN_DISABLE;
		iowrite32(val, rng_base + RNG200_CTRL_OFFSET);
	}
	return 0;
}
static int rng_probe_gen(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct device_node *node;
	const char *rng_name;
	node = pdev->dev.of_node;
	rng_name = node->name;

	if (!of_device_is_available(node))
		return -ENODEV;

	of_property_read_string(node, "rng-type", &rng_name);
	if (strcmp(rng_name, "rng200") == 0)
		ret = iproc_rng200_probe(pdev);
	else if (strcmp(rng_name, "rng") == 0)
		ret = rng_probe(pdev);

	return ret;
}

static int rng_remove_gen(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct device_node *node;
	const char *rng_name;
	node = pdev->dev.of_node;
	rng_name = node->name;

	if (!of_device_is_available(node))
		return -ENODEV;

	of_property_read_string(node, "rng-type", &rng_name);
	if (strcmp(rng_name, "rng200") == 0)
		ret = iproc_rng200_remove(pdev);
	else if (strcmp(rng_name, "rng") == 0)
		ret = rng_remove(pdev);

	return ret;
}

static const struct of_device_id bcm_iproc_dt_ids[] = {
	{ .compatible = "brcm,iproc-rng"},
	{  }
};
MODULE_DEVICE_TABLE(of, bcm_iproc_dt_ids);

static struct platform_driver iproc_rng_driver = {
	.driver = {
		.name = "iproc-rng",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm_iproc_dt_ids),
	},
	.probe = rng_probe_gen,
	.remove = rng_remove_gen,
};
module_platform_driver(iproc_rng_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("iProc RNG/RNG200 Random Number Generator driver");
MODULE_LICENSE("GPL v2");
