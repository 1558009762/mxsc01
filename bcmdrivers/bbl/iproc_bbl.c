#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "mach/socregs-cygnus.h"
#include "iproc_bbl.h"

#define BBL_REG_ACC_DONE        (1<<0)
#define BBL_IND_SOFT_RST_N      (1<<10)
#define BBL_REG_WR_CMD          (1<<11)
#define BBL_REG_RD_CMD          (1<<12)

#define BBL_POWER_GOOD_BIT      (1<<3)

typedef struct bbl_regs
{
    unsigned int reg_SPRU_BBL_WDATA;
    unsigned int reg_SPRU_BBL_CMD;
    unsigned int reg_SPRU_BBL_STATUS;
    unsigned int reg_SPRU_BBL_RDATA;
} bbl_reg_t;

typedef struct crmu_regs
{
    unsigned int reg_CRMU_PWR_GOOD_STATUS;
    unsigned int reg_CRMU_POWER_REQ_CFG;
    unsigned int reg_CRMU_POWER_POLL;
    unsigned int reg_CRMU_ISO_CELL_CONTROL;
    unsigned int rsvd;
    unsigned int reg_CRMU_SPRU_SOURCE_SEL_STAT;
} crmu_reg_t;

typedef struct auth_regs
{
    unsigned int reg_CRMU_BBL_AUTH_CODE;
    unsigned int reg_CRMU_BBL_AUTH_CHECK;
} bbl_auth_t;


typedef struct _iproc_bbl_t {
	volatile bbl_reg_t *regs;
    volatile crmu_reg_t *crmu_reg;
    volatile bbl_auth_t *auth_reg;
    spinlock_t         lock;
} iproc_bbl_t;

iproc_bbl_t iproc_bbl;

static inline int wait_acc_done(void)
{
    unsigned int v;
    int cnt = 0x10000;

    v = readl(&iproc_bbl.regs->reg_SPRU_BBL_STATUS);
    while(!(v&BBL_REG_ACC_DONE))
    {
        v = readl(&iproc_bbl.regs->reg_SPRU_BBL_STATUS);
        cnt--;
        if(cnt == 0)
            return 0;
    }

    return 1;
}

static inline void bbl_reg_write(unsigned int reg_addr, unsigned int v)
{
    unsigned int cmd = 0;
    
    writel(v, &iproc_bbl.regs->reg_SPRU_BBL_WDATA);
    cmd = (reg_addr&0x3FF)|BBL_REG_WR_CMD|BBL_IND_SOFT_RST_N; /*Write command*/
    writel(cmd, &iproc_bbl.regs->reg_SPRU_BBL_CMD);
    wait_acc_done();
    
    return;
}

static inline unsigned int bbl_reg_read(unsigned int reg_addr)
{
    unsigned int cmd = 0, v;

    cmd = (reg_addr&0x3FF)|BBL_REG_RD_CMD|BBL_IND_SOFT_RST_N; /*Read command*/
    writel(cmd, &iproc_bbl.regs->reg_SPRU_BBL_CMD);
    wait_acc_done();
    v = readl(&iproc_bbl.regs->reg_SPRU_BBL_RDATA);
    
    return v;
}

/******************************************************************************************
                                    Exported APIS
******************************************************************************************/
/*******************************************************************************************************
** write access to secure 1280 bits, referred to as SEC1280
** Parameter mem_addr is the SEC BANK Register Number BBL_SEC0_MEM,BBL_SEC1_MEM etc...refer iproc_bbl.h
** this is the number ranging from 0 to 40 each bank register is 32bit size
** return 0 on success, return -1 on fail
********************************************************************************************************/
int iproc_bbl_1280b_mem_write
    (
    unsigned int mem_addr, 
    unsigned int *p_dat, 
    unsigned int len
    )
{
    int i;

    for(i = 0; i < len; i++)
    {
        if(mem_addr > BBL_SEC39_MEM)
            return -1;

        bbl_reg_write(BBL_SEC0+mem_addr*4, p_dat[i]);
        mem_addr++;
    }

    return 0;
}

/******************************************************************************************************* 
** read access to secure 1280 bits, referred to as SEC1280
** Parameter mem_addr is the SEC BANK Register Number BBL_SEC0_MEM,BBL_SEC1_MEM etc...refer iproc_bbl.h
** this is the number ranging from 0 to 40 each bank register is 32bit size
** *p_dat will point to value and return 0 on success , return -1 on fail
********************************************************************************************************/
int iproc_bbl_1280b_mem_read
    (
    unsigned int mem_addr, 
    unsigned int *p_dat, 
    unsigned int len
    )
{
    int i;

    for(i = 0; i < len; i++)
    {
        if(mem_addr > BBL_SEC39_MEM)
            return -1;

        p_dat[i] = bbl_reg_read(BBL_SEC0+mem_addr*4);
        mem_addr++;
    }

    return 0;
}

int iproc_bbl_1280b_mem_clr(void)
{
    unsigned int v;
    
    v = bbl_reg_read(BBL_CONFIG1);
    v |= sec1280_clr_bit;
    bbl_reg_write(BBL_CONFIG1, v);

    return 0;
}

/************************************************************************

indirect_reg_addr: from BBL_RTC_PER to BBL_WR_BLOCK

************************************************************************/
void iproc_bbl_reg_write(unsigned int indirect_reg_addr, unsigned int v)
{
    bbl_reg_write(indirect_reg_addr, v);
}

unsigned int iproc_bbl_reg_read(unsigned int indirect_reg_addr)
{
    return bbl_reg_read(indirect_reg_addr);
}

int iproc_bbl_tamper_p0n0_enable(void)
{
    unsigned int v, v1;
    
    v = bbl_reg_read(BBL_GLITCH_CFG);
    bbl_reg_write(BBL_GLITCH_CFG, v|(1<<18));

    v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
    v1 = bbl_reg_read(BBL_EN_TAMPERIN);

    bbl_reg_write(BBL_EN_TAMPERIN, v1|0x01); /* Enable P0/N0 pair as static tamper inputs */
    bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v|0x201); /* enable P0/N0 as tamper sources */

    return 0;
}

int iproc_bbl_tamper_p0n0_disable(void)
{
    unsigned int v, v1;
    
    v = bbl_reg_read(BBL_GLITCH_CFG);
    bbl_reg_write(BBL_GLITCH_CFG, v|(1<<18));

    v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
    v1 = bbl_reg_read(BBL_EN_TAMPERIN);

    bbl_reg_write(BBL_EN_TAMPERIN, v1&~0x01); /* disable P0/N0 as tampers */
    bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v&~0x201); /* disable P0/N0 pair as static tamper inputs */

    return 0;
}

int iproc_bbl_emesh_enable(void)
{
    unsigned int v;

    bbl_reg_write(BBL_EMESH_CONFIG, 1<<25);
    bbl_reg_write(BBL_EMESH_CONFIG, ~(1<<25));

    v = bbl_reg_read(BBL_MESH_CONFIG);
    bbl_reg_write(BBL_MESH_CONFIG, v|(7<<5)); /* Enable internal mesh and dynamic mode */

    bbl_reg_write(BBL_EMESH_CONFIG, 0x180FFFF); /* n[8:1] are outputs, p[8:1] are inputs, fc enabled and dyn mode */
    bbl_reg_write(BBL_EMESH_CONFIG_1, 0x3e4);
    bbl_reg_write(BBL_EMESH_PHASE_SEL0, 0xFAC644); /* P1 = phase 1, P2 = phase2, .... */
    v = bbl_reg_read(BBL_GLITCH_CFG);
    bbl_reg_write(BBL_GLITCH_CFG, v|(0x1FE<<18)); /* Enable glitch filter on external mesh pins */

    v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
    bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v|0x1C0000); /* enable external and internal mesh as tamper sources*/

    return 0;
}

int iproc_bbl_emesh_disable(void)
{
    unsigned int v;

    /*Disable as tamper sources*/
    v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
    bbl_reg_write(BBL_TAMPER_SRC_ENABLE, ~(v|0x1c0000)); /* enable external and internal mesh as tamper sources*/

    v = bbl_reg_read(BBL_MESH_CONFIG);
    bbl_reg_write(BBL_MESH_CONFIG, v|(7<<5)); /* disable internal mesh and dynamic mode */

    /* reset mesh logic */
    bbl_reg_write(BBL_EMESH_CONFIG, 1 << 25);
    bbl_reg_write(BBL_EMESH_CONFIG, ~(1 << 25));

    return 0;
}

int iproc_bbl_tmon_enable(void)
{
  unsigned int v, v1;

  /* set high/low temperature triggers */
  /* LOW TEMP = -10C ==> 0x36B */
  /* HIGH TEMP = 85C ==> 0x2A7 */
  v = bbl_reg_read(BBL_TMON_CONFIG);
  v &= 0x00000FFF;
  v = v | (0x36B2A7000);
  bbl_reg_write(BBL_TMON_CONFIG, v);

  v = bbl_reg_read (BBL_TAMPER_SRC_ENABLE);
  v1 = bbl_reg_read (BBL_TMON_CONFIG);

  bbl_reg_write(BBL_TMON_CONFIG, v1 & ~0x1); /* power up temp monitor */
  bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v | 0x7 << 22); /* enable high, low, delta tampers */

  return 0;
}

int iproc_bbl_tmon_disable(void)
{
  unsigned int v, v1;

  v = bbl_reg_read (BBL_TAMPER_SRC_ENABLE);
  v1 = bbl_reg_read (BBL_TMON_CONFIG);

  bbl_reg_write(BBL_TMON_CONFIG, v1 | 0x1); /* power down temp monitor */
  bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v & ~(0x7 << 22)); /* disable high, low, delta tampers */

  return 0;
}

int iproc_bbl_fmon_enable(void)
{
  unsigned int v, v1;

  /* run FMON auto calibration */
  bbl_reg_write(FMON_CNG, 0x02000c30); 
  bbl_reg_write(FMON_CNG_1, 0xA4); 
  mdelay(100);
  bbl_reg_write(FMON_CNG_1, 0xAC);
  mdelay(100);
  do{
      v = bbl_reg_read (BBL_STAT);
  } while(!(v & 0x40000));

  /* get FMON calibration value data[18:11]*/
  v = ((v >> 11) & 0xFF);
  v1 = 0x02000c30 | (v << 16);
  bbl_reg_write (FMON_CNG, v1);

  /* enable as a tamper source */
  v = bbl_reg_read (BBL_TAMPER_SRC_ENABLE);
  bbl_reg_write (BBL_TAMPER_SRC_ENABLE, v | 3 << 25);

  return 0;
}

int iproc_bbl_fmon_disable(void)
{
  unsigned int v;

  /* disable as a tamper source */
  v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
  bbl_reg_write (BBL_TAMPER_SRC_ENABLE, ~(v | 3 << 25));

  return 0;
}

int iproc_bbl_vmon_enable(void)
{
  unsigned int v, v1;

  v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
  v1 = bbl_reg_read(BBL_TAMPER_SRC_ENABLE_1);

  bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v | BBL_TAMPER_SRC_ENABLE__BBL_Vmon_En);
  bbl_reg_write(BBL_TAMPER_SRC_ENABLE_1, v1 | 0x30); /* coincell prot and high voltage en */

  return 0;
}

int iproc_bbl_vmon_disable(void)
{
  unsigned int v, v1;

  v = bbl_reg_read(BBL_TAMPER_SRC_ENABLE);
  v1 = bbl_reg_read(BBL_TAMPER_SRC_ENABLE_1);

  bbl_reg_write(BBL_TAMPER_SRC_ENABLE, v & ~BBL_TAMPER_SRC_ENABLE__BBL_Vmon_En);
  bbl_reg_write(BBL_TAMPER_SRC_ENABLE_1, v1 & ~0x30); /* coincell protection and high voltage en */

  return 0;
}


EXPORT_SYMBOL(iproc_bbl_1280b_mem_write);
EXPORT_SYMBOL(iproc_bbl_1280b_mem_read);
EXPORT_SYMBOL(iproc_bbl_1280b_mem_clr);
EXPORT_SYMBOL(iproc_bbl_reg_write);
EXPORT_SYMBOL(iproc_bbl_reg_read);
EXPORT_SYMBOL(iproc_bbl_tamper_p0n0_enable);
EXPORT_SYMBOL(iproc_bbl_tamper_p0n0_disable);
EXPORT_SYMBOL(iproc_bbl_emesh_enable);
EXPORT_SYMBOL(iproc_bbl_emesh_disable);
EXPORT_SYMBOL(iproc_bbl_tmon_enable);
EXPORT_SYMBOL(iproc_bbl_tmon_disable);
EXPORT_SYMBOL(iproc_bbl_fmon_enable);
EXPORT_SYMBOL(iproc_bbl_fmon_disable);
EXPORT_SYMBOL(iproc_bbl_vmon_enable);
EXPORT_SYMBOL(iproc_bbl_vmon_disable);


static void bbl_init(void)
{
    unsigned int v;
    int read_cnt = 50;

    v = readl(&iproc_bbl.crmu_reg->reg_CRMU_SPRU_SOURCE_SEL_STAT);
    while(v != 0) /*SPRU_SOURCE_SELECT  AON*/
    {
        read_cnt--;
        if(read_cnt == 0)
        {
            printk("BBL: AON power is not available\n");
            return;
        }
        udelay(1);
        v = readl(&iproc_bbl.crmu_reg->reg_CRMU_SPRU_SOURCE_SEL_STAT);
    }
    
    /*Wait for reset cycle*/
    writel(0, &iproc_bbl.regs->reg_SPRU_BBL_CMD);
    udelay(200);
    writel(BBL_IND_SOFT_RST_N, &iproc_bbl.regs->reg_SPRU_BBL_CMD);

    /*- remove BBL related isolation from CRMU*/
    v = readl(&iproc_bbl.crmu_reg->reg_CRMU_ISO_CELL_CONTROL);
    v &= ~((1<<16)|(1<<24));
    writel(v, &iproc_bbl.crmu_reg->reg_CRMU_ISO_CELL_CONTROL);

    /* program CRMU auth_code resister*/
    writel(0x12345678, &iproc_bbl.auth_reg->reg_CRMU_BBL_AUTH_CODE);
    /* program CRMU auth_code_check register*/
    /* auth_code must equal to auth_code_check*/
    writel(0x12345678, &iproc_bbl.auth_reg->reg_CRMU_BBL_AUTH_CHECK);
}

static int __init iproc_bbl_init(void)
{
	int ret = 0;

    memset(&iproc_bbl, 0, sizeof(iproc_bbl_t));
    spin_lock_init(&iproc_bbl.lock);
	iproc_bbl.regs = (volatile bbl_reg_t *)ioremap(IPROC_BBL_REG_BASE, 16);
	if (!iproc_bbl.regs) {
		printk("unable to ioremap MEM resource IPROC_BBL_REG_BASE\n");
		ret = -ENOMEM;
		goto fail;
	}
    printk("REGS = %08x\n", (unsigned int)iproc_bbl.regs);
    
	iproc_bbl.crmu_reg = (volatile crmu_reg_t *)ioremap(IPROC_BBL_POWER_STS, 24);
	if (!iproc_bbl.crmu_reg) {
		printk("unable to ioremap MEM resource IPROC_BBL_POWER_STS\n");
		ret = -ENOMEM;
		goto fail1;
	}
    printk("crmu_reg = %08x\n", (unsigned int)iproc_bbl.crmu_reg);
    
    iproc_bbl.auth_reg = (volatile bbl_auth_t *)ioremap(IPROC_BBL_AUTH_REG, 8);
    if (!iproc_bbl.auth_reg) {
        printk("unable to ioremap MEM resource IPROC_BBL_AUTH_REG\n");
        ret = -ENOMEM;
        goto fail2;
    }
    printk("auth_reg = %08x\n", (unsigned int)iproc_bbl.auth_reg);

    bbl_init();

    printk("BBL init successfully\n");
	return 0;

fail2:
    iounmap(iproc_bbl.crmu_reg);
fail1:
    iounmap(iproc_bbl.regs);
fail:
	return ret;
}
module_init(iproc_bbl_init);

static void __exit iproc_bbl_exit(void)
{
    iounmap(iproc_bbl.auth_reg);
    iounmap(iproc_bbl.crmu_reg);
	iounmap(iproc_bbl.regs);
    printk("BBL exit\n");
}
module_exit(iproc_bbl_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC PBBL Driver");
MODULE_LICENSE("GPL");



