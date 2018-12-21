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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <plat/types.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include "bcm28nm_otp.h"

static void * baseAddr;

/* Enabled for simulate write-dword and write-bit commands */
#define OTP_WRITE_SIM 0

#if defined(BCMDBG) || defined(BCMDBG_ERR)
#define OTP_ERR(args)	printk args
#else
#define OTP_ERR(args)
#endif 

#ifdef BCMDBG
#define R_REG(reg)             \
                ({ u32 v = ioread32(baseAddr + ((reg) & 0x0fff)); \
                printk("OTP reg read  [0x%08x] = 0x%08x\n", 0x0301c000+(reg), v); \
                v; })
#if OTP_WRITE_SIM
#define W_REG(reg, val)        
#else
#define W_REG(reg, val)        do { \
                                  printk("OTP reg write [0x%08x] = 0x%08x\n", 0x0301c000+(reg), val); \
                                  iowrite32(val, baseAddr + ((reg) & 0x0fff)); \
                               } while(0)
#endif
#else
#define R_REG(reg)             ioread32(baseAddr + ((reg) & 0x0fff))
#if OTP_WRITE_SIM
#define W_REG(reg, val)        
#else
#define W_REG(reg, val)        iowrite32(val, baseAddr + ((reg) & 0x0fff))
#endif
#endif

#define BCM_OTP_INIT_CHK()      \
    do {                        \
         if (!baseAddr) {       \
             return -ENOMEM;    \
		 }                      \
    } while(0)

#define OTPC_RETRIES        10000000	/* # of tries for OTPP */

/* Sequence to enable OTP program */
#define OTPC_PROG_EN_SEQ            { 0xf, 0x4, 0x8, 0xd }

/* OTPC Commands */
#define OTPC_CMD_READ               0x0
#define OTPC_CMD_ReadBurst          0x1
#define OTPC_CMD_OTP_ProgEnable     0x2
#define OTPC_CMD_OTP_ProgDisable    0x3
#define OTPC_CMD_Prescreen_test     0x4
#define OTPC_CMD_Prescreen_RP       0x5
#define OTPC_CMD_Flush              0x6
#define OTPC_CMD_NOP                0x7
#define OTPC_CMD_PROGRAM_WORD       0x8
#define OTPC_CMD_PROGRAM            0xA
#define OTPC_CMD_PROGRAM_RP         0xB
#define OTPC_CMD_PROGRAM_OVST       0xC
#define OTPC_CMD_RELOAD             0xD
#define OTPC_CMD_ERASE              0xE
#define OTPC_CMD_Read_Verify0       0x14
#define OTPC_CMD_Read_Verify1       0x15
#define OTPC_CMD_Program_Lock       0x19

/* OTPC Status Bits */
#define OTPC_STAT_CMD_DONE          (1 << 1)
#define OTPC_STAT_PROG_OK           (1 << 2)
#define OTPC_STAT_FDONE             (1 << 3)
#define OTPC_STAT_CMD_FAIL          (1 << 4)

/* OTPC register definition */
#define CRMU_CHIP_OTPC_RST_CNTRL_BASE 0x058
#define CRMU_CHIP_OTPC_RST_CNTRL__CRMU_JTAG_OTP_RESETB 0
#define OTPC_MODE_REG_BASE 0x800
#define OTPC_MODE_REG__OTPC_MODE 0
#define OTPC_COMMAND_BASE 0x804
#define OTPC_COMMAND__COMMAND_R 0
#define OTPC_COMMAND__COMMAND_WIDTH 6
#define OTPC_CMD_START_BASE 0x808
#define OTPC_CMD_START__START 0
#define OTPC_CPU_STATUS_BASE 0x80c
#define OTPC_CPU_STATUS__OTP_STATUS_R 0
#define OTPC_CPU_STATUS__OTP_STATUS_WIDTH 24
#define OTPC_CPU_DATA_BASE 0x810
#define OTPC_CPUADDR_REG_BASE 0x828
#define OTPC_CPUADDR_REG__OTPC_CPU_ADDRESS_R 0
#define OTPC_CPUADDR_REG__OTPC_CPU_ADDRESS_WIDTH 16
#define OTPC_CPU_WRITE_REG_BASE 0x82c

#define OTPC_CMD_MASK ((1 << OTPC_COMMAND__COMMAND_WIDTH) - 1)
static void
set_command(u32 command) 
{
    W_REG(OTPC_COMMAND_BASE, 
        (command & OTPC_CMD_MASK) << OTPC_COMMAND__COMMAND_R);
}

#define OTPC_ADDR_MASK ((1 << OTPC_CPUADDR_REG__OTPC_CPU_ADDRESS_WIDTH) - 1)
static void
set_cpu_address(u32 address)
{
    W_REG(OTPC_CPUADDR_REG_BASE, 
        (address & OTPC_ADDR_MASK) << OTPC_CPUADDR_REG__OTPC_CPU_ADDRESS_R);
}

static void
set_start_bit(void)
{
    W_REG(OTPC_CMD_START_BASE, 1 << OTPC_CMD_START__START);
}

static void
reset_start_bit(void)
{
    W_REG(OTPC_CMD_START_BASE, 0);
}

static void
write_cpu_data(u32 value)
{
    W_REG(OTPC_CPU_WRITE_REG_BASE, value);
}

static int
poll_cpu_status(u32 mask, u32 value)
{
    u32 status;
    u32 k;
    
	for (k = 0; k < OTPC_RETRIES; k++) {
        status = R_REG(OTPC_CPU_STATUS_BASE);
		if ((status & mask) == value) {
		    break;
		}
	}

#if OTP_WRITE_SIM
#else	
	if (k == OTPC_RETRIES) {
		return -1;
	}
#endif

    return 0;
}

static int
enable_otp_program(void)
{
    static const u32 vals[] = OTPC_PROG_EN_SEQ;
    int i;
    int rv;
    
    /* Write the magic sequence to enable programming */
    set_command(OTPC_CMD_OTP_ProgEnable);
    for(i=0; i<sizeof(vals)/sizeof(u32); i++) {
        write_cpu_data(vals[i]);
        set_start_bit();
        rv = poll_cpu_status(OTPC_STAT_CMD_DONE, OTPC_STAT_CMD_DONE);
        reset_start_bit();
        if (rv) {
            return rv;
        }
    }
    
    return poll_cpu_status(OTPC_STAT_PROG_OK, OTPC_STAT_PROG_OK);
}

static int
disable_otp_program(void)
{
    int rv;
    
    set_command(OTPC_CMD_OTP_ProgDisable);
    set_cpu_address(0);
    set_start_bit();
    rv = poll_cpu_status(OTPC_STAT_PROG_OK, OTPC_STAT_PROG_OK);
    reset_start_bit();
    
    return rv;
}

static void
deassert_otp_reset(void)
{
    W_REG(CRMU_CHIP_OTPC_RST_CNTRL_BASE, 
        R_REG(CRMU_CHIP_OTPC_RST_CNTRL_BASE) | 
        (1 << CRMU_CHIP_OTPC_RST_CNTRL__CRMU_JTAG_OTP_RESETB)
        );
}

static int
bcm28nm_otp_init(phys_addr_t reg_base)
{
    if (baseAddr == NULL) {
        baseAddr = ioremap(reg_base, 0x2000);
    }
    BCM_OTP_INIT_CHK();
    
    /* Make sure it's out of reset */
    deassert_otp_reset();
    
    /* Enable CPU access to OTPC */
    W_REG(OTPC_MODE_REG_BASE, 
          R_REG(OTPC_MODE_REG_BASE) | (1 << OTPC_MODE_REG__OTPC_MODE));
          
    /* Clear START bit for later operations */
    reset_start_bit();

    return 0;
}

static int
bcm28nm_otp_exit(void)
{
    if (baseAddr) {
		iounmap(baseAddr); 
		baseAddr = NULL;
    }

	return 0;
}

int
bcm28nm_otp_read_dword(u32 row, u32 *data)
{
    int rv;

    if (data == NULL) {
        return -1;
    }

    BCM_OTP_INIT_CHK();

    set_command(OTPC_CMD_READ);
    set_cpu_address(row);
    set_start_bit();
    rv = poll_cpu_status(OTPC_STAT_CMD_DONE, OTPC_STAT_CMD_DONE);
    *data = R_REG(OTPC_CPU_DATA_BASE);
    reset_start_bit();

	return rv;
}

int
bcm28nm_otp_write_dword(u32 row, u32 data)
{
    int rv;

    BCM_OTP_INIT_CHK();
    
    rv = enable_otp_program();
    if (rv) {
        return rv;
    }

    set_command(OTPC_CMD_PROGRAM);
    set_cpu_address(row);
    write_cpu_data(data);
    set_start_bit();
    rv = poll_cpu_status(OTPC_STAT_CMD_DONE, OTPC_STAT_CMD_DONE);
    reset_start_bit();
    
    disable_otp_program();

    return rv;
}

int
bcm28nm_otp_write_bit(u32 off)
{
    u32 row, col;
    
    row = off / 32;
    col = off % 32;
    
    return bcm28nm_otp_write_dword(row, 1 << col);
}

static int __devinit 
bcm28nm_otp_probe(struct platform_device *pdev)
{
    struct resource *res;
    int rv;

    /* We only accept one device, and it must have an id of -1 */
    if (pdev->id != -1)
        return -ENODEV;
        
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);        
    if (!res) {
        dev_err(&pdev->dev, "can't get resource 0\n");
        return -EIO;
    }

	rv = bcm28nm_otp_init(res->start);
	if (rv) {
	    OTP_ERR(("Broadcom CE 28nm OTP module init failed.\n"));
		return rv;
	}
    
    return 0;
}

static int __devexit 
bcm28nm_otp_remove(struct platform_device *pdev)
{
	bcm28nm_otp_exit();
    return 0;
}

static struct platform_driver otp_driver = {
    .driver = {
        .name = "brcm-ce-28nm-otp",
        .owner = THIS_MODULE,
    },
    .remove    = __devexit_p(bcm28nm_otp_remove),
    .probe     = bcm28nm_otp_probe,
};

static char banner[] __initdata = KERN_INFO "Broadcom CE 28nm OTP Driver\n";

static int 
bcm_otp_module_init(void)
{
    int rv;

    printk(banner);
    
    rv = platform_driver_register(&otp_driver);
    if (rv) {
        return rv;
    }

    return 0;
}

static void 
bcm_otp_module_exit(void)
{
    platform_driver_unregister(&otp_driver);
}

module_init(bcm_otp_module_init);
module_exit(bcm_otp_module_exit);

EXPORT_SYMBOL(bcm28nm_otp_write_bit);
EXPORT_SYMBOL(bcm28nm_otp_read_dword);
EXPORT_SYMBOL(bcm28nm_otp_write_dword);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom CE 28nm OTP Device Driver");
MODULE_LICENSE("GPL");
