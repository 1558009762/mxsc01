/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <sdhci.h>

#include <asm/arch/socregs.h>
#include <asm/iproc-common/reg_utils.h>

struct iproc_sdhci_host {
    struct sdhci_host host;
    u32 shadow_cmd;
    u32 shadow_blk;
};

/*
* SDIO_CAPS_L
*
* Field                 Bit(s)
* ===========================
* DDR50                     31
* SDR104                    30
* SDR50                     29
* SLOTTYPE                  28:27
* ASYNCHIRQ                 26
* SYSBUS64                  25
* V18                       24
* V3                        23
* V33                       22
* SUPRSM                    21
* SDMA                      20
* HSPEED                    19
* ADMA2                     18
* EXTBUSMED                 17
* MAXBLK                    16:15
* BCLK                      14:7
* TOUT                      6
* TOUTFREQ                  5:0
*/
#define SDIO_CAPS_L                 0xA17f6470

/*
* SDIO_CAPS_H
*
* Field                 Bit(s)
* ===========================
* reserved                  31:20
* SPIBLOCKMODE              19
* SPIMODE_CAP               18
* CLOCKMULT                 17:10
* RETUNE_MODE               9:8
* USETUNE_SDR50             7
* TMRCNT_RETUNE             6:3
* DRVR_TYPED                2
* DRVR_TYPEC                1
* DRVR_TYPEA                0
*/
#define SDIO_CAPS_H                 0x000C000f

/*
* Preset value
*
* Field                 Bit(s)
* ===========================
* Driver Strength           12:11
* Clock Generator           10
* SDCLK Frequeency          9:0
*/

/*
* SDIO_PRESETVAL1
*
* Field                 Bit(s)      Description
* ============================================================
* DDR50_PRESET              25:13   Preset Value for DDR50
* DEFAULT_PRESET            12:0    Preset Value for Default Speed
*/
#define SDIO_PRESETVAL1             0x01004004

/*
* SDIO_PRESETVAL2
*
* Field                 Bit(s)      Description
* ============================================================
* HIGH_SPEED_PRESET         25:13   Preset Value for High Speed
* INIT_PRESET               12:0    Preset Value for Initialization
*/
#define SDIO_PRESETVAL2             0x01004100

/*
* SDIO_PRESETVAL3
*
* Field                 Bit(s)      Description
* ============================================================
* SDR104_PRESET             25:13   Preset Value for SDR104
* SDR12_PRESET              12:0    Preset Value for SDR12
*/
#define SDIO_PRESETVAL3             0x00000004

/*
* SDIO_PRESETVAL4
*
* Field                 Bit(s)      Description
* ============================================================
* SDR25_PRESET              25:13   Preset Value for SDR25
* SDR50_PRESET              12:0    Preset Value for SDR50
*/
#define SDIO_PRESETVAL4             0x01005001


static inline void iproc_sdhci_raw_writel(struct sdhci_host *host, u32 val, int reg)
{
    reg32_write((uint32_t *)(host->ioaddr + reg), val);
}

static inline u32 iproc_sdhci_raw_readl(struct sdhci_host *host, int reg)
{
    return reg32_read((uint32_t *)(host->ioaddr + reg));
}

static void iproc_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
    iproc_sdhci_raw_writel(host, val, reg);
}

static void iproc_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
    struct iproc_sdhci_host *iproc_host = (struct iproc_sdhci_host *)host;
    u32 oldval, newval;
    u32 word_num = (reg >> 1) & 1;
    u32 word_shift = word_num * 16;
    u32 mask = 0xffff << word_shift;

    if (reg == SDHCI_COMMAND) {
        if (iproc_host->shadow_blk != 0) {
            iproc_sdhci_raw_writel(host, iproc_host->shadow_blk, SDHCI_BLOCK_SIZE);
            iproc_host->shadow_blk = 0;
        }
        oldval = iproc_host->shadow_cmd;
    } else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
        oldval = iproc_host->shadow_blk;
    } else {
        oldval = iproc_sdhci_raw_readl(host, reg & ~3);
    }
    newval = (oldval & ~mask) | (val << word_shift);

    if (reg == SDHCI_TRANSFER_MODE) {
        iproc_host->shadow_cmd = newval;
    } else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
        iproc_host->shadow_blk = newval;
    } else {
        iproc_sdhci_raw_writel(host, newval, reg & ~3);
    }
}

static void iproc_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
    u32 oldval, newval;
    u32 byte_num = reg & 3;
    u32 byte_shift = byte_num * 8;
    u32 mask = 0xff << byte_shift;

    oldval = iproc_sdhci_raw_readl(host, reg & ~3);
    newval = (oldval & ~mask) | (val << byte_shift);
    
    iproc_sdhci_raw_writel(host, newval, reg & ~3);
}

static u32 iproc_sdhci_readl(struct sdhci_host *host, int reg)
{
    return iproc_sdhci_raw_readl(host, reg);
}

static u16 iproc_sdhci_readw(struct sdhci_host *host, int reg)
{
    u32 val;
    u32 word_num = (reg >> 1) & 1;
    u32 word_shift = word_num * 16;

    val = iproc_sdhci_raw_readl(host, (reg & ~3));
    return (val >> word_shift) & 0xffff;
}

static u8 iproc_sdhci_readb(struct sdhci_host *host, int reg)
{
    u32 val;
    u32 byte_num = reg & 3;
    u32 byte_shift = byte_num * 8;

    val = iproc_sdhci_raw_readl(host, (reg & ~3));
    return (val >> byte_shift) & 0xff;
}

static const struct sdhci_ops iproc_sdhci_ops = {
    .write_l = iproc_sdhci_writel,
    .write_w = iproc_sdhci_writew,
    .write_b = iproc_sdhci_writeb,
    .read_l = iproc_sdhci_readl,
    .read_w = iproc_sdhci_readw,
    .read_b = iproc_sdhci_readb,
};

int board_mmc_init(bd_t *bis)
{
    struct iproc_sdhci_host *iproc_host;
    struct sdhci_host *host;
    u32 val;
    
    iproc_host = malloc(sizeof(*iproc_host));
    if (!iproc_host) {
        printf("sdhci_host malloc fail!\n");
        return 1;
    }

    /* Enable the SDIO clock */
    val = reg32_read((uint32_t *)(SDIO_IDM0_IO_CONTROL_DIRECT));
    val |= (1 << SDIO_IDM0_IO_CONTROL_DIRECT__CMD_COMFLICT_DISABLE);
    val |= (1 << SDIO_IDM0_IO_CONTROL_DIRECT__FEEDBACK_CLK_EN);
    val |= (1 << SDIO_IDM0_IO_CONTROL_DIRECT__clk_enable);
    reg32_write((uint32_t *)(SDIO_IDM0_IO_CONTROL_DIRECT), val);

    /* Set the 1.8v fail control for HR3.
     * This setting will not impact the uboot SD/MMC driver, since uboot doesn't 
     * support 1.8v. The 1.8v SDIO will be supportted in Kernel. */
    val = reg32_read((uint32_t *)(IPROC_WRAP_SDIO_1P8_FAIL_CONTROL));
    val |= (1 << IPROC_WRAP_SDIO_1P8_FAIL_CONTROL__SDIO_VDDO_18V_FAIL_SOVW);
    val &= ~(1 << IPROC_WRAP_SDIO_1P8_FAIL_CONTROL__SDIO_UHS1_18V_VREG_FAIL);
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_1P8_FAIL_CONTROL), val);

    /* Release reset */
    reg32_write((uint32_t *)(SDIO_IDM0_IDM_RESET_CONTROL), 0x1);
    udelay(1000);
    reg32_write((uint32_t *)(SDIO_IDM0_IDM_RESET_CONTROL), 0x0);

    /*
    * Configure SDIO host controller capabilities
    * (common setting for all SDIO controllers)
    */
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL), SDIO_CAPS_H);
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL1), SDIO_CAPS_L);

    /*
    * Configure SDIO host controller preset values
    * (common setting for all SDIO controllers)
    */
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL2), SDIO_PRESETVAL1);
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL3), SDIO_PRESETVAL2);
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL4), SDIO_PRESETVAL3);
    reg32_write((uint32_t *)(IPROC_WRAP_SDIO_CONTROL5), SDIO_PRESETVAL4);

    host = &iproc_host->host;
    host->name = "iproc_sdhci";
    host->ioaddr = (void *)SDIO0_eMMCSDXC_SYSADDR;
    host->quirks = SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER | SDHCI_QUIRK_WAIT_SEND_CMD;
    host->ops = &iproc_sdhci_ops;
    host->version = sdhci_readw(host, SDHCI_HOST_VERSION);

    /* WR for HR3 SDIO */
    host->quirks |= (SDHCI_QUIRK_FORCE_DRIVER_STRENGTH);
    host->drv_type = SDHCI_DRIVER_TYPE_A;

    debug("SDIO controller capabilities, cap1: %.8x and cap2: %.8x\n", 
                                sdhci_readl(host, SDHCI_CAPABILITIES),
                                sdhci_readl(host, SDHCI_CAPABILITIES_1));
    add_sdhci(host, 0, 0);
    return 0;
}
