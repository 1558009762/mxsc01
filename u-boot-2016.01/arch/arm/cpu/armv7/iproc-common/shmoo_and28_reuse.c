/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <malloc.h>
#include <linux/mtd/mtd.h>
#include <spi.h>
#include <spi_flash.h>
#include <nand.h>
#include <asm/arch/socregs.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/iproc-common/shmoo_public.h>

#if defined(CONFIG_SHMOO_AND28_REUSE) && CONFIG_SHMOO_AND28_REUSE

#include "shmoo_and28/shmoo_and28.h"

#define SHMOO28_HEADER_MAGIC      0xdeadbeaf
#define SHMOO_MIN_BLOCK_SIZE      0x10000

/* Enable/disable debugging */
#define SHMOO28_DEBUG   0

struct shmoo_sig_t {
    uint32_t        magic;
    uint32_t        length;
    uint32_t        checksum;
};

struct shmoo_datagram_t {
    struct shmoo_sig_t sig;
    and28_shmoo_config_param_t config;
};

#if defined(CONFIG_IPROC_NAND) && defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND

/* If Shmoo data is saved to NAND */
static int write_shmoo_to_flash(void *buf, int length)
{
    nand_info_t *nand;
    int ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_NAND_OFFSET;
    uint32_t end = offset + CONFIG_SHMOO_REUSE_NAND_RANGE;
    uint32_t blksize;
    
    /* Get flash handle */
    nand = &nand_info[0];
    if (nand->size < offset || nand->writesize == 0 || nand->erasesize == 0) {
        printf("Failed to initialize NAND flash for saving Shmoo values!\n");
        return -1;
    }

    /* For NAND with bad blocks, we always erase all blocks in the range */
    {
        nand_erase_options_t opts;
        memset(&opts, 0, sizeof(opts));
        opts.offset = offset / nand->erasesize * nand->erasesize;
        opts.length = (CONFIG_SHMOO_REUSE_NAND_RANGE - 1) / nand->erasesize * nand->erasesize + 1;
        opts.quiet  = 1;
        ret = nand_erase_opts(nand, &opts);
        if (ret) {
            printf("NAND flash erase failed, error=%d\n", ret);
            return ret;
        }
    }
    
    /* Write data */
    blksize = nand->erasesize > SHMOO_MIN_BLOCK_SIZE? 
        nand->erasesize : SHMOO_MIN_BLOCK_SIZE;
    while (offset < end) {
        if (nand_block_isbad(nand, offset)) {
            offset += blksize;
            continue;
        }
        ret = nand_write(nand, offset, (size_t *)&length, (u_char *)buf);
        if (ret) {
            printf("NAND flash write failed, error=%d\n", ret);
        }
        break;
    }
    return ret;
}

#elif defined (CONFIG_SPI_FLASH) && defined(CONFIG_ENV_IS_IN_SPI_FLASH) && CONFIG_ENV_IS_IN_SPI_FLASH

/* If shmoo data is saved to SPI */
static int write_shmoo_to_flash(void *buf, int length)
{
    struct spi_flash *flash;
    int erase = 0;
    volatile uint32_t *flptr;
    int i, j, ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_QSPI_OFFSET;
    
    /* Check if erasing is required */
    flptr = (volatile uint32_t *)(IPROC_QSPI_MEM_BASE + offset / 4 * 4);    
    j = (length - 1) / 4 + 1;
    for(i=0; i<j; i++, flptr++) {
        if (*flptr != 0xFFFFFFFF) {
            erase = 1;
            break;
        }
    }
    
    /* Probe flash */
    flash = spi_flash_probe(
                CONFIG_ENV_SPI_BUS, 
                CONFIG_ENV_SPI_CS, 
                CONFIG_ENV_SPI_MAX_HZ, 
                CONFIG_ENV_SPI_MODE
                );
    if (!flash) {
        printf("Failed to initialize SPI flash for saving Shmoo values!\n");
        return -1;
    }
    
    /* Erase if necessary */
    if (erase) {
        ret = spi_flash_erase(
                flash, 
                offset / flash->sector_size * flash->sector_size, 
                flash->sector_size
                );
        if (ret) {
            printf("SPI flash erase failed, error=%d\n", ret);
            spi_flash_free(flash);
            return ret;
        }
    }
    
    /* Write data */
    ret = spi_flash_write(flash, offset, length, buf);
    if (ret) {
        printf("SPI flash write failed, error=%d\n", ret);
    }
   
    /* Free flash instance */
    spi_flash_free(flash);
    
    return ret;
}

#elif defined (CONFIG_ENV_IS_IN_FLASH)

/* If shmoo data is saved to NOR flash */
static int write_shmoo_to_flash(void *buf, int length)
{
    int erase = 0;
    volatile uint32_t *flptr, shmoo_start;
    int i, j, ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_NOR_OFFSET;
    int sect_len;

    /* Check if erasing is required */
    flptr = (volatile uint32_t *)(IPROC_NOR_MEM_BASE + offset / 4 * 4);
    shmoo_start = flptr;
    j = (length - 1) / 4 + 1;
    for(i=0; i<j; i++, flptr++) {
        if (*flptr != 0xFFFFFFFF) {
            erase = 1;
            break;
        }
    }

    sect_len = (((length / 0x20000) + 1)*0x20000 - 1);   
    /* Erase if necessary */
    if (erase) {
        ret = flash_sect_erase((ulong)shmoo_start, (ulong)shmoo_start + sect_len);
        if (ret) {
            printf("NOR flash erase failed, error=%d, start addr: 0x%x, end addr: 0x%x\n", 
                            ret, (ulong)shmoo_start, (ulong)shmoo_start + sect_len);
            return ret;
        }
    }

    /* Write data */
    ret = flash_write((char *)buf, (ulong)shmoo_start, length);

    if (ret) {
        printf("NOR flash write failed, error=%d\n", ret);
    }

    return ret;

}
#else
 #error Flash (SPI or NAND) must be enabled 
#endif

/* Return flash pointer; or NULL if validation failed */
static volatile uint32_t *validate_flash_shmoo_values(void)
{
    volatile char *ptr;
    volatile char *flptr;
    struct shmoo_sig_t *sig;
    uint32_t checksum;
    int offset;
    int i;
    int length;

    /* Calculate required length (register/value pair) */
    length = sizeof(and28_shmoo_config_param_t);
    if (SHMOO28_DEBUG) 
        printf("SHMOO28 Control structure size: %d\n", length);
    
#if defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND
    /* Read SHMOO data from NAND */
    flptr = (volatile char *)(IPROC_NAND_MEM_BASE + CONFIG_SHMOO_REUSE_NAND_OFFSET);
//    offset = (CONFIG_SHMOO_REUSE_NAND_RANGE - 1) / SHMOO_MIN_BLOCK_SIZE * SHMOO_MIN_BLOCK_SIZE;
    offset = 0;
#elif defined (CONFIG_ENV_IS_IN_FLASH) 
    /* Read SHMOO data from NOR */
    flptr = (volatile char *)(IPROC_NOR_MEM_BASE + CONFIG_SHMOO_REUSE_NOR_OFFSET);
    offset = 0;
#else
    /* Read SHMOO data from SPI */
    flptr = (volatile char *)(IPROC_QSPI_MEM_BASE + CONFIG_SHMOO_REUSE_QSPI_OFFSET);
    offset = 0;
#endif
    
    if (SHMOO28_DEBUG) 
        printf("SHMOO flash address: 0x%x, offset: 0x%x\n", (int)flptr, (int) offset);

    /* Construct signature */
    sig = (struct shmoo_sig_t *) (flptr + offset);

    if (sig->magic == SHMOO28_HEADER_MAGIC) {
        if (SHMOO28_DEBUG) 
            printf("Shmoo signature matched\n");
    } else {
        return NULL;
    }
 
    if (sig->length == length) {
        if (SHMOO28_DEBUG) 
            printf("Shmoo data length matched\n");
    } else {
        return NULL;
    }

    /* Verify checksum */
    checksum = 0;
    ptr = (char *) (flptr + offset + sizeof (struct shmoo_sig_t));
    for(i=0; i< length; i++, ptr++) {
        checksum += *ptr;
    }
    if (sig->checksum != checksum) {
        printf("Shmoo checksum is not matching!\n");
        return NULL;
    }
    
    if (SHMOO28_DEBUG) 
        printf("Shmoo checksum verified\n");
    return (volatile uint32_t *)(flptr + offset + sizeof(struct shmoo_sig_t));
}

void restore_shmoo_config(and28_shmoo_config_param_t *shmoo_control_para)
{
    volatile char *ptr;
	volatile char *sptr;
    volatile char *flptr;
    int i;
    int length, offset;

    /* Calculate required length (register/value pair) */
    length = sizeof(and28_shmoo_config_param_t);
    if (SHMOO28_DEBUG) 
        printf("SHMOO28 Control structure size: %d\n", length);
    
#if defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND
    /* Read SHMOO data from NAND */
    flptr = (volatile uint32_t *)(IPROC_NAND_MEM_BASE + CONFIG_SHMOO_REUSE_NAND_OFFSET);
    offset = 0;
#elif defined (CONFIG_ENV_IS_IN_FLASH) 
    /* Read SHMOO data from NOR */
    flptr = (volatile uint32_t *)(IPROC_NOR_MEM_BASE + CONFIG_SHMOO_REUSE_NOR_OFFSET);
    offset = 0;
#else
    /* Read SHMOO data from SPI */
    flptr = (volatile char *)(IPROC_QSPI_MEM_BASE + CONFIG_SHMOO_REUSE_QSPI_OFFSET);
    offset = 0;
#endif
    
    if (SHMOO28_DEBUG) 
        printf("SHMOO flash address: 0x%x, offset: 0x%x\n", (int)flptr, (int) offset);
    ptr = (char *) (flptr + offset + sizeof(struct shmoo_sig_t));
	sptr = (char *)shmoo_control_para;
	
    for (i=0; i < length; i++) {
	   /* Copy shmoo data from flash to shmoo pointer */    
       *sptr = *ptr;
	   sptr++;
	   ptr++;
	}
   if (SHMOO28_DEBUG) {
       display_shmoo_stored_data(shmoo_control_para);
   }
}

int is_shmoo_data_valid(void)
{
    volatile uint32_t *flptr;
    int valid = 1;
    
    /* Validate values in flash */
    printf("Validate Shmoo parameters stored in flash ..... ");
    flptr = validate_flash_shmoo_values();
    if (flptr == NULL) {
        printf("failed\n");
        return 0;
    }
    printf("OK\n");

    /* Check if user wants to skip restoring and run Shmoo */
    if (CONFIG_SHMOO_REUSE_DELAY_MSECS > 0) {
        char c = 0;
        unsigned long start;
        printf("Press Ctrl-C to run Shmoo ..... ");
        start = get_timer(0);
        while(get_timer(start) <= CONFIG_SHMOO_REUSE_DELAY_MSECS) {
            if (tstc()) {
                c = getc();
                if (c == 0x03) {
                    printf("Pressed.\n");
                    printf("Do you want to run the Shmoo? [y/N] ");
                    for(;;) {
                        c = getc();
                        if (c == 'y' || c == 'Y') {
                            printf("Y\n");
                            valid = 0;
                            break;
                        } else if (c == '\r' || c == 'n' || c == 'N') {
                            if (c != '\r')
                                printf("N\n");
                            break;
                        }
                    }
                    break;
                } else {
                    c = 0;
                }
            }
        }
        if (c == 0) 
            printf("skipped\n");
    }

    return valid;
}

void display_shmoo_stored_data(and28_shmoo_config_param_t *config)
{
    int ix, jx;

    for (ix=0; ix <  SHMOO_AND28_PHY_NOF_AD; ix++) {  
        printf("control_regs_ad[%d]      : 0x%x\n", ix, config->control_regs_ad[ix]);
    }

    for (ix=0; ix <  SHMOO_AND28_PHY_NOF_BA; ix++) {  
        printf("control_regs_ba[%d]      : 0x%x\n", ix, config->control_regs_ba[ix]);
    }

    for (ix=0; ix < SHMOO_AND28_PHY_NOF_AUX; ix++) {  
        printf("control_regs_aux[%d]     : 0x%x\n", ix, config->control_regs_aux[ix]);
    }

    for (ix=0; ix <  SHMOO_AND28_PHY_NOF_CS; ix++) {  
        printf("control_regs_cs[%d]      : 0x%x\n", ix, config->control_regs_cs[ix]);
    }

    printf("control_regs_par              : 0x%x\n",    config->control_regs_par);
    printf("control_regs_ras_n            : 0x%x\n",    config->control_regs_ras_n);
    printf("control_regs_cas_n            : 0x%x\n",    config->control_regs_cas_n);
    printf("control_regs_cke              : 0x%x\n",    config->control_regs_cke);
    printf("control_regs_rst_n            : 0x%x\n",    config->control_regs_rst_n);
    printf("control_regs_odt              : 0x%x\n",    config->control_regs_odt);
    printf("control_regs_we_n             : 0x%x\n",    config->control_regs_we_n);
    printf("control_regs_vref_dac_control : 0x%x\n",  config->control_regs_vref_dac_control);
 
    for (ix =0; ix < SHMOO_AND28_BYTES_PER_INTERFACE; ix++) {  
        printf("wr_vdl_dqsp[%d]           : 0x%x\n",  ix, config->wr_vdl_dqsp[ix]);
        printf("wr_vdl_dqsn[%d            : 0x%x\n",  ix, config->wr_vdl_dqsn[ix]);
        printf("wr_vdl_dm[%d]             : 0x%x\n",  ix, config->wr_vdl_dm[ix]);
        printf("wr_vdl_edc[%d]            : 0x%x\n",  ix, config->wr_vdl_edc[ix]);
        printf("wr_chan_dly_cyc[%d]       : 0x%x\n",  ix, config->wr_chan_dly_cyc[ix]);

        printf("rd_vdl_dqsp[%d]           : 0x%x\n",  ix, config->rd_vdl_dqsp[ix]);
        printf("rd_vdl_dqsn[%d]           : 0x%x\n",  ix, config->rd_vdl_dqsn[ix]);
        printf("rd_vdl_dmp[%d]            : 0x%x\n",  ix, config->rd_vdl_dmp[ix]);
        printf("rd_vdl_dmn[%d]            : 0x%x\n",  ix, config->rd_vdl_dmn[ix]);
        printf("rd_en_dly_cyc[%d]         : 0x%x\n",  ix, config->rd_en_dly_cyc[ix]);
        printf("rd_control[%d]            : 0x%x\n",  ix, config->rd_control[ix]);
    }

    for (ix=0; ix < SHMOO_AND28_BYTES_PER_INTERFACE; ix++) {
        for (jx=0; jx < SHMOO_AND28_BYTE; jx++) {
            printf("rd_vdl_dqp[%d][%d]   : 0x%x\n", ix, jx, config->rd_vdl_dqp[ix][jx]);
            printf("rd_vdl_dqn[%d][%d]   : 0x%x\n", ix, jx, config->rd_vdl_dqn[ix][jx]);
            printf("wr_vdl_dq[%d][%d]    : 0x%x\n", ix, jx, config->wr_vdl_dq[ix][jx]);
        }
    }

    for (ix=0; ix < SHMOO_AND28_BYTES_PER_INTERFACE; ix++) {
        for (jx=0; jx < SHMOO_AND28_PHY_NOF_CS; jx++) {
            printf("rd_en_vdl_cs[%d][%d] : 0x%x\n", ix, jx, config->rd_en_vdl_cs[ix][jx]);
        }
    }
}

void save_shmoo_to_flash(void)
{
    volatile uint32_t *flptr;
    int length;
    uint32_t checksum;
    int i;
    struct shmoo_datagram_t shmoo_buf;
    and28_shmoo_config_param_t config_param;
    char *fptr, *sptr;

    /* Get current SHMOO values */
    memset((char *)&config_param, 0x0, sizeof(and28_shmoo_config_param_t));
    soc_and28_shmoo_ctl(0, 0, SHMOO_AND28_SHMOO_RSVP, 0, 1, SHMOO_AND28_ACTION_SAVE, &config_param);

    /* Check if the flash already contains the same SHMOO values */
    flptr = validate_flash_shmoo_values();
    if (flptr != NULL) {
        if (!memcmp((void *)flptr, &config_param, sizeof(and28_shmoo_config_param_t))) {
            /* Not need to save */
            return;
        }
    }
    
    /* calculate checksum and save to buffer */
    checksum = 0;
    sptr = (char *)&config_param;
    length = sizeof(and28_shmoo_config_param_t);
    for(i=0; i< length; i++, sptr++) {
        checksum += *sptr;
    }

    shmoo_buf.sig.checksum = checksum;
    shmoo_buf.sig.length = length;
    shmoo_buf.sig.magic = SHMOO28_HEADER_MAGIC;
    
    /* Copy passed config to buffer */
    fptr = (char *)&shmoo_buf.config;
    sptr = (char *)&config_param;

    for (i=0; i < length; i++) {
        *fptr = *sptr;
        fptr++;
        sptr++;
    }

#if (SHMOO28_DEBUG == 1)
    printf("\n shmoo config data to flash\n");
    display_shmoo_stored_data(&shmoo_buf.config);

    printf("\n shmoo config data from shmoo API\n");
    display_shmoo_stored_data(&config_param);
#endif

    /* Now write buffer to flash */
    printf("Writing Shmoo values into flash .....\n");
    write_shmoo_to_flash((char *)&shmoo_buf, sizeof(struct shmoo_datagram_t));
}

#endif /* CONFIG_SHMOO_AND28_REUSE */
