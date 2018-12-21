/*****************************************************************************
* Copyright 2011 Broadcom Corporation.  All rights reserved.
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
*****************************************************************************/

/*
 * V4L2 Driver for unicam/rhea camera host
 */

#define pr_fmt(fmt) "unicam_camera: " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-contig.h>
#include <media/soc_mediabus.h>
#include <media/videobuf2-core.h>

#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mediabus.h>
//#include <asm/bitops.h>
#include <mach/memory.h>
#include <mach/socregs-cygnus.h>

#include "iproc_csi0.h"
#include "iproc_videodev2.h"
#include "iproc_rdb_util.h"
/*
 * Unicam camera host exports
 */

#define vb2_plane_dma_addr vb2_dma_contig_plane_dma_addr

#define UNICAM_BUF_MAGIC		0xBABEFACE
#define UNICAM_CAM_DRV_NAME		"unicam-camera"

#define UNICAM_CAPTURE_MODE		BUFFER_TRIGGER
#define UC_TIMEOUT_MS			500
#define UC_RETRY_CNT			3

uint32_t asiu_cam_init_seq(void);

/*****************************************************************************
 * Copyright 2011 Broadcom Corporation.  All rights reserved.
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
 *****************************************************************************/

struct mm_csi0_generic {
	int devbusy;
	enum host_mode mode;
	enum afe_num afe;
	enum csi2_lanes lanes;
	int db_en;
	int trigger;
	struct semaphore sem; /* Do a sema init on this at init time */
};

/* For now using local addresses */
/* This will be changed once the same is
   available via platform driver */
unsigned int V_BASE;		
unsigned int MM_CFG_BASE;

struct mm_csi0_generic cam_state = {
	0,
	INVALID,
};
#define post_log printk 

#define CAM_TOP_REGS_CAMCTL_Config  0x180a1000
uint32_t CAM_TOP_REGS_CAMCTL_base; /* pointer to iomapped register memory region */

static uint32_t cpu_rd_single(uint32_t addr, int size) {
    return ((size == 1) ?  *((volatile uint8_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))) :
                            ((size == 2) ? *((volatile uint16_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))) :
                            (*((volatile uint32_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))))));
};

static void cpu_wr_single(uint32_t addr, uint32_t data, int size) {
    if(size == 1) *((volatile uint8_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))) = data ;
    else if(size == 2) *((volatile uint16_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))) = data ;
    else *((volatile uint32_t *)(V_BASE - CAM_TOP_REGS_CAMCTL_Config + (addr))) = data;
};

struct config_param {
	uint32_t seq_asiu_cam_soft_reset_enable ;
	uint32_t seq_asiu_cam_cpi_data_width ;
	uint32_t seq_asiu_cam_cpi_mode ;
	uint32_t seq_asiu_cam_unpack_option ;
	uint32_t seq_asiu_cam_pack_option ;
	uint32_t cfg_asiu_cam_image_buff_start_addr ;
	uint32_t cfg_asiu_cam_image_buff_end_addr ;
	uint32_t cfg_asiu_cam_data_buff_end_addr ;
	uint32_t cfg_asiu_cam_data_buff_start_addr ;
	uint32_t cfg_asiu_cam_image_line_stride ;
	uint32_t seq_asiu_horz_window_enable ;
	uint32_t cfg_asiu_horz_window_value ;
	uint32_t seq_asiu_verti_window_enable ;
	uint32_t cfg_asiu_verti_window_value ;
	uint32_t seq_asiu_multiple_frames_en ;
	uint32_t cfg_asiu_cam_image_buff_start_addr1 ;
	uint32_t cfg_asiu_cam_image_buff_end_addr1 ;
	uint32_t cam_non_secure_en ;
	uint32_t cpi_clk_neg_edg_en ;
	uint32_t cpi_burst_space_enable ;
};
struct config_param test_config;
#define IMAGE_START 0x60100000
#define IMAGE_END   0x60200000
#define DATA_END  0x60300000
#define DATA_START 0x60400000
/* Bit definitions for control register */
#define LCD_CNTL_LCDEN		(1 << 0)
#define LCD_CNTL_BPP1		(0 << 1)
#define LCD_CNTL_BPP2		(1 << 1)
#define LCD_CNTL_BPP4		(2 << 1)
#define LCD_CNTL_BPP8		(3 << 1)
#define LCD_CNTL_BPP16		(4 << 1) /*1555*/
#define LCD_CNTL_BPP24		(5 << 1)
#define LCD_CNTL_BPP16_565	(6 << 1)
#define LCD_CNTL_BPP12_444	(7 << 1)

#define LCD_CNTL_LCDBW			(1 << 4)
#define LCD_CNTL_LCDTFT			(1 << 5)
#define LCD_CNTL_LCDMONO8		(1 << 6)
#define LCD_CNTL_LCDDUAL		(1 << 7)
#define LCD_CNTL_BGR			(1 << 8)
#define LCD_CNTL_BEBO			(1 << 9)
#define LCD_CNTL_BEPO			(1 << 10)
#define LCD_CNTL_LCDPWR			(1 << 11)
#define LCD_CNTL_LCDVCOMP(x)	((x) << 12)
#define LCD_CNTL_LDMAFIFOTIME	(1 << 15)
#define LCD_CNTL_WATERMARK		(1 << 16)

#define CNTL (LCD_CNTL_LCDEN | LCD_CNTL_LCDTFT | LCD_CNTL_LCDPWR | LCD_CNTL_LCDVCOMP(3) | LCD_CNTL_WATERMARK )

static spinlock_t lock;
void unicam_reg_dump(void)
{
	u32 base = V_BASE;
	pr_info("  CAM_CTL 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMCTL_OFFSET));
	pr_info("  CAM_STA 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMSTA_OFFSET));
	pr_info("  CAM_ANA 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMANA_OFFSET ));
	pr_info("  CAM_PRI 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMPRI_OFFSET ));
	pr_info("  CAM_CLK 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMCLK_OFFSET ));
	pr_info("  CAM_CLT 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMCLT_OFFSET ));
	pr_info("  CAM_DAT0 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMDAT0_OFFSET ));
	pr_info("  CAM_DAT1 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMDAT1_OFFSET ));
	pr_info("  CAM_DLT 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMDLT_OFFSET ));
	pr_info("  CAM_ICTL 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMICTL_OFFSET ));
	pr_info("  CAM_ISTA 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMISTA_OFFSET ));
	pr_info("  CAM_IDI 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMIDI0_OFFSET ));
	pr_info("  CAM_IPIPE 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMIPIPE_OFFSET ));
	pr_info("  CAM_IBSA 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMIBSA0_OFFSET ));
	pr_info("  CAM_IBEA 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMIBSA0_OFFSET ));
	pr_info("  CAM_IBLS 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMIBLS_OFFSET ));
	pr_info("  CAM_DCS 0x%x\n", BRCM_READ_REG(base,CAM_TOP_REGS_CAMDCS_OFFSET ));
	pr_info("  CAM_TOP_REGS_CAMFIX0_OFFSET 0x%x\n", BRCM_READ_REG(base, CAM_TOP_REGS_CAMFIX0_OFFSET ));

}
void *get_mm_csi0_handle(enum host_mode mode, enum afe_num afe,
			 enum csi2_lanes lanes)
{
	unsigned long flags;

	spin_lock_init(&lock);
	spin_lock_irqsave(&lock, flags);
	if ((cam_state.devbusy == 1) || (cam_state.mode != INVALID)) {
		goto out;
	}
	if (mode != DVP) {
		goto out;
	}
	if ((afe != AFE0) && (afe != AFE1)) {
		goto out;
	}

	cam_state.devbusy = 1;
	cam_state.mode = mode;
	cam_state.afe = afe;

	spin_unlock_irqrestore(&lock, flags);
	return (void *)&cam_state;
out:
	spin_unlock_irqrestore(&lock, flags);
	return NULL;
}

int mm_csi0_init()
{
	u32 base = MM_CFG_BASE;
	u32 ret = 0;
	/* LDO Enable */
	/* point base to mm_cfg base address */
//	ret = (BRCM_READ_REG(base, 0) | (1 << ASIU_TOP_CLK_GATING_CTRL__CAM_CLK_GATE_EN));
//	BRCM_WRITE_REG(base, 0, ret);

	/* To check Enable all memories within mm_csi0*/
	return 0;

}

int mm_csi0_teardown()
{
	u32 base = MM_CFG_BASE;
	unsigned long flags;
	/* LDO Disable */
	/* point base to mm_cfg base address */
	spin_lock_irqsave(&lock, flags);
//	flags = (BRCM_READ_REG(base, 0) & ~(1 << ASIU_TOP_CLK_GATING_CTRL__CAM_CLK_GATE_EN));
//	BRCM_WRITE_REG(base, 0, flags);
	memset(&cam_state, 0x0, sizeof(struct mm_csi0_generic));
	cam_state.devbusy = 0;
	cam_state.mode = INVALID;
	spin_unlock_irqrestore(&lock, flags);
	/* To check Enable all memories within mm_csi0*/
	return 0;
}

int mm_csi0_set_mode(enum csi1ccp2_clock_mode clk_mode)
{
	u32 base = V_BASE;

	/* As per the recommended seq, we set the mode over here */
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__CPI_SELECT, cam_state.mode);

	return 0;
}

int mm_csi0_cfg_pipeline_pack(enum pix_pack_mode pack)
{
	u32 base = V_BASE;

	if ((pack & ~0x7) != 0) {
		return -EINVAL;
	}
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMIPIPE_OFFSET, CAM_TOP_REGS_CAMIPIPE__PPM_R, pack,CAM_TOP_REGS_CAMIPIPE__PPM_WIDTH);
	return 0;
}

int mm_csi0_cfg_pipeline_unpack(enum pix_unpack_mode unpack)
{
	u32 base = V_BASE;

	if ((unpack & ~0x7) != 0) {
		return -EINVAL;
	}
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMIPIPE_OFFSET, CAM_TOP_REGS_CAMIPIPE__PUM_R, unpack,CAM_TOP_REGS_CAMIPIPE__PUM_WIDTH);
	return 0;
}

int mm_csi0_config_int(struct int_desc *desc, enum buffer_type type)
{
	u32 base = V_BASE;

	if (type == IMAGE_BUFFER) {
		if (desc->fsi)
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET , CAM_TOP_REGS_CAMICTL__FEIE, 0x1);
		else
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__FEIE, 0x0);
		if (desc->fei)
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__FEIE, 0x1);
		else
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__FEIE, 0x0);
		if (desc->lci)
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__LCIE_R, desc->lci);
		else
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__LCIE_R, 0);
		/* The CSI2 on UNICAM cannot have EDL=0 due to a limitation
		   Not sure where to address this                        */
	} else if (type == DATA_BUFFER) {
		/* For data buffer there is one interrupt
		   The interrupt can be triggered on an FE or embedded
		   data line end
		   fei is used to say if interrupt is on an FE else interrupt
		   on EDL from desc->dataline
		   For data if FE is not required, pl provide the number of
		   lines of data properly */
		pr_info("Data buffer set\n");
		if (desc->die) {
			/*BRCM_WRITE_REG_FIELD(base, CAM_DCS, DIE, 0x1);
			  if (desc->fei) {
			  BRCM_WRITE_REG_FIELD(base, CAM_DCS, DIM, 0x0);
			  }
			  else {
			  BRCM_WRITE_REG_FIELD(base, CAM_DCS, DIM, 0x0);
			  BRCM_WRITE_REG_FIELD(base, CAM_DCS, EDL,
			  desc->dataline);
			  }*/
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDCS_OFFSET, CAM_TOP_REGS_CAMDCS__EDL_R,
					desc->dataline);
		} else {
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDCS_OFFSET, CAM_TOP_REGS_CAMDCS__DIE, 0x0);
		}
	} else {
		return -EINVAL;
	}
	return 0;
}

int mm_csi0_get_int_stat(struct int_desc *desc, int ack)
{
	u32 base = V_BASE;
	u32 reg;

	if (desc == NULL) {
		return -EINVAL;
	}
	reg = BRCM_READ_REG(base, CAM_TOP_REGS_CAMISTA_OFFSET);
	if (ack)
		BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMISTA_OFFSET, reg);

	return reg;
}

int mm_csi0_get_data_stat(struct int_desc *desc, int ack)
{
	u32 base = V_BASE;

	if (desc == NULL) {
		return -EINVAL;
	}

	return 0;
}

u32 mm_csi0_get_rx_stat(struct rx_stat_list *list, int ack)
{
	u32 base = V_BASE;
	u32 reg;

	if (list == NULL) {
		return 0;
	}
	reg = BRCM_READ_REG(base, CAM_TOP_REGS_CAMSTA_OFFSET);
	if (reg & (1<<CAM_TOP_REGS_CAMSTA__SYN))
		list->syn = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__CS ))
		list->clk_hs_present = 1;



	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__OES))
		list->oes = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__IFO ))
		list->ifo = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__OFO))
		list->ofo = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__BFO))
		list->bfo = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__DL))
		list->dl = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__PS ))
		list->ps = 1;
	if (reg & ( 1<< CAM_TOP_REGS_CAMSTA__IS ))
		list->is = 1;

	return reg;
}

int mm_csi0_buffering_mode(enum buffer_mode bmode)
{
	if (BUFFER_DOUBLE == bmode) {
		cam_state.db_en = 1;
		cam_state.trigger = 0;
	} else {
		cam_state.db_en = 0;
		if (bmode == BUFFER_TRIGGER) {
			cam_state.trigger = 1;
		} else
			cam_state.trigger = 0;
	}
	return 0;
}

int mm_csi0_update_addr(struct buffer_desc *im0, struct buffer_desc *im1,
			struct buffer_desc *dat0, struct buffer_desc *dat1)
{
	u32 base = V_BASE;
	int data = 1;

	if (im0 == NULL) {
		return -EINVAL;
	}
	if (cam_state.db_en && (im1 == NULL)) {
		return -EINVAL;
	}

	if (dat0 == NULL)
		data = 0;
	else {
		if (cam_state.db_en && (dat1 == NULL)) {
			/* Not sure if this is an error or we can ignore data
			   For now disabling data */
			data = 0;
		}
		if (cam_state.db_en && dat1)
			data = 2;
	}
	if (data) {
		if ((dat0->start & 0xF) ||
				((dat0->start + dat0->size) & 0xF) ||
				(dat0->ls & 0xF)) {
			data = 0;
		}
		if ((data == 2) && ((dat1->start & 0xF) ||
					((dat1->start + dat1->size) & 0xF) ||
					(dat1->ls & 0xF))) {
			data = 0;
		}
	}
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 0);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 0);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 0);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 1);

	/* Check the logic above again */
	/* register prog start */
	if (cam_state.db_en)
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 1);
	if (im0) {
		BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBSA0_OFFSET, im0->start);
		BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBEA0_OFFSET, (im0->start + im0->size));
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMIBLS_OFFSET , CAM_TOP_REGS_CAMIBLS__IBLS_R, im0->ls); //cctv
		if (im0->wrap_en == 1)
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__IBOB, 1);
		else
			BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__IBOB, 0);

	}
	if (im1) {
		BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBSA1_OFFSET, im1->start);
		BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBEA1, (im1->start + im1->size));
	}
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__LIP_R, 1,CAM_TOP_REGS_CAMICTL__LIP_WIDTH);
	if (data) {
		if (data == 1) {
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBSA0_OFFSET, dat0->start);
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBEA0_OFFSET, (dat0->start + dat0->size));
		}
		if (data == 2) {
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBSA1_OFFSET, dat1->start);
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBEA1_OFFSET, (dat1->start + dat1->size));
		}
	}
	if (cam_state.db_en) {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 0);
	} else {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 1);
	}
	return 0;
}

int mm_csi0_update_one(struct buffer_desc *im, int buf_num,
		enum buffer_type type)
{
	u32 base = V_BASE;

	if (im == NULL) {
		return -EINVAL;
	}
	if ((im->start & 0xF) || ((im->start + im->size) & 0xF) ||
			(im->ls & 0xF)) {
		return -EINVAL;
	}
	if (type == DATA_BUFFER)  {
		/* Data buffer */
		if (buf_num == 0) {
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBSA0_OFFSET, im->start);
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBEA0_OFFSET, (im->start + im->size));
		} else {
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBSA1_OFFSET, im->start);
			BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMDBEA1_OFFSET, (im->start + im->size));
		}
	}
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__LIP_R, 1);
	if (cam_state.db_en) {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 0);
	} else {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 0);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET, CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 1);
	}

	return 0;
}

int mm_csi0_trigger_cap(void)
{
	u32 base = V_BASE;

	if (cam_state.trigger) {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__TFC, 0x1);
		return 0;
	} else {
		return -EINVAL;
	}
}

int mm_csi0_rx_burst()
{
	u32 base = V_BASE;
	u32 val_sw;

	/* For QoS enable */
	/* Enabling panic enable */
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMPRI_OFFSET, 0);
	return 0;

	return 0;
}

static int enable_done;

int mm_csi0_enable_unicam(void)
{
	u32 base = V_BASE;
	u32 val = 0;

	if (enable_done) {
		return 0;
	}

	val = BRCM_READ_REG(base, CAM_TOP_REGS_CAMISTA_OFFSET);
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMISTA_OFFSET,val );
	val = BRCM_READ_REG(base, CAM_TOP_REGS_CAMSTA_OFFSET);
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMSTA_OFFSET, val);

	if (cam_state.trigger)
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__FCM, 0x1);
	else
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMICTL_OFFSET, CAM_TOP_REGS_CAMICTL__FCM, 0x0);

	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__CPE, 1);

	enable_done = 1;

	return 0;
}

static int rx_init_done;

int mm_csi0_start_rx(void)
{
	u32 base = V_BASE;

	if (rx_init_done) {
		return 0;
	}

	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIS_OFFSET, CAM_TOP_REGS_CAMCPIS__ENB, 1);
	udelay(1);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIS_OFFSET, CAM_TOP_REGS_CAMCPIS__ACT, 1);
	if (cam_state.db_en) {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__DB_EN, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF0_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMDBCTL_OFFSET, CAM_TOP_REGS_CAMDBCTL__BUF1_IE, 1);
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMFIX0_OFFSET,  CAM_TOP_REGS_CAMFIX0__DIS_DB_IE, 0);
	}
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIR_OFFSET, CAM_TOP_REGS_CAMCPIR__VSAL, 1);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIR_OFFSET, CAM_TOP_REGS_CAMCPIR__HSAL, 1);
	
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIF_OFFSET, CAM_TOP_REGS_CAMCPIF__SMODE, 0);
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMCPIF_OFFSET, CAM_TOP_REGS_CAMCPIF__VMODE_R, 0,CAM_TOP_REGS_CAMCPIF__VMODE_WIDTH);
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMCPIF_OFFSET, CAM_TOP_REGS_CAMCPIF__HMODE_R, 0,CAM_TOP_REGS_CAMCPIF__HMODE_WIDTH);
	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMCPIF_OFFSET, CAM_TOP_REGS_CAMCPIF__FMODE_R, 0,CAM_TOP_REGS_CAMCPIF__FMODE_WIDTH);

	BRCM_WRITE_REG_FIELD_MUL(base, CAM_TOP_REGS_CAMCPIW_OFFSET, CAM_TOP_REGS_CAMCPIW__FGATE_R, 0,CAM_TOP_REGS_CAMCPIW__FGATE_WIDTH);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIW_OFFSET, CAM_TOP_REGS_CAMCPIW__ENB, 1);

	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMCPIWVC_OFFSET, 0xf100f1);
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMCPIWHC_OFFSET, 0);

	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMTGBL_OFFSET, 0);
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMCPIWHC_OFFSET, 0);
	
	udelay(5);
	rx_init_done = 1;

	return 0;
}

int mm_csi0_stop_rx(void)
{
	u32 base = V_BASE;
	u32 val;
	unsigned int i;

	enable_done = 0;
	rx_init_done = 0;
	
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIS_OFFSET, CAM_TOP_REGS_CAMCPIS__ENB, 0);
	udelay(1);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCPIS_OFFSET, CAM_TOP_REGS_CAMCPIS__ACT, 0);

	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__SOE, 1);

	udelay(10);

	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__SOE, 0);

	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__CPR, 1);
	udelay(10);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__CPR, 0);

	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBSA0_OFFSET, 0x0);
	BRCM_WRITE_REG(base, CAM_TOP_REGS_CAMIBEA0_OFFSET, 0x0);
	BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMCTL_OFFSET, CAM_TOP_REGS_CAMCTL__CPE, 0);

	return 0;
}

/* STA register bit PS will be high to denote a panic was signalled */
bool mm_csi0_get_panic_state()
{
	u32 base = V_BASE;

	if (BRCM_READ_REG_FIELD(base, CAM_TOP_REGS_CAMSTA_OFFSET, CAM_TOP_REGS_CAMSTA__PS)) {
		BRCM_WRITE_REG_FIELD(base, CAM_TOP_REGS_CAMSTA_OFFSET, CAM_TOP_REGS_CAMSTA__PS, 1);
		return 1;
	}
	return 0;
}

char const unicam_camera_name[] = "iproc-camera";

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff
/* int only_once = 0; */

struct unicam_camera_dev {
	/* soc and vb3 rleated */
	struct soc_camera_device *icd;
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	struct vb2_alloc_ctx *alloc_ctx;
#endif
	struct soc_camera_host soc_host;
	/* generic driver related */
	unsigned int irq;
	struct device *dev;
	/* h/w specific */
	void __iomem *csi_base;
	void __iomem *mm_cfg_base;
	void __iomem *mm_clk_base;

	void *handle;
	/* data structure needed to support streaming */
	int sequence;
	spinlock_t lock;
	struct vb2_buffer *active;
	struct list_head capture;
	struct vb2_buffer *buff[2];
	bool curr;
	atomic_t streaming;
	u32 skip_frames;
	struct semaphore stop_sem;
	atomic_t stopping;
	enum buffer_mode b_mode;
	struct v4l2_crop crop;
	int cap_mode;
	int cap_done;
	u32 panic_count;
	atomic_t cam_triggered;
	struct v4l2_format active_fmt;
	int frame_info_en;
	struct v4l2_frame_info frame_info;
};

struct unicam_camera_buffer {
	struct vb2_buffer vb;
	struct list_head queue;
	unsigned int magic;
};
int first = 0;

static irqreturn_t unicam_camera_isr(int irq, void *arg);
static irqreturn_t unicam_camera_isr_bh(int irq, void *arg);
static int unicam_stop(void);

#ifdef UNICAM_DEBUG
/* for debugging purpose */
static void dump_file(char *filename, void *src, int size)
{
	mm_segment_t old_fs;
	int fd = 0, nbytes = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	pr_err("Writing %d bytes to file %s\n", size, filename);
	fd = sys_open(filename, (O_RDWR | O_CREAT), 0777);
	if (fd < 0) {
		pr_err("Error in opening file %s\n", filename);
		goto out;
	}

	nbytes = sys_write(fd, src, size);
	pr_err("Wrote %d bytes to file %s\n", nbytes, filename);
	sys_close(fd);

out:
	set_fs(old_fs);
}
#endif

static struct unicam_camera_buffer *to_unicam_camera_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct unicam_camera_buffer, vb);
}


/* videobuf operations */
static int unicam_videobuf_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *count, unsigned int *numplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev =
	    (struct unicam_camera_dev *)ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;
	*numplanes = 1;
	unicam_dev->sequence = 0;

	sizes[0] = bytes_per_line * icd->user_height;
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	alloc_ctxs[0] = unicam_dev->alloc_ctx;
#endif

	if (!*count)
		*count = 2;
	return 0;
}

static int unicam_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);
	unsigned long size;

	if (bytes_per_line < 0)
		return bytes_per_line;

	pr_debug("vb=0x%p buf=0x%p, size=%lu", vb,
			(void *)vb2_plane_dma_addr(vb, 0),
			vb2_get_plane_payload(vb, 0));

	size = icd->user_height * bytes_per_line;

	if (vb2_plane_size(vb, 0) < size) {
		return -ENOBUFS;
	}
	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

/* should be called with unicam_dev->lock held */
static int unicam_camera_update_buf(struct unicam_camera_dev *unicam_dev)
{

	struct v4l2_subdev *sd = soc_camera_to_subdev(unicam_dev->icd);
	struct buffer_desc im0, dat0;
	dma_addr_t phys_addr;
	unsigned int line_stride;
	int thumb = 0, ret;

	if (!unicam_dev->active) {
		return -ENOMEM;
	}
	phys_addr = vb2_plane_dma_addr(unicam_dev->active, 0);
	if (!phys_addr) {
		unicam_dev->active = NULL;
		return -ENOMEM;
	}

	/* stride is in bytes */
	if (unicam_dev->icd->current_fmt->code != V4L2_MBUS_FMT_JPEG_1X8) {
		
		if ((unicam_dev->crop.c.top == 0) ||
			(unicam_dev->crop.c.left == 0)) {	   
			line_stride =
			    soc_mbus_bytes_per_line(unicam_dev->icd->user_width,
						    unicam_dev->icd->
						    current_fmt->host_fmt);
		} else {
			line_stride =
			    soc_mbus_bytes_per_line(unicam_dev->crop.c.width,
						    unicam_dev->icd->
						    current_fmt->host_fmt);
		}
		im0.start = (UInt32) phys_addr;
		im0.ls = (UInt32) line_stride;
		im0.size = line_stride * unicam_dev->icd->user_height;
		im0.wrap_en = 1;

		/* Coverity Fix: Dead Code */
		/* if(unicam_dev->b_mode == BUFFER_DOUBLE && phys_addr1){ */
			/* image 1 */
		/*	im1.start = phys_addr1;
			im1.ls = im0.ls;
			im1.size = im0.size;
			mm_csi0_update_addr(&im0, &im1, NULL, NULL);
		} else { */
		mm_csi0_update_addr(&im0, NULL, NULL, NULL);
		} 
	return 0;
}

/* should be called with unicam_dev->lock held */
static int unicam_camera_capture(struct unicam_camera_dev *unicam_dev)
{
	int ret = 0;
	struct int_desc i_desc;

	if (!unicam_dev->active) {
		return ret;
	}
	if (unicam_dev->b_mode != BUFFER_TRIGGER)
		return 0;

	/* enable frame Interrupts */
	memset(&i_desc, 0x0, sizeof(struct int_desc));
	i_desc.fei = 1;
	i_desc.fsi = 1;
	ret = mm_csi0_trigger_cap();
/*	if (ret == 0) {
		atomic_set(&unicam_dev->cam_triggered, 1);
		if (atomic_read(&unicam_dev->streaming) == 1)
			mod_timer(&unicam_dev->unicam_timer, \
				jiffies + msecs_to_jiffies(UC_TIMEOUT_MS));
	}
*/
	return ret;
}

static void unicam_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vb);
	unsigned long flags;
	struct int_desc idesc;

	pr_debug("vb=0x%p pbuf=0x%p size=%lu  \n", vb,
			(void *)vb2_plane_dma_addr(vb, 0),
			vb2_get_plane_payload(vb, 0));
	/* pr_info("Q 0x%x\n", vb2_plane_paddr(vb, 0)); */
	spin_lock_irqsave(&unicam_dev->lock, flags);
	list_add_tail(&buf->queue, &unicam_dev->capture);
	if (unicam_dev->cap_mode && unicam_dev->cap_done) {
		spin_unlock_irqrestore(&unicam_dev->lock, flags);
		return;
	}
	if ((!unicam_dev->active)) {
		unicam_dev->active = vb;
		unicam_camera_update_buf(unicam_dev);
		if (atomic_read(&unicam_dev->streaming)) {
			mm_csi0_start_rx();
			/* set data capture */
			{
				idesc.fsi = 0;
				idesc.fei = 0;
				idesc.lci = unicam_dev->icd->user_height;
				idesc.die = 0;
				idesc.dataline = 0;
//				mm_csi0_config_int(&idesc, DATA_BUFFER);
			}
		}
	}
	spin_unlock_irqrestore(&unicam_dev->lock, flags);
}

static void unicam_videobuf_release(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vb);
	unsigned long flags;

	pr_debug("vb=0x%p pbuf=0x%p size=%lu  \n", vb,
			(void *)vb2_plane_dma_addr(vb, 0),
			vb2_get_plane_payload(vb, 0));
	spin_lock_irqsave(&unicam_dev->lock, flags);

	if (buf->magic == UNICAM_BUF_MAGIC)
		list_del_init(&buf->queue);
	spin_unlock_irqrestore(&unicam_dev->lock, flags);

}

static int unicam_videobuf_init(struct vb2_buffer *vb)
{
	struct unicam_camera_buffer *buf = to_unicam_camera_vb(vb);

	INIT_LIST_HEAD(&buf->queue);
	buf->magic = UNICAM_BUF_MAGIC;
	return 0;
}

static void unicam_camera_get_frame_info_int(struct unicam_camera_dev *ucdev)
{
	int ret;
	struct soc_camera_device *icd = ucdev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	if (!ucdev->frame_info_en)
		return;
	ucdev->frame_info.timestamp = (struct timespec){-1, -1};
	ret = v4l2_subdev_call(sd, core, ioctl, VIDIOC_SENSOR_G_FRAME_INFO,
			&ucdev->frame_info);
	if (ret < 0)
		ucdev->frame_info_en = 0;
}

static void unicam_camera_set_frame_info_int(struct unicam_camera_dev *ucdev)
{
	int ret;
	struct soc_camera_device *icd = ucdev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct vb2_buffer *vb = ucdev->active;
	struct timeval *tv = &vb->v4l2_buf.timestamp;

	if (!ucdev->frame_info_en)
		return;
	ucdev->frame_info.timestamp =
		(struct timespec){tv->tv_sec, tv->tv_usec * 1000};
	ret = v4l2_subdev_call(sd, core, ioctl, VIDIOC_SENSOR_S_FRAME_INFO,
			       &ucdev->frame_info);
	if (ret < 0)
		ucdev->frame_info_en = 0;
}

static int unicam_videobuf_start_streaming_int(struct unicam_camera_dev *unicam_dev, unsigned int count)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct rx_stat_list rx;
	int ret;
	int lane_err;
	int thumb;
	u32 raw_rx;

	enum afe_num afe;
	enum host_mode hmode;
	int vc = 0;
	int id = 0;
	struct int_desc idesc;
	struct lane_timing timing;
	uint32_t data1;
	uint32_t wr_data;
	uint32_t rd_data;
	int test = 0;
	int speed_mode = 0;
	int read_value;
	uint32_t error_value;
	
	uint32_t reg_value;
	
	read_value = 0;
	error_value = 0;

	unicam_dev->panic_count = 0;
	atomic_set(&unicam_dev->cam_triggered, 0);


	hmode = DVP;
	unicam_dev->b_mode = BUFFER_TRIGGER;

	unicam_dev->handle = get_mm_csi0_handle(hmode, afe, 0);
	
	if (unicam_dev->handle == NULL) {
		return -EBUSY;
	}
	ret = mm_csi0_init();
	
	if (ret) {
		mm_csi0_teardown();
		return -EINVAL;
	}
	/* Digital PHY Setup */
	/* Compulsary to get these values from sensor for CSI2 */
	/* Don't care for CCP2/CSI1 can send a struct with junk values
	   Will not be read 
	timing.hs_settle_time =
		unicam_dev->if_params.parms.serial.hs_settle_time;
	timing.hs_term_time = unicam_dev->if_params.parms.serial.hs_term_time;
	pr_debug("HS: settle_t = %d, term_t = %d\n",
			timing.hs_settle_time, timing.hs_term_time);*/

	/* Set Mode */
	mm_csi0_set_mode(0);

	/* check if frame_info is supported */
	unicam_dev->frame_info_en = 1;
	/*unicam_camera_get_frame_info_int(unicam_dev);*/

	/* set image identifier (CSI mode only) */

	/* if thumbnail is supported we expect
	 * thumbnail to be in image ptr format of thumbnails is yuv422
	 * format is checked in try format.
	 * in case where thumbnail is not supported we get jpeg
	 * image in data pointer. so we set the id as 0
	 */

	thumb = 0;
	ret = v4l2_subdev_call(sd, core, ioctl, VIDIOC_THUMB_SUPPORTED,
				(void *)&thumb);
	if (ret < 0)
		dev_warn(unicam_dev->dev,
			 "sensor returns error(%d) for VIDIOC_THUMB_SUPPORTED\n",
			 ret);

	if ((icd->current_fmt->code == V4L2_MBUS_FMT_JPEG_1X8) && (thumb == 0))
		id = 0;
	/* thumbnail not supported */
	/* RAW10 */
	else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10))
		id = 0x2b;
	/* RAW8 */
	else if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG8_1X8)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB8_1X8))
		id = 0x2a;
	/* YUV422 */
	else
		id = 0x1e;

	if (icd->current_fmt->code == V4L2_MBUS_FMT_JPEG_1X8)
		pr_info("JPEG mode of capture !!!!\n");

	ret = 0;

	/* UNPACK */
	if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
#ifdef UNPACK_RAW10_TO_16BITS
		ret |= mm_csi0_cfg_pipeline_unpack(PIX_UNPACK_RAW8);
#endif
	} else {
		ret |= mm_csi0_cfg_pipeline_unpack(PIX_UNPACK_NONE);
	}

	/* PACK */
	if ((icd->current_fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10)
		  || (icd->current_fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
#ifdef UNPACK_RAW10_TO_16BITS
		ret |= mm_csi0_cfg_pipeline_pack(PIX_UNPACK_RAW8);
#endif
	} else {
		ret |= mm_csi0_cfg_pipeline_pack(PIX_PACK_NONE);
	}

	/* Output engine */
	mm_csi0_buffering_mode(unicam_dev->b_mode);
//	mm_csi0_enable_unicam();
    asiu_cam_init_seq();
	
	if (unicam_dev->active) {
	 atomic_set(&unicam_dev->streaming, 1);
	 }
    mdelay(100);
printk("00000000000000000 haha %s open interrupt \n", __func__);
		   //cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xffffffe0) | 0x0000001f) , 4 );
			cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xfffffff0) | 0x0000000f) , 4 ); //Vsync is active low and hsync is active low
		  
		   //Enable FRAME Start and End Interrupts
			 cpu_wr_single (CAM_TOP_REGS_CAMICTL, ((cpu_rd_single(CAM_TOP_REGS_CAMICTL,4) & 0xffffffC4) | 0x0000003B) , 4 );
		  
			 //Enalbe Interrupt -- Reg. config. during CPI Capture
			 cpu_wr_single (CAM_TOP_REGS_CAMCPIS, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIS,4) & 0x7ffffffc) | 0x80008003) , 4 );
	 

	ret = v4l2_subdev_call(sd, video, s_stream, 1);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		return ret;
	}

/*	if (unicam_dev->active)
		if (unicam_dev->if_params.if_mode == \
				V4L2_SUBDEV_SENSOR_MODE_SERIAL_CSI2)
			mod_timer(&unicam_dev->unicam_timer, \
				jiffies + msecs_to_jiffies(UC_TIMEOUT_MS));
*/
	return 0;
}

int unicam_videobuf_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd ;//= soc_camera_from_vb2q(q);
	struct soc_camera_host *ici ;//= to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev ;//= ici->priv;
	int ret = 0;
	icd = soc_camera_from_vb2q(q);
	
	ici = to_soc_camera_host(icd->parent);
	
	unicam_dev = ici->priv;

	/*atomic_set(&unicam_dev->retry_count, 0);*/
	if (!atomic_read(&unicam_dev->streaming)){
				ret = unicam_videobuf_start_streaming_int(unicam_dev, count);
		}
	else
		pr_err("unicam_videobuf_start_streaming: already started\n");
 	cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xfffffff0) | 0x0000000f) , 4 );  //cctv
    cpu_wr_single (CAM_TOP_REGS_CAMICTL, ((cpu_rd_single(CAM_TOP_REGS_CAMICTL,4) & 0xffffffC4) | 0x0000003B) , 4 ); //cctv
 	cpu_wr_single (CAM_TOP_REGS_CAMCPIS, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIS,4) & 0x7ffffffc) | 0x80008003) , 4 ); //cctv

	return 0;//ret;
}

static int unicam_videobuf_stop_streaming_int(struct unicam_camera_dev *unicam_dev)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int ret = 0;
	unsigned long flags;
	struct rx_stat_list rx;

	/* grab the lock */
	spin_lock_irqsave(&unicam_dev->lock, flags);
	if (!atomic_read(&unicam_dev->streaming)) {
		pr_err("stream already turned off\n");
		goto out;
	}
	if (unicam_dev->active) {
		atomic_set(&unicam_dev->stopping, 1);
		spin_unlock_irqrestore(&unicam_dev->lock, flags);
		ret = down_timeout(&unicam_dev->stop_sem,
				msecs_to_jiffies(500));
		atomic_set(&unicam_dev->stopping, 0);
		if (ret == -ETIME) {
			pr_err("Unicam: semaphore timed out waiting to STOP\n");
		}
	} else {
		spin_unlock_irqrestore(&unicam_dev->lock, flags);
	}
	usleep_range(50, 60); /*TODO: Need to double-check with ASIC team*/
	spin_lock_irqsave(&unicam_dev->lock, flags);

	unicam_stop();

	/* Restart rx stat */
	mm_csi0_get_rx_stat(&rx, 1);
	/* Don't bother what values were returned */
	mm_csi0_teardown();
	unicam_dev->active = NULL;
	atomic_set(&unicam_dev->streaming, 0);
	memset(&unicam_dev->crop, 0x00, sizeof(struct v4l2_crop));
	unicam_dev->cap_done = 0;
	unicam_dev->cap_mode = 0;

out:
	atomic_set(&unicam_dev->cam_triggered, 0);
	spin_unlock_irqrestore(&unicam_dev->lock, flags);

	/* stop sensor streaming after UNICAM is disabled */
	ret = v4l2_subdev_call(sd, video, s_stream, 0);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		pr_err("failed to stop sensor streaming\n");
		ret = -1;
	}

	return ret;
}

int unicam_videobuf_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = soc_camera_from_vb2q(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	int ret = 0;

	if (atomic_read(&unicam_dev->streaming))
		ret = unicam_videobuf_stop_streaming_int(unicam_dev);
	else
		pr_err("unicam_videobuf_start_streaming: already stopped\n");

/*	atomic_set(&unicam_dev->retry_count, 0);
	del_timer_sync(&(unicam_dev->unicam_timer));
	flush_work_sync(&unicam_dev->retry_work); */

	return ret;
}

static struct vb2_ops unicam_videobuf_ops = {
	.queue_setup = unicam_videobuf_setup,
	.buf_prepare = unicam_videobuf_prepare,
	.buf_queue = unicam_videobuf_queue,
	.buf_cleanup = unicam_videobuf_release,
	.buf_init = unicam_videobuf_init,
	.start_streaming = unicam_videobuf_start_streaming,
	.stop_streaming = unicam_videobuf_stop_streaming,
	.wait_prepare = soc_camera_unlock,
	.wait_finish = soc_camera_lock
};

static int unicam_camera_init_videobuf(struct vb2_queue *q,struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv = icd;
	q->ops = &unicam_videobuf_ops;

#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	q->mem_ops = &vb2_dma_contig_memops;
#endif
	q->buf_struct_size = sizeof(struct unicam_camera_buffer);
	return vb2_queue_init(q);
}

static int unicam_camera_set_bus_param(struct soc_camera_device *icd,
				       __u32 pixfmt)
{
	return 0;
}

static int unicam_camera_querycap(struct soc_camera_host *ici,
				  struct v4l2_capability *cap)
{
	/* cap->name is set by the firendly caller:-> */
	strlcpy(cap->card, "Unicam Camera", sizeof(cap->card));
	cap->version = KERNEL_VERSION(0, 1, 0);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static unsigned int unicam_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int unicam_camera_try_fmt(struct soc_camera_device *icd,
				 struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	struct v4l2_format thumb_fmt;
	struct v4l2_pix_format *thumb_pix;
	__u32 pixfmt = pix->pixelformat;
	int thumb = 0;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->parent, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}

	pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field = V4L2_FIELD_NONE;
		break;
	default:
		dev_err(icd->parent, "Field type %d unsupported.\n",
			mf.field);
		return -EINVAL;
	}

	/* what format can unicam support */
	switch (mf.code) {
	case V4L2_MBUS_FMT_JPEG_1X8:
		/* check here if thumbnail is supported
		and check thumbnail format */
		ret =
		    v4l2_subdev_call(sd, core, ioctl, VIDIOC_THUMB_SUPPORTED,
				     (void *)&thumb);
		if ((!ret) && thumb) {
			ret =
			    v4l2_subdev_call(sd, core, ioctl,
					     VIDIOC_THUMB_G_FMT,
					     (void *)&thumb_fmt);
			if (ret < 0) {
				dev_err(icd->parent,
					"sensor driver should report thumbnail format\n");
				return -EINVAL;
			}
			thumb_pix = &thumb_fmt.fmt.pix;
			switch (thumb_pix->pixelformat) {
			case V4L2_PIX_FMT_YUYV:
			case V4L2_PIX_FMT_UYVY:
				pr_debug
				    ("sensor supports thumbnail %c%c%c%c format  \n",
				     pixfmtstr(thumb_pix->pixelformat));
				break;
			default:
				dev_err(icd->parent,
					"sensor thumbnail format %c%c%c%c not supported\n",
					pixfmtstr(thumb_pix->pixelformat));
				return -EINVAL;
			}
		} else
			pr_debug
			    ("sensor doesnot support thumbnail (thumb=%d, ret=%d)\n",
			     thumb, ret);

	case V4L2_MBUS_FMT_YUYV8_2X8:
	case V4L2_MBUS_FMT_UYVY8_2X8:
		/* Above formats are supported */
		break;
	case V4L2_MBUS_FMT_SBGGR10_1X10:
	case V4L2_MBUS_FMT_SGBRG10_1X10:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	case V4L2_MBUS_FMT_SRGGB10_1X10:
		/* ISP needs 32 byte input boundary on line stride */
		pix->bytesperline = ((((pix->width * 10) >> 3) + 31) & ~(31));
#ifdef UNPACK_RAW10_TO_16BITS
		pix->bytesperline = ((pix->width * 2) + 31) & ~(31);
#endif
		break;
		/* ISP needs 32 byte input boundary on line stride */
	case V4L2_MBUS_FMT_SBGGR8_1X8:
	case V4L2_MBUS_FMT_SGBRG8_1X8:
	case V4L2_MBUS_FMT_SGRBG8_1X8:
	case V4L2_MBUS_FMT_SRGGB8_1X8:
		pix->bytesperline = ((pix->width + 31) & ~(31));
		break;
	default:
		dev_err(icd->parent, "Sensor format code %d unsupported.\n",
			mf.code);
		return -EINVAL;
	}
	return ret;
}

static int unicam_stop()
{
	struct int_desc idesc;

	mm_csi0_stop_rx();
	memset(&idesc, 0x00, sizeof(struct int_desc));
	mm_csi0_config_int(&idesc, IMAGE_BUFFER);   
	mm_csi0_config_int(&idesc, DATA_BUFFER); 
	return 0;
}

static int unicam_camera_s_ctrl(struct soc_camera_device *icd,
				 struct v4l2_control *ctl)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	int ret = 0;

	if (ctl == NULL) {
		pr_err("Wrong host ops s_ctrl\n");
		return -EINVAL;
	}
	switch (ctl->id) {
	case V4L2_CID_CAM_CAPTURE:
		pr_info("V4L2_CID_CAM_CAPTURE\n");
		unicam_dev->cap_mode = 1;
		unicam_dev->cap_done = 0;

		/*for camera driver also invoke s_ctrl */
		ret = -ENOIOCTLCMD;
		break;
	case V4L2_CID_CAM_CAPTURE_DONE:
		pr_info("V4L2_CID_CAM_CAPTURE_DONE\n");
		unicam_dev->cap_mode = 0;

		/*for camera driver also invoke s_ctrl */
		ret = -ENOIOCTLCMD;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

/* This method shall be used only for unicam windowing
   for zoom use-case */
static int unicam_camera_get_crop(struct soc_camera_device *icd,
				 struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;

	if (crop != NULL)
		*crop = unicam_dev->crop;

	return 0;
}
static int unicam_camera_set_crop(struct soc_camera_device *icd,
				struct v4l2_crop *crop)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;

	if (crop == NULL)
		return -EINVAL;
	unicam_dev->crop = *crop;
	return 0;
}
static int unicam_camera_set_fmt_int(struct unicam_camera_dev *unicam_dev)
{
	struct soc_camera_device *icd = unicam_dev->icd;
	struct device *dev = icd->parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_format *f = &(unicam_dev->active_fmt);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	int ret;
	u32 skip_frames = 0;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pix->pixelformat);
		return -EINVAL;
	}

	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (mf.code != xlate->code)
		return -EINVAL;
	if (ret < 0) {
		return ret;
	}
	/*TODO limit here any maximum size */
	ret = v4l2_subdev_call(sd, sensor, g_skip_frames, &skip_frames);
	if (ret < 0) {
		skip_frames = 0;
		ret = 0;
	}
	unicam_dev->skip_frames = skip_frames;
	unicam_dev->curr = 0;
	first = 2;
	unicam_dev->buff[0] = NULL;
	unicam_dev->buff[1] = NULL;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->field = mf.field;
	pix->colorspace = mf.colorspace;
	icd->current_fmt = xlate;

	/* Initialize crop window for now */
	unicam_dev->crop.c.width = pix->width;
	unicam_dev->crop.c.height = pix->height;
	unicam_dev->crop.c.top = unicam_dev->crop.c.left = 0;
	return ret;
}

static int unicam_camera_set_fmt(struct soc_camera_device *icd,
				 struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	int ret;

	unicam_dev->active_fmt = *f;
	ret = unicam_camera_set_fmt_int(unicam_dev);
	return ret;
}

static int unicam_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	int err = 0;

	if (unicam_dev->icd) {
		dev_warn(icd->parent,
			 "Unicam camera driver already attached to another client\n");
		err = -EBUSY;
		goto eicd;
	}

	/* register irq */
	err =
	    request_threaded_irq(unicam_dev->irq, unicam_camera_isr,
			unicam_camera_isr_bh,
			IRQF_DISABLED | IRQF_SHARED, UNICAM_CAM_DRV_NAME,
			unicam_dev);
	if (err) {
		dev_err(icd->parent, "cound not install irq %d\n",
			unicam_dev->irq);
		err = -ENODEV;
		goto eirq;
	}
	err = v4l2_subdev_call(sd, core, s_power, 1);
	if (err < 0 && err != -ENOIOCTLCMD && err != -ENODEV) {
		dev_err(icd->parent, "cound not power up subdevice\n");
		return err;
	} else {
		err = 0;
	}
	unicam_dev->icd = icd;
eirq:
eicd:
	return err;
}

static void unicam_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct unicam_camera_dev *unicam_dev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	BUG_ON(icd != unicam_dev->icd);
	if (atomic_read(&unicam_dev->streaming)) {
		/* stop streaming */
		/* we should call streamoff from queue operations */
		unicam_videobuf_stop_streaming(&icd->vb2_vidq);
	}
	v4l2_subdev_call(sd, core, s_power, 0);
	free_irq(unicam_dev->irq, unicam_dev);
	unicam_dev->icd = NULL;
}

static struct soc_camera_host_ops unicam_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = unicam_camera_add_device,
	.remove = unicam_camera_remove_device,
	.set_fmt = unicam_camera_set_fmt,
	.try_fmt = unicam_camera_try_fmt,
	.init_videobuf2 = unicam_camera_init_videobuf,
	.poll = unicam_camera_poll,
	.querycap = unicam_camera_querycap,
	.set_bus_param = unicam_camera_set_bus_param,
	.set_livecrop = unicam_camera_set_crop,
	.set_crop = unicam_camera_set_crop,
	.get_crop = unicam_camera_get_crop,
};

static irqreturn_t unicam_camera_isr(int irq, void *arg)
{
	struct unicam_camera_dev *unicam_dev = (struct unicam_camera_dev *)arg;
	struct v4l2_subdev *sd = soc_camera_to_subdev(unicam_dev->icd);
	int ret;
	struct int_desc idesc;
	struct rx_stat_list rx;
	u32 isr_status, raw_stat;
	struct buffer_desc im0;
	dma_addr_t dma_addr;
	unsigned long flags;

	isr_status = mm_csi0_get_int_stat(&idesc, 1);

	/* has the interrupt occured for Channel 0? */
	memset(&rx, 0, sizeof(struct rx_stat_list));
	memset(&idesc, 0, sizeof(idesc));
	raw_stat = mm_csi0_get_rx_stat(&rx, 1);
	if (atomic_read(&unicam_dev->streaming) == 0) {
		isr_status = mm_csi0_get_int_stat(&idesc, 1);
		goto out;
	} else if (isr_status&0x2){
			{
			struct vb2_buffer *vb = unicam_dev->active;
			/* FS and FE handling */
			if (rx.ps)
			atomic_set(&unicam_dev->cam_triggered, 0);
			if (!vb)
				goto out;

			if (likely(unicam_dev->skip_frames <= 0)) {
				struct v4l2_control ctrl;
				int ret = -1;
				ctrl.value = 0;
				ctrl.id = V4L2_CID_CAMERA_READ_MODE_CHANGE_REG;
				ret = v4l2_subdev_call(sd, core, g_ctrl, &ctrl);

				if ((ret >= 0) && (ctrl.value > 0)) {
					/* capture mode is not ready yet */
					unicam_dev->skip_frames = ctrl.value;
				}
			}
			if (likely(unicam_dev->skip_frames <= 0)) {
				spin_lock_irqsave(&unicam_dev->lock, flags);
				list_del_init(&to_unicam_camera_vb(vb)->queue);
				spin_unlock_irqrestore(&unicam_dev->lock,
								flags);
				do_gettimeofday(&vb->v4l2_buf.timestamp);
				vb->v4l2_planes[0].bytesused = 0;

				if (unicam_dev->icd->current_fmt->code ==
				    V4L2_MBUS_FMT_JPEG_1X8) {
				} else {
					ret = 1;
				}
				unicam_camera_set_frame_info_int(unicam_dev);
				vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
				spin_lock_irqsave(&unicam_dev->lock, flags);
				if (atomic_read(&unicam_dev->stopping) == 1) {
					up(&unicam_dev->stop_sem);
					unicam_dev->active = NULL;
				} else if (!list_empty(&unicam_dev->capture)) {
					unicam_dev->active =
					    &list_entry(unicam_dev->capture.next, struct
							unicam_camera_buffer,
							queue)->vb;
				} else {
					unicam_dev->active = NULL;
				}
				spin_unlock_irqrestore(&unicam_dev->lock,
					flags);
				if (unicam_dev->cap_mode == 1) {
					unicam_dev->cap_done = 1;
					goto out;
				}
			} else {
				unicam_dev->skip_frames--;
			}

			if (unicam_dev->active) {
				dma_addr = vb2_plane_dma_addr(
					unicam_dev->active, 0);
				if (!dma_addr) {
					unicam_dev->active = NULL;
					goto out;
				}
				im0.start = dma_addr;
				im0.size = unicam_dev->icd->user_width *
					unicam_dev->icd->user_height * 2;
				im0.ls = unicam_dev->icd->user_width * 2;
				if (unicam_dev->icd->current_fmt->code !=
					V4L2_MBUS_FMT_JPEG_1X8) {

				} else {
					mm_csi0_update_one(&im0,
						unicam_dev->curr, DATA_BUFFER);
				}
				unicam_camera_capture(unicam_dev);
			} 
		}

	}
out:
	return (idesc.fsi && unicam_dev->skip_frames <= 0) ?
		IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t unicam_camera_isr_bh(int irq, void *arg)
{
	struct unicam_camera_dev *unicam_dev = (struct unicam_camera_dev *)arg;
	unicam_camera_get_frame_info_int(unicam_dev);
	return IRQ_HANDLED;
}

/*---------------------------  Gen PLL Configuration function -----------------------------------*/
 void asiu_audio_gen_pll_pwr_on (uint32_t crmu_pll_pwr_on)
 {
	void __iomem *reg_addr=0;
	unsigned int reg_val = 0;

 //uint32_t wr_data = 0;
 // uint32_t rd_data = 0;
 // uint32_t mask = 0;
 reg_addr = ioremap_nocache(CRMU_PLL_AON_CTRL,0x4);
 __raw_writel( ((__raw_readl(reg_addr) & 0xfffffbff) | 0x00000400), (reg_addr));
 msleep(1);
 __raw_writel( ((__raw_readl(reg_addr) & 0xfffff5ff) | 0x00000a00), (reg_addr));
 msleep(1);
 __raw_writel( ((__raw_readl(reg_addr) & 0xfffffeff)), (reg_addr));
 iounmap(reg_addr);
 msleep(1);


 }


/*---------------------------  Gen PLL Configuration function -----------------------------------*/
 void asiu_audio_gen_pll_group_id_config (uint32_t pdiv, uint32_t ndiv_int, uint32_t ndiv_frac, uint32_t mdiv, uint32_t gain_ki, uint32_t gain_ka, uint32_t gain_kp, uint32_t user_macro, uint32_t i2s_port_num, uint32_t audio_ext_test_clock_en  )
{

	 int test = 0;
	 int read_value;
	 uint32_t error_value;
	 
	 uint32_t reg_value;
	 
	 read_value = 0;
	 error_value = 0;
	 uint32_t wr_data_user =0;
	 uint32_t wr_data_clk_gate =0;
	 uint32_t wr_data =0;
	 uint32_t reset_val = 0;
	 uint32_t rd_data = 0;
	 uint32_t timeout = 120;
	 uint32_t bypass_disable = 0;
	 uint32_t mask = 0;
	 uint32_t data,slave_address1;
	 void __iomem *reg_addr=0;
	 unsigned int reg_val = 0;

	  reg_addr = ioremap_nocache(ASIU_TOP_CLK_GATING_CTRL,0x4);
	  rd_data = __raw_readl(reg_addr);
	  wr_data_clk_gate = (rd_data |(1 << 2));
	  __raw_writel( wr_data_clk_gate,reg_addr);
	  iounmap(reg_addr);
	  bypass_disable = 0;
	  mask = 0;
	  reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_MACRO_REG,0x4);
	  wr_data_user = (wr_data_user	|  5 << AUD_FMM_IOP_PLL_0_MACRO_REG__MACRO_SELECT_R );
	  __raw_writel( wr_data_user, (reg_addr));
	  iounmap(reg_addr);
	  reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_USER_MDIV_Ch0_REG,0x4);
	  rd_data = __raw_readl(reg_addr);	
	  mask = AUD_FMM_IOP_PLL_0_USER_MDIV_Ch0_REG_DATAMASK;
	  bypass_disable = rd_data & mask & 0xfffffcff;
	  __raw_writel(bypass_disable, reg_addr);
	  iounmap(reg_addr);
	  reset_val = 0;
	  reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_RESET_REG,0x4);
	  reset_val = (reset_val | 1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD |1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA);
	  __raw_writel(reset_val, reg_addr);
	  reset_val = 0;
	  reset_val = (reset_val |1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD | 0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA );
	  __raw_writel(reset_val, reg_addr);
	  reset_val = 0;
	  reset_val = (reset_val | 0<< AUD_FMM_IOP_PLL_0_RESET_REG__RESETD | 0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA );
	  __raw_writel(reset_val, reg_addr);
	  iounmap(reg_addr);
	  reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_LOCK_STATUS_REG,0x4);
	  do {
	  rd_data=__raw_readl(reg_addr);  
	  timeout--;
	  } while ((rd_data  != 0x00000001) & (timeout > 0));
	  iounmap(reg_addr);
}

/*--------------------------- function -----------------------------------*/
uint32_t asiu_cam_load_coeff(void)
{

	int i;
     cpu_wr_single (CAM_TOP_REGS_CAMIDCA, 0x00000000, 4);
     for (i=0;i<2;i=i+1)
	{
        // 00
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FC20000, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FFE0FFF, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00400000, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00000000, 4);
        // 01
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F521F88,4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FEE0FF8,4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00C80083,4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FF81FFD,4);
        // 02
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1EF11F1F, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FCF0FE1, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0157010F, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FE91FF1, 4);
        // 03
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E9D1EC5, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FA30FBB, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x01EB01A0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FD51FE0, 4);
        // 04
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E571E78, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0F690F88, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x02850238, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FBB1FC8, 4);
        // 05
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E1E1E39, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0F220F47, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x032402D4, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F9C1FAC, 4);
        // 06
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DF01E05, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0ED00EFB, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x03C60375, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F7A1F8B, 4);
        // 07
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DCE1DDE, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0E720EA2, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x046C0419, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F541F67, 4);
        // 08
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DB61DC0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0E0A0E40, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x051404C0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F2C1F40, 4);
        // 09
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA71DAD, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0D990D33, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x05BD0568, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F031F18, 4);
        // 0A
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA21DA3, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0D1E0D5D, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x06680613, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1ED81EED, 4);
        // 0B
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA41DA2, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0C9C0CDE, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x071206BD, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1EAE1EC3, 4);
        // 0C
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DAE1DA8, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0C120C58, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x07BC0768, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1e841e98, 4);
        // 0D
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DBF1DB6, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0B810BCA, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x08650811, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E5B1E6F, 4);
        // 0E
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DD51DC9, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0AEB0B37, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x090B08B9, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E351E47, 4);
        // 0F
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DF11DE3, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0A4F0A9D, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x09AF095E, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E111E22, 4);
        // 10
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E111E00, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x09AF0A00, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0A4F0A00, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DF11E00, 4);
        // 11
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E351E22, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x090B095E, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0AEB0A9D, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DD51DE3, 4);
        // 12
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E5B1E47, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x086508B9, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0B810B37, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DBF1DC9, 4);
        // 13
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E841E6F, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x07BC0811, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0C120BCA, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DAE1DB6, 4);
        // 14
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1EAE1E98, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x07120768, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0C9C0C58, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA41DA8, 4);
        // 15
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1ED81EC3, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x066806BD, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0D1E0CDE, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA21DA2, 4);
        // 16
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F031EED, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x05BD0613, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0D990D5D, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DA71DA3, 4);
        // 17
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F2C1F18, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x05140568, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0E0A0DD3, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DB61DAD, 4);
        // 18
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F541F40, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x046C04C0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0E720E40, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DCE1DC0, 4);
        // 19
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F7A1F67, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x03C60419, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0ED00EA2, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1DF01DDE, 4);
        // 1A
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F9C1F8B, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x03240375, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0F220EFB, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E1E1E05, 4);
        // 1B
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FBB1FAC, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x028502D4, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0F690F47, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E571E39, 4);
        // 1C
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FD51FC8, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x01EB0238, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FA30F88, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1E9D1E78, 4);
        // 1D
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FE91FE0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x015701A0, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FCF0FBB, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1EF11EC5, 4);
        // 1E
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FF81FF1, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00C8010F, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FEE0FE1, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1F521F1F, 4);
        // 1F
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00001FFD, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x00400083, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x0FFE0FF8, 4);
        cpu_wr_single(CAM_TOP_REGS_CAMIDCD,0x1FC21F88, 4);
        // All done
     }
     return 0;
}


/*--------------------------- function -----------------------------------*/
//uint32_t asiu_cam_cpi_init_seq(uint32_t seq_asiu_cam_soft_reset_enable,uint32_t seq_asiu_cam_cpi_data_width,uint32_t seq_asiu_cam_cpi_mode,uint32_t cfg_asiu_horz_window_value,uint32_t cfg_asiu_verti_window_value )
uint32_t asiu_cam_cpi_seq(void)
{

	 uint32_t reg_value;
	if(test_config.seq_asiu_cam_soft_reset_enable == 0)
       cpu_wr_single (CAM_TOP_REGS_CAMCPIS, 0x00000000 , 4 );
	else
       cpu_wr_single (CAM_TOP_REGS_CAMCPIS, 0x00000020 , 4 );
       cpu_wr_single (CAM_TOP_REGS_CAMCPIS, 0x00000001, 4 );

       if (test_config.seq_asiu_cam_cpi_data_width == 8) //8 bit data width
       cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000000, 4 );
       else if (test_config.seq_asiu_cam_cpi_data_width == 10) //10 bit data width
       cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000080 ,4 );
	   else
       cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000000, 4 );

	if (test_config.seq_asiu_cam_cpi_mode == 0)
       {
      	// MODE=HV
    	cpu_wr_single (CAM_TOP_REGS_CAMCPIF, 0x00000000, 4);
		//cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
        //cpu_wr_single(CAM_TOP_REGS_CAMCPIWHC, 0x05000000 ,4);
        //cpu_wr_single(CAM_TOP_REGS_CAMCPIWVC, 0x02D00000 ,4);
	}

       else if (test_config.seq_asiu_cam_cpi_mode == 1)
       	{
       	// Embedded Mode

			cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
          		cpu_wr_single(CAM_TOP_REGS_CAMCPIWHC, 0x08000000 ,4);
          		cpu_wr_single(CAM_TOP_REGS_CAMCPIWVC, 0x00100000 ,4);
          		if (test_config.seq_asiu_cam_cpi_data_width == 10)  // defaults to 10 bit and we can disable embedded sync shift

          		{
			cpu_wr_single (CAM_TOP_REGS_CAMCPIF, (0x00000035 |   // Horizontal Sync Mode=HSAL,Vertical Sync Mode=VSAL,Sync Mode=Embedded
                       		           	1 << 12       |   // enable embedded sync shift
                                      			0 << 13       |   // shift the sync by 4 bits to the left
                                      			7 << 16       |   // vsync bit in the 10 bit data
                                      			6 << 20       |   // hsync bit in the 10 bit data
                                      			8 << 24       )   // field bit in the 10 bit data
                                     			, 4);
          		}
			else
         		{
            		cpu_wr_single (CAM_TOP_REGS_CAMCPIF, (0x00000035 |   // Horizontal Sync Mode=HSAL,Vertical Sync Mode=VSAL,Sync Mode=Embedded
                                      			1 << 12       |   // enable embedded sync shift
                                    			1 << 13       |   // shift the sync by 4 bits to the left
                                      			5 << 16       |   // vsync bit in the 8 bit data
                                      			4 << 20       |   // hsync bit in the 8 bit data
                                      			6 << 24       )   // field bit in the 8 bit data
                                      			, 4);
			}
		}


	else if (test_config.seq_asiu_cam_cpi_mode == 2)
       	{
          if (test_config.seq_asiu_cam_cpi_data_width == 8) //8 bit data width
            cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000002,4 );

          else if (test_config.seq_asiu_cam_cpi_data_width == 10)
            cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000082 ,4); //10 bit data width
          else
            cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000000 ,4);

          cpu_wr_single (CAM_TOP_REGS_CAMCPIF, 0x00000008,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIWHC, test_config.cfg_asiu_horz_window_value,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIWVC, test_config.cfg_asiu_verti_window_value ,4);

        	}
       else
        	{
          cpu_wr_single (CAM_TOP_REGS_CAMCPIF, 0x00000000,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIWHC, test_config.cfg_asiu_horz_window_value,4);
          cpu_wr_single(CAM_TOP_REGS_CAMCPIWVC, test_config.cfg_asiu_verti_window_value ,4);
        	}

        cpu_wr_single (CAM_TOP_REGS_CAMCPIW, 0x00000000,4);

        cpu_wr_single (CAM_TOP_REGS_CAMIDI0,0x00000000,4);

        if (test_config.seq_asiu_cam_cpi_mode == 3)
        cpu_wr_single (CAM_TOP_REGS_CAMIDI0,0x00000000,4);
        else
        cpu_wr_single (CAM_TOP_REGS_CAMIDI0,0x00000080,4);

        cpu_wr_single (CAM_TOP_REGS_CAMIBLS,0x00000140,4);
        cpu_wr_single (CAM_TOP_REGS_CAMDCS,0x00000000,4);
        cpu_wr_single (CAM_TOP_REGS_CAMCTL,0x0002050b,4);

    reg_value = cpu_rd_single(CAM_TOP_REGS_CAMCTL,4);
    reg_value = reg_value & (~(1 << 21));  
    reg_value = reg_value | (1 << 20);  
    reg_value = reg_value | 1;  
    cpu_wr_single(CAM_TOP_REGS_CAMCTL,reg_value,4);
	mdelay(1);
    cpu_wr_single(CAM_TOP_REGS_CAMFIX0, (0x00000000 | 1<<17 | 1<<11 | 1<<10), 4);  // can remove cctv

    return 0;
}

/*--------------------------- function -----------------------------------*/
/*
uint32_t asiu_cam_test_init_seq(uint32_t seq_asiu_cam_soft_reset_enable,uint32_t seq_asiu_cam_cpi_data_width,uint32_t seq_asiu_cam_cpi_mode,
				uint32_t seq_asiu_cam_unpack_option,uint32_t seq_asiu_cam_pack_option,uint32_t cfg_asiu_cam_image_buff_start_addr,
				uint32_t cfg_asiu_cam_image_buff_end_addr,uint32_t cfg_asiu_cam_data_buff_end_addr,uint32_t cfg_asiu_cam_data_buff_start_addr,
				uint32_t cfg_asiu_cam_image_line_stride, uint32_t seq_asiu_horz_window_enable,uint32_t cfg_asiu_horz_window_value,uint32_t seq_asiu_verti_window_enable,
				uint32_t cfg_asiu_verti_window_value,uint32_t seq_asiu_multiple_frames_en, uint32_t cfg_asiu_cam_image_buff_start_addr1, uint32_t cfg_asiu_cam_image_buff_end_addr1,
				uint32_t cam_non_secure_en , uint32_t cpi_clk_neg_edg_en, uint32_t cpi_burst_space_enable)*/
uint32_t asiu_cam_init_seq(void)
{
	  uint32_t data1;
	  uint32_t wr_data;
	  uint32_t rd_data;
	  int test = 0;
	  int speed_mode = 0;
	  int read_value;
	  uint32_t error_value;
	  
	  uint32_t reg_value;
	  
	  read_value = 0;
	  error_value = 0;


		test_config.seq_asiu_cam_soft_reset_enable = 0;
		test_config.seq_asiu_cam_cpi_data_width = 0;
		test_config.seq_asiu_cam_cpi_mode = 0;
		test_config.seq_asiu_cam_unpack_option = 0;
		test_config.seq_asiu_cam_pack_option = 0;
		test_config.cfg_asiu_cam_image_buff_start_addr = 0;
		test_config.cfg_asiu_cam_image_buff_end_addr = 0;
		test_config.cfg_asiu_cam_data_buff_end_addr = 0;
		test_config.cfg_asiu_cam_data_buff_start_addr = 0;
		test_config.cfg_asiu_cam_image_line_stride = 0;
		test_config.seq_asiu_horz_window_enable = 0;
		test_config.cfg_asiu_horz_window_value = 0;
		test_config.seq_asiu_verti_window_enable = 0;
		test_config.cfg_asiu_verti_window_value = 0;
		test_config.seq_asiu_multiple_frames_en = 0;
		test_config.cfg_asiu_cam_image_buff_start_addr1 = 0;
		test_config.cfg_asiu_cam_image_buff_end_addr1 = 0;
		test_config.cam_non_secure_en = 0;
		test_config.cpi_clk_neg_edg_en = 0;
		test_config.cpi_burst_space_enable = 0;

		test_config.seq_asiu_cam_cpi_data_width = 8;
		test_config.seq_asiu_cam_cpi_mode = 0;
test_config.cfg_asiu_cam_image_line_stride = 0x140;
		test_config.seq_asiu_cam_pack_option = 1;
		test_config.seq_asiu_multiple_frames_en = 0;

	  
		cpu_wr_single(CAM_TOP_REGS_CAMFIX0,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMIHWIN,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMIVWIN,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMICC,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMIDC,0x00000000, 4);
		cpu_wr_single(CAM_TOP_REGS_CAMIDPO,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMDCS,0x00000000, 4);
	  
		cpu_wr_single(CAM_TOP_REGS_CAMIPIPE,0x00000000, 4);

		data1 = cpu_rd_single(CAM_TOP_REGS_CAMCTL,4);
		data1 |= 0x00000004;
		cpu_wr_single(CAM_TOP_REGS_CAMCTL,data1,4);
		data1 &= 0xFFFFFFFB;
		cpu_wr_single(CAM_TOP_REGS_CAMCTL,data1,4);
	  
  		asiu_cam_cpi_seq();
	  
		data1 = cpu_rd_single (CAM_TOP_REGS_CAMCTL,4);
		data1 &= 0xFFF00FFF;
		data1 |= 0x00020000;
		cpu_wr_single(CAM_TOP_REGS_CAMCTL,data1,4);
	  
		data1 = cpu_rd_single (CAM_TOP_REGS_CAMCTL,4);
		data1 &= 0xFFF00FFF;
		data1 |= 0x000FF000;
		cpu_wr_single (CAM_TOP_REGS_CAMCTL,data1,4);
	  
	  	cpu_wr_single (CAM_TOP_REGS_CAMFIX0, (0x00000000 | 1<<17 | 1<<11 | 1<<10), 4);
	  
	  
	   asiu_cam_load_coeff();
	  
	   if ((test_config.seq_asiu_cam_unpack_option == 0) && (test_config.seq_asiu_cam_pack_option == 1))
	   {
		 if (test_config.seq_asiu_cam_cpi_data_width == 10)
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000100,4); 
		 else
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000080,4); 
	   }
	  
	  
	   else if ((test_config.seq_asiu_cam_unpack_option == 0) && (test_config.seq_asiu_cam_pack_option == 0))
		 cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000000,4);
	  
	   else if ((test_config.seq_asiu_cam_unpack_option == 1) && (test_config.seq_asiu_cam_pack_option == 0))
	   {
		  if (test_config.seq_asiu_cam_cpi_data_width == 10)
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000004,4);
		 else
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000003,4);
	   }
	  
	  
	   else if ((test_config.seq_asiu_cam_unpack_option == 1) && (test_config.seq_asiu_cam_pack_option == 1))

	   {
	  if (test_config.seq_asiu_cam_cpi_data_width == 10)
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000104,4);  
		 else
		   cpu_wr_single(CAM_TOP_REGS_CAMIPIPE, 0x00000083,4);  
	   }
	  
	   if (test_config.seq_asiu_horz_window_enable == 1)
	   {
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIWHC, test_config.cfg_asiu_horz_window_value ,4);
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIF, 0x0000000A ,4);
		   if (test_config.seq_asiu_cam_cpi_data_width == 8)  
			 cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000003, 4  );
		   else if (test_config.seq_asiu_cam_cpi_data_width == 10)
			 cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000083 ,4 ); 
		   else
			 cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000003, 4 );
	   }
	  
	  cpu_wr_single(CAM_TOP_REGS_CAMCPIF, 0x0000000A ,4);
	  cpu_wr_single ( CAM_TOP_REGS_CAMCPIR, 0x00000083 ,4 ); 
	  
	   if (test_config.seq_asiu_verti_window_enable == 1)
	   {
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIW, 0x00000001 ,4);
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIWVC, test_config.cfg_asiu_verti_window_value ,4);
		 cpu_wr_single(CAM_TOP_REGS_CAMCPIF, 0x0000000A ,4);
		   if (test_config.seq_asiu_cam_cpi_data_width == 8) 
			 cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000003, 4  );
		   else if (test_config.seq_asiu_cam_cpi_data_width == 10)
			 cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000083 ,4 ); 
		   else
			 cpu_wr_single (CAM_TOP_REGS_CAMCPIR, 0x00000003, 4 );
	   }
	  
	   //cpu_wr_single(`CAMIBLS,  0x00000800,4);
	   cpu_wr_single(CAM_TOP_REGS_CAMIBLS,test_config.cfg_asiu_cam_image_line_stride,4);
	  
	   //For Multiple Frames Buffer1 Address has to configured
	   if (test_config.seq_asiu_multiple_frames_en == 1)
	   {
	   cpu_wr_single(CAM_TOP_REGS_CAMDBCTL, 0x00000001,4); 
	   cpu_wr_single(CAM_TOP_REGS_CAMICTL, 0x00000030,4);
	   cpu_wr_single(CAM_TOP_REGS_CAMIBSA1, test_config.cfg_asiu_cam_image_buff_start_addr1 ,4);
	   cpu_wr_single(CAM_TOP_REGS_CAMIBEA1, test_config.cfg_asiu_cam_image_buff_end_addr1 ,4);
	   }
	    
	   if (test_config.seq_asiu_cam_cpi_mode == 3)
	   {
		   post_log("CAMERA APIS: Enabling JPEG Mode\n");
		 cpu_wr_single (CAM_TOP_REGS_CAMDCS,0x00010021,4);
	   }
	   else
		 cpu_wr_single (CAM_TOP_REGS_CAMDCS,0x00000000,4);
	  
	   if (test_config.cpi_clk_neg_edg_en)
	  {
		cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xffffffe3) | 0x0000001c) , 4 );
	  }
	   if (test_config.cpi_burst_space_enable)
	   {
		 //Burst Space Configuration
		 cpu_wr_single (CAM_TOP_REGS_CAMPRI, ((cpu_rd_single(CAM_TOP_REGS_CAMPRI,4) & 0xffff0fff) | 0x00008000) , 4 );
	  
		 //Enable FRAME Start and End Interrupts
		 cpu_wr_single (CAM_TOP_REGS_CAMICTL, ((cpu_rd_single(CAM_TOP_REGS_CAMICTL,4) & 0xffffffC4) | 0x0000003B) , 4 );
	  
		 
	   }


#ifdef MUL			  
	   //cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xffffffe0) | 0x0000001f) , 4 );
		cpu_wr_single (CAM_TOP_REGS_CAMCPIR, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIR,4) & 0xfffffff0) | 0x0000000f) , 4 ); //Vsync is active low and hsync is active low
	  
	   //Enable FRAME Start and End Interrupts
		 cpu_wr_single (CAM_TOP_REGS_CAMICTL, ((cpu_rd_single(CAM_TOP_REGS_CAMICTL,4) & 0xffffffC4) | 0x0000003B) , 4 );
	  
		 //Enalbe Interrupt -- Reg. config. during CPI Capture
		 cpu_wr_single (CAM_TOP_REGS_CAMCPIS, ((cpu_rd_single(CAM_TOP_REGS_CAMCPIS,4) & 0x7ffffffc) | 0x80008003) , 4 );
 
#endif		  
		  return test;

}

static int __devinit unicam_camera_probe(struct platform_device *pdev)
{
	struct unicam_camera_dev *unicam_dev;
	struct soc_camera_host *soc_host;
	struct resource        *res   = NULL;
	struct resource *cam_reg_mem;  /* pointer to requested register memory region */
	void __iomem    *cam_reg_base; /* pointer to iomapped register memory region */
	int irq;
	int err = 0;
	int ret;
	void __iomem *reg_addr=0;
	unsigned int reg_val;

	reg_addr = ioremap_nocache(CRMU_IOMUX_CTRL7_OFFSET,0x4);  	
	reg_val = __raw_readl(reg_addr);  
	reg_val &= ~(0x7ff);	
	__raw_writel( reg_val, (reg_addr));
    iounmap(reg_addr);
	reg_addr = ioremap_nocache(ASIU_TOP_PAD_CTRL_0,0x4);  	
	reg_val = __raw_readl(reg_addr);  
	reg_val |= 0x100;	
	__raw_writel( reg_val, (reg_addr));
    iounmap(reg_addr);
	reg_addr = ioremap_nocache(ASIU_TOP_SW_RESET_CTRL,0x14);  	
	reg_val = __raw_readl(reg_addr);  
	reg_val |= (1<< ASIU_TOP_SW_RESET_CTRL__CAM_SW_RESET_N);	
	__raw_writel( reg_val, (reg_addr));
	reg_val = __raw_readl(reg_addr+ 4);  
	reg_val |= (1<< ASIU_TOP_CLK_GATING_CTRL__CAM_CLK_GATE_EN);
	__raw_writel( reg_val, (reg_addr + 4));
	reg_val = __raw_readl(reg_addr+ 0x10);  
	reg_val &= 0xffffe3ff;
	__raw_writel( reg_val, (reg_addr + 0x10));
    iounmap(reg_addr);

	MM_CFG_BASE  = (unsigned int)ioremap_nocache(ASIU_TOP_CLK_GATING_CTRL,0x4);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		err = -ENODEV;
		goto edev;
	}

	unicam_dev = vzalloc(sizeof(*unicam_dev));
	if (!unicam_dev) {
		dev_err(&pdev->dev,
			"Could not allocate unicam camera object\n");
		err = -ENOMEM;
		goto ealloc;
	}

	INIT_LIST_HEAD(&unicam_dev->capture);
	spin_lock_init(&unicam_dev->lock);

	/*---------- Register Address -------------*/
	/* get  register address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0 /* the first IORESOURCE_MEM is what we want */ );
	if(res == NULL)
	{
		pr_err("ERROR: Could not get Platform Resource LCD Register Memory Resource\n");
		ret = -ENXIO;
		goto ealloc;
	}

	/* request memory region */
	cam_reg_mem = request_mem_region(res->start, (res->end - res->start + 1), pdev->name);
	if (cam_reg_mem == NULL)
	{
		pr_err("ERROR: Could not request mem region, for LCD Register Memory Locations\n");
		ret = -ENOENT;
		goto ealloc;
	}

	/* map */
	cam_reg_base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (cam_reg_base == NULL) {
		pr_err("Unable to mape device registers\n");
		ret = -ENXIO;
		goto ealloc;
	}

    V_BASE = (unsigned int)cam_reg_base;
    unicam_dev->csi_base = cam_reg_base;
	unicam_dev->dev = &pdev->dev;
	unicam_dev->irq = irq;
	soc_host = &unicam_dev->soc_host;
	soc_host->drv_name = UNICAM_CAM_DRV_NAME;
	soc_host->ops = &unicam_soc_camera_host_ops;
	soc_host->priv = unicam_dev;
	soc_host->v4l2_dev.dev = &pdev->dev;
	soc_host->nr = pdev->id;
	atomic_set(&unicam_dev->stopping, 0);
	sema_init(&unicam_dev->stop_sem, 0);

#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	unicam_dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(unicam_dev->alloc_ctx)) {
		err = PTR_ERR(unicam_dev->alloc_ctx);
		goto eallocctx;
	}
#endif
	
	err = soc_camera_host_register(soc_host);
	if (err)
		goto ecamhostreg;
	
	mdelay(1);
	
	return 0;

ecamhostreg:
#if defined(CONFIG_VIDEOBUF2_DMA_CONTIG)
	vb2_dma_contig_cleanup_ctx(unicam_dev->alloc_ctx);
eallocctx:
#endif
	vfree(unicam_dev);
ealloc:
edev:
	return err;
}

static int __devexit unicam_camera_remove(struct platform_device *pdev)
{
	/*TODO */
	return 0;
}

static struct platform_driver unicam_camera_driver = {
	.driver = {
		   .name = UNICAM_CAM_DRV_NAME,
		   },
	.probe = unicam_camera_probe,
	.remove = __devexit_p(unicam_camera_remove),
};

static int __init unicam_camera_init(void)
{
	int err = -1;
	
	err = platform_driver_register(&unicam_camera_driver);
	if (err)
	{
		return err;
	}

	return err;
}

static void __exit unicam_camera_exit(void)
{
	platform_driver_unregister(&unicam_camera_driver);
}

module_init(unicam_camera_init);
module_exit(unicam_camera_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Unicam Camera Host driver");
MODULE_AUTHOR("Broadcom <broadcom@broadcom.com>");
