/*
 * OmniVision OV5640 sensor driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 *modify it under the terms of the GNU General Public License as
 *published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 *kind, whether express or implied; without even the implied warranty
 *of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/version.h>
	 
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <asm/mach/irq.h>
	 //#include <mach/platform.h>
#include <linux/version.h>
#include <mach/irqs.h>
	 //#include <asm-arm/irq.h>
	 //#include <asm-arm/delay.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <asm/uaccess.h>	/* for copy_from_user */
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/pwm.h>
	 
#include <mach/socregs-cygnus.h>

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include "iproc_videodev2.h"
#include "iproc_ov5640.h"

#define V4L2_IDENT_OV5640  262

/* #define OV5640_DEBUG */

#define iprintk(format, arg...)	\
	printk(KERN_INFO"[%s]: "format"\n", __func__, ##arg)

/* OV5640 has only one fixed colorspace per pixelcode */
struct ov5640_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

struct ov5640_timing_cfg {
	u16 x_addr_start;
	u16 y_addr_start;
	u16 x_addr_end;
	u16 y_addr_end;
	u16 h_output_size;
	u16 v_output_size;
	u16 h_total_size;
	u16 v_total_size;
	u16 isp_h_offset;
	u16 isp_v_offset;
	u8 h_odd_ss_inc;
	u8 h_even_ss_inc;
	u8 v_odd_ss_inc;
	u8 v_even_ss_inc;
	u8 out_mode_sel;
	u8 sclk_dividers;
	u8 sys_mipi_clk;

};

static const struct ov5640_datafmt ov5640_fmts[] = {
	/*
	 * Order important: first natively supported,
	 *second supported with a GPIO extender
	 */
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
/*	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG}, */

};

enum ov5640_size {
	OV5640_SIZE_QVGA,	/*  320 x 240 */
	OV5640_SIZE_VGA,	/*  640 x 480 */
	OV5640_SIZE_720P,
	OV5640_SIZE_1280x960,	/*  1280 x 960 (1.2M) */
	OV5640_SIZE_UXGA,	/*  1600 x 1200 (2M) */
	OV5640_SIZE_QXGA,	/*  2048 x 1536 (3M) */
	OV5640_SIZE_5MP,
	OV5640_SIZE_LAST,
	OV5640_SIZE_MAX
};

static const struct v4l2_frmsize_discrete ov5640_frmsizes[OV5640_SIZE_LAST] = {
	{320, 240},
	{640, 480},
	{1280, 720},
	{1280, 960},
	{1600, 1200},
	{2048, 1536},
	{2560, 1920},
};

/* Scalers to map image resolutions into AF 80x60 virtual viewfinder */
static const struct ov5640_af_zone_scale af_zone_scale[OV5640_SIZE_LAST] = {
	{4, 4},
	{8, 8},
	{16, 12},
	{16, 16},
	{20, 20},
	{26, 26},
	{32, 32},
};

/* Find a data format by a pixel code in an array */
static int ov5640_find_datafmt(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5640_fmts); i++)
		if (ov5640_fmts[i].code == code)
			break;

	/* If not found, select latest */
	if (i >= ARRAY_SIZE(ov5640_fmts))
		i = ARRAY_SIZE(ov5640_fmts) - 1;

	return i;
}

/* Find a frame size in an array */
static int ov5640_find_framesize(u32 width, u32 height)
{
	int i;

	for (i = 0; i < OV5640_SIZE_LAST; i++) {
		if ((ov5640_frmsizes[i].width >= width) &&
		    (ov5640_frmsizes[i].height >= height))
			break;
	}

	/* If not found, select biggest */
	if (i >= OV5640_SIZE_LAST)
		i = OV5640_SIZE_LAST - 1;

	return i;
}

struct ov5640 {
	struct v4l2_subdev subdev;
	int i_size;
	int i_fmt;
	int brightness;
	int contrast;
	int colorlevel;
	int sharpness;
	int saturation;
	int antibanding;
	int whitebalance;
	int framerate;
	int focus_mode;
	/*
	 * focus_status = 1 focusing
	 * focus_status = 0 focus cancelled or not focusing
	 */
	atomic_t focus_status;

	/*
	 * touch_focus holds number of valid touch focus areas. 0 = none
	*/
	int touch_focus;
	v4l2_touch_area touch_area[OV5640_MAX_FOCUS_AREAS];
	int flashmode;
};

static struct ov5640 *to_ov5640(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov5640, subdev);
}


static const struct ov5640_timing_cfg timing_cfg_yuv[OV5640_SIZE_LAST] = {
	[OV5640_SIZE_QVGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 320,
			      .v_output_size = 240,
			/*  ISP Windowing size 1296 x 972 --> 1280 x 960 */
			      .isp_h_offset = 8,
			      .isp_v_offset = 6,
			/*  Total size (+blanking) */
			      .h_total_size = 2200,
			      .v_total_size = 1280,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 3,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 3,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x01,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x11,
			       },
	[OV5640_SIZE_VGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 640,
			      .v_output_size = 480,
			/*  ISP Windowing size  1296 x 972 --> 1280 x 960 */
			      .isp_h_offset = 8,
			      .isp_v_offset = 6,
			/*  Total size (+blanking) */
			      .h_total_size = 2200,
			      .v_total_size = 1280,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 3,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 3,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x01,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x11,
			       },
	[OV5640_SIZE_720P] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1280,
			      .v_output_size = 720,
			/*  ISP Windowing size  1296 x 972 --> 1280 x 960 */
			      .isp_h_offset = 8,
			      .isp_v_offset = 6,
			/*  Total size (+blanking) */
			      .h_total_size = 2200,
			      .v_total_size = 1280,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 3,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 3,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x01,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x11,
			      },
	[OV5640_SIZE_1280x960] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1280,
			      .v_output_size = 960,
			/*  ISP Windowing size  1296 x 972 --> 1280 x 960 */
			      .isp_h_offset = 8,
			      .isp_v_offset = 6,
			/*  Total size (+blanking) */
			      .h_total_size = 2200,
			      .v_total_size = 1280,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 3,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 3,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x01,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x11,
			      },
	[OV5640_SIZE_UXGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1600,
			      .v_output_size = 1200,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x00,
			      .sclk_dividers = 0x02,
			      .sys_mipi_clk = 0x12,
			      },
	[OV5640_SIZE_QXGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 2048,
			      .v_output_size = 1536,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x00,
			      .sclk_dividers = 0x02,
			      .sys_mipi_clk = 0x12,
			      },
	[OV5640_SIZE_5MP] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 2560,
			      .v_output_size = 1920,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Enabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x00,
			      .sclk_dividers = 0x02,
			      .sys_mipi_clk = 0x12,
			      },
};

static const struct ov5640_timing_cfg timing_cfg_jpeg[OV5640_SIZE_LAST] = {
	[OV5640_SIZE_QVGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 320,
			      .v_output_size = 240,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
	[OV5640_SIZE_VGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 640,
			      .v_output_size = 480,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
	[OV5640_SIZE_720P] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1280,
			      .v_output_size = 720,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
	[OV5640_SIZE_1280x960] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1280,
			      .v_output_size = 960,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			      },
	[OV5640_SIZE_UXGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 1600,
			      .v_output_size = 1200,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
	[OV5640_SIZE_QXGA] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/*  Output image size */
			      .h_output_size = 2048,
			      .v_output_size = 1536,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/*  Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/*  Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
	[OV5640_SIZE_5MP] = {
			/*  Timing control  2624 x 1952 --> 2592 x 1944 */
			      .x_addr_start = 16,
			      .y_addr_start = 4,
			      .x_addr_end = 2607,
			      .y_addr_end = 1947,
			/* Output image size */
			      .h_output_size = 2560,
			      .v_output_size = 1920,
			/*  ISP Windowing size	2592 x 1944 --> 2560 x 1920 */
			      .isp_h_offset = 16,
			      .isp_v_offset = 12,
			/* Total size (+blanking) */
			      .h_total_size = 2844,
			      .v_total_size = 1968,
			/* Sensor Read Binning Disabled */
			      .h_odd_ss_inc = 1,
			      .h_even_ss_inc = 1,
			      .v_odd_ss_inc = 1,
			      .v_even_ss_inc = 1,
			      .out_mode_sel = 0x26,
			      .sclk_dividers = 0x01,
			      .sys_mipi_clk = 0x12,
			       },
};

#define post_log(...) 
#define SCCB_ID 0x3c
uint32_t smbus; 
static uint32_t cpu_rd_single(uint32_t addr, int size) {
    return ((size == 1) ?  *((volatile uint8_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))) :
                            ((size == 2) ? *((volatile uint16_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))) :
                            (*((volatile uint32_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))))));
};

static void cpu_wr_single(uint32_t addr, uint32_t data, int size) {
    if(size == 1) *((volatile uint8_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))) = data ;
    else if(size == 2) *((volatile uint16_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))) = data ;
    else *((volatile uint32_t *)(smbus - ChipcommonG_SMBus0_SMBus_Config + (addr))) = data;
};


static struct resource smbus_resources = {
	    .start = ChipcommonG_SMBus0_SMBus_Config,
		.end   = ChipcommonG_SMBus0_SMBus_Config + 0x1000 - 1,
		.name = "smbus",
		.flags  = IORESOURCE_MEM,

};
/******************************************/
int cam_sbp(uint32_t regess)
{
	uint32_t data,regess1;
		 for(regess1=0;regess1<=0x7F;regess1++)
		 {
		 cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Data_Write), (regess1<<1), 4);
		            cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Command), ((0 << ChipcommonG_SMBus0_SMBus_Master_Command__SMBUS_PROTOCOL_R) | (1<<31)), 4);
		            do {
		                data = cpu_rd_single((ChipcommonG_SMBus0_SMBus_Master_Command), 4);
		                data &= (1 << 31);
		            } while (data);
		            data = cpu_rd_single((ChipcommonG_SMBus0_SMBus_Master_Command), 4);
		            data = (data >> 25) & 0x7;
		 }
}

/*******************************/
int sb_read(uint32_t address)
{
	uint32_t data,reg;
	uint8_t value;

	reg = SCCB_ID;

	   cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Data_Write, ((reg << 1) | 1 | (1 << 31)), 4);
	   cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Command, ((0x4 << 9) | (1<<31)|(0x01)), 4);
	   data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);

	   do {
		   data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
		   data &= (1 << 31);
		   } while (data);

		data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
		
		data = (data >> 25) & 0x7;

		if(data != 0)
		{
			return -1;
		}
		else
		{

		   do
		   {
			  data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Data_Read, 4);
			  value = data & 0x000000FF;
			  data = (data & 0xc0000000);
		   } while (data != 0xc0000000);
			return value;
		}
}

int sb_write_1(uint32_t reg,uint32_t address)
{
	uint32_t data;

 	   cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Data_Write, ((reg << 1) | 0), 4);
       cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Data_Write, ((address & 0xFF00)>>8), 4);
       cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Data_Write, (address & 0x00FF) | (1 << 31), 4);
       cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Command, ((0x5 << 9) | (1<<31)), 4);
       data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);

       do {
           data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
           data &= (1 << 31);
           } while (data);

        data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
		
        data = (data >> 25) & 0x7;

        if(data != 0)
        {
            return -1;
        }
        else
 		{
 		    return 0;
 		}


}

/*********************************/
int sb_write(uint32_t reg,uint32_t address,uint32_t value)
{
	uint32_t data;

 	   cpu_wr_single((+ChipcommonG_SMBus0_SMBus_Master_Data_Write), ((reg << 1) | 0), 4);
       cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Data_Write), ((address & 0xFF00)>>8), 4);
       cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Data_Write), (address & 0x00FF), 4);
       cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Data_Write), (value | (1 << 31)), 4);
       cpu_wr_single((ChipcommonG_SMBus0_SMBus_Master_Command), ((0x7 << 9) | (1<<31)), 4);
       data = cpu_rd_single((ChipcommonG_SMBus0_SMBus_Master_Command), 4);
       do {
           data = cpu_rd_single((ChipcommonG_SMBus0_SMBus_Master_Command), 4);
           data &= (1 << 31);
           } while (data);
        data = cpu_rd_single((ChipcommonG_SMBus0_SMBus_Master_Command), 4);
        data = (data >> 25) & 0x7;
        if(data != 0)
        {
            return -1;
        }
        else
 		{
 		    return 0;
 		}


}
void sm_init1(uint32_t speed_mode)
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
	uint32_t data,regess1;
	void __iomem *reg_addr=0;
	unsigned int reg_val = 0;

	reg_addr = ioremap_nocache(CRMU_CHIP_IO_PAD_CONTROL,0x4);	
	__raw_writel( 0, (reg_addr));
	iounmap(reg_addr);

	reg_addr = ioremap_nocache(CRMU_IOMUX_CTRL7,0x4);	
	reg_val = __raw_readl(reg_addr);  
	reg_val = 0;	
	__raw_writel( reg_val, (reg_addr));
	iounmap(reg_addr);
	reg_addr = ioremap_nocache(ASIU_TOP_PAD_CTRL_0,0x4);	
	reg_val = __raw_readl(reg_addr);  
	reg_val |= 0x100;	//0x101
	__raw_writel( reg_val, (reg_addr));
	iounmap(reg_addr);
	reg_addr = ioremap_nocache(ASIU_TOP_SW_RESET_CTRL,0x14);
	__raw_writel( 0, (reg_addr));
	reg_val = __raw_readl(reg_addr);  
	reg_val |= 0x000003ff;	 //(1<< ASIU_TOP_SW_RESET_CTRL__CAM_SW_RESET_N);
	__raw_writel( reg_val, (reg_addr));
	reg_val = __raw_readl(reg_addr+ 4);  
	reg_val |= (1<< ASIU_TOP_CLK_GATING_CTRL__CAM_CLK_GATE_EN) | (1 << ASIU_TOP_CLK_GATING_CTRL__AUD_CLK_GATE_EN);
	__raw_writel( reg_val, (reg_addr + 4));
	reg_val = __raw_readl(reg_addr+ 0x10);	
	reg_val &= 0xffffe3ff;
	__raw_writel( reg_val, (reg_addr + 0x10));
	iounmap(reg_addr);

	reg_addr = ioremap_nocache(CRMU_PLL_AON_CTRL,0x4);
	__raw_writel( ((__raw_readl(reg_addr) & 0xfffffbff) | 0x00000400), (reg_addr));
	msleep(1);
	__raw_writel( ((__raw_readl(reg_addr) & 0xfffff5ff) | 0x00000a00), (reg_addr));
	msleep(1);
	__raw_writel( ((__raw_readl(reg_addr) & 0xfffffeff)), (reg_addr));
	iounmap(reg_addr);
	msleep(1);
    reg_addr = ioremap_nocache(ASIU_TOP_CLK_GATING_CTRL,0x4);
	rd_data = __raw_readl(reg_addr);
	wr_data_clk_gate = (rd_data |(1 << 2));
	__raw_writel( wr_data_clk_gate,reg_addr);
    iounmap(reg_addr);
    msleep(1);
	bypass_disable = 0;
	mask = 0;
	reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_MACRO_REG,0x4);
	wr_data_user = (wr_data_user  |  5 << AUD_FMM_IOP_PLL_0_MACRO_REG__MACRO_SELECT_R );
	__raw_writel( wr_data_user, (reg_addr));
    iounmap(reg_addr);
    msleep(1);
	reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_USER_MDIV_Ch0_REG,0x4);
	rd_data = __raw_readl(reg_addr);  
	mask = AUD_FMM_IOP_PLL_0_USER_MDIV_Ch0_REG_DATAMASK;
	bypass_disable = rd_data & mask & 0xfffffcff;
	__raw_writel(bypass_disable, reg_addr);
    iounmap(reg_addr);
    msleep(1);
	reset_val = 0;
	reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_RESET_REG,0x4);
	reset_val = (reset_val | 1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD |1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA);
	__raw_writel(reset_val, reg_addr);
    msleep(1);
	reset_val = 0;
 	reset_val = (reset_val |1 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETD | 0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA	);
	__raw_writel(reset_val, reg_addr);
    msleep(1);
	reset_val = 0;
	reset_val = (reset_val | 0<< AUD_FMM_IOP_PLL_0_RESET_REG__RESETD | 0 << AUD_FMM_IOP_PLL_0_RESET_REG__RESETA );
	__raw_writel(reset_val, reg_addr);
    iounmap(reg_addr);
    msleep(1);
	reg_addr = ioremap_nocache(AUD_FMM_IOP_PLL_0_LOCK_STATUS_REG,0x4);
	do {
	rd_data=__raw_readl(reg_addr);  
	timeout--;
	} while ((rd_data  != 0x00000001) & (timeout > 0));
    iounmap(reg_addr);
    msleep(1);
	reg_addr = ioremap_nocache(ASIU_TOP_PAD_CTRL_0,0x4);
	__raw_writel(1, reg_addr );
    iounmap(reg_addr);
    msleep(1);			
    reg_addr = ioremap_nocache(ASIU_TOP_PAD_CTRL_0,0x4);
    reg_value = __raw_readl(reg_addr);
    reg_value = reg_value | 0x100;
    __raw_writel(reg_value, reg_addr);
    iounmap(reg_addr);	
    msleep(1);
	reg_addr = ioremap_nocache(ASIU_TOP_CAM_AXI_SB_CTRL,0x4); 
	rd_data = __raw_readl(reg_addr); 
	wr_data = rd_data &  0xffffe3ff; 
	__raw_writel(wr_data, reg_addr);
	iounmap(reg_addr);
	reg_addr = ioremap_nocache(ASIU_TOP_CAM_AXI_SB_CTRL,0x4);
	reg_value = __raw_readl(reg_addr);
	reg_value = reg_value &  0xffffe3ff;
	__raw_writel(reg_value, reg_addr);
	iounmap(reg_addr);  
	msleep(1);
	reg_addr = ioremap_nocache(CAM_TOP_REGS_CAMCTL,0x4);
	reg_value = __raw_readl(reg_addr); 
	reg_value = reg_value & (~(1 << 21)); 
	reg_value = reg_value | (1 << 20); 
	reg_value = reg_value | 1; 
	__raw_writel(reg_value, reg_addr);
    iounmap(reg_addr);
	reg_addr = ioremap_nocache(CAM_TOP_REGS_CAMFIX0,0x4);
	__raw_writel((0x00000000 | 1<<17 | 1<<11 | 1<<10), reg_addr);
	iounmap(reg_addr);  
	msleep(1);

	data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Config,4);
	data |= (1 << ChipcommonG_SMBus0_SMBus_Config__SMB_EN);
	cpu_wr_single(ChipcommonG_SMBus0_SMBus_Config, data,4);
	data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Config,4);

	data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Config,4);
	data |= (1 << ChipcommonG_SMBus0_SMBus_Config__RESET);
	cpu_wr_single(ChipcommonG_SMBus0_SMBus_Config, data,4);
	data &= ~(1 << ChipcommonG_SMBus0_SMBus_Config__RESET);
	cpu_wr_single(ChipcommonG_SMBus0_SMBus_Config, data,4);

	 if(1) {
		   data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Timing_Config,4);
		   data |= (1 << ChipcommonG_SMBus0_SMBus_Timing_Config__MODE_400);
		   cpu_wr_single(ChipcommonG_SMBus0_SMBus_Timing_Config, data,4);
	   } else {
		   data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Timing_Config,4);
		   data &= ~(1 << ChipcommonG_SMBus0_SMBus_Timing_Config__MODE_400);
		   cpu_wr_single(ChipcommonG_SMBus0_SMBus_Timing_Config, data,4);
	   }
	   msleep(1);

		sb_write_1(SCCB_ID,0x302A);

	    read_value = sb_read(0x302A);

	 for(regess1=0;regess1<=0x7F;regess1++)
	 {
        cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Data_Write, (regess1<<1), 4);
        cpu_wr_single(ChipcommonG_SMBus0_SMBus_Master_Command, ((0 << ChipcommonG_SMBus0_SMBus_Master_Command__SMBUS_PROTOCOL_R) | (1<<31)), 4);
        do {
            data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
            data &= (1 << 31);
        } while (data);
        data = cpu_rd_single(ChipcommonG_SMBus0_SMBus_Master_Command, 4);
        data = (data >> 25) & 0x7;
	 }
	

 }
/********************************/
 int smb_write( const struct ov5640_reg reglist[])
{
	int err = 0, index;
	uint32_t reg;

	reg = SCCB_ID;

	for (index = 0; ((reglist[index].reg != 0xFFFF) && (err == 0)); index++)
	{
		err |= sb_write(reg, reglist[index].reg,reglist[index].val);
		mdelay(2);
		//soft delay of 10 microsecond added.
		//plus ..smbus_default_idle time is 50 micor second

	}
	return 0;
}

int sm_init()
{
    int ret;
    int i=0;
	void __iomem *smbus_io;  

	/* map */
	smbus_io = ioremap_nocache(smbus_resources.start, smbus_resources.end - smbus_resources.start + 1);
	if (smbus_io == NULL) {
		ret = -ENXIO;
	}
	smbus = (uint32_t)smbus_io;
	sm_init1(1);
	mdelay(1);
	cam_sbp(SCCB_ID);
   return ret;
}
static int ov5640_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	union i2c_smbus_data i2cdata;
	int status;

	sb_write_1(SCCB_ID,reg );
	*val = sb_read(reg);
	
    return 0;

#if 0	
	i2cdata.word = reg & 0xFF;
	/* Value to write copied in upper 8 bits */
	i2cdata.word |= (*val << 8);
	printk("%s 111111111111 reg = 0x%x i2cdata.word = 0x%x \n",__func__,reg, i2cdata.word);

	status = i2c_smbus_xfer(client->adapter, client->addr, 0x0,
						I2C_SMBUS_READ, ((reg >> 8) & 0xFF),
						I2C_SMBUS_BYTE_DATA, &i2cdata);

    *val = i2cdata.word >> 8;
	printk("%s 2222222222222 status = 0x%x reg = 0x%x *val = 0x%x i2cdata.word = 0x%x\n",__func__,status, reg, *val,i2cdata.word);
	return (status < 0) ? status : i2cdata.byte;


	int ret;
	u8 data[2] = { 0 };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = data,
	};

	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	printk(" %s 00000 \n",__func__);

	ret = i2c_smbus_write_byte_data(client, data, data[0]);
		printk(" %s 111111 ret = 0x%x  val = 0x%x \n",__func__,ret,*val);
	ret = i2c_smbus_write_byte_data(client, I2C_FUNC_SMBUS_WRITE_BYTE, data[1]);
	
	printk(" %s 222222 ret = 0x%x  val = 0x%x \n",__func__,ret,*val);
	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	msg.len = 1;
	ret = i2c_smbus_read_byte_data(client, I2C_FUNC_SMBUS_READ_BYTE_DATA);

	printk(" %s 33333 ret = 0x%x  val = 0x%x \n",__func__,ret,*val);
	if (ret < 0)
		goto err;

	*val = data[0];
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
#endif
}
static int ov5640_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret = 0;

#if 0
	union i2c_smbus_data i2cdata;
	int status;

	printk("%s w0 bbbbbbbbbbbbegin reg = 0x%x *val = 0x%x \n",__func__,reg, val);


	i2cdata.word = reg & 0xFF;
	/* Value to write copied in upper 8 bits */
	i2cdata.word |= (val << 8);
	printk("%s w1 reg = 0x%x i2cdata.word = 0x%x \n",__func__,reg, i2cdata.word);

	status = i2c_smbus_xfer(client->adapter, client->addr, 0x0,
						I2C_SMBUS_WRITE, ((reg >> 8) & 0xFF),
						I2C_SMBUS_BYTE_DATA, &i2cdata);

	printk("%s w2 end status = 0x%x reg = 0x%x *val = 0x%x i2cdata.word = 0x%x\n",__func__,status, reg, val,i2cdata.word);
//	return status;
#endif

   ret = sb_write(SCCB_ID,reg,  val);
   return ret;

#if 0
	int ret;
	unsigned char data[3] = { (u8) (reg >> 8), (u8) (reg & 0xff), val };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 3,
		.buf = data,
	};
	printk(" %s 0000 \n",__func__);

	ret = i2c_transfer(client, reg, val);
	
	printk(" %s 0000  ret = 0x%x \n",__func__,ret);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
#endif
}

static const struct v4l2_queryctrl ov5640_controls[] = {
	{
	 .id = V4L2_CID_CAMERA_BRIGHTNESS,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Brightness",
	 .minimum = EV_MINUS_1,
	 .maximum = EV_PLUS_1,
	 .step = 1,
	 .default_value = EV_DEFAULT,
	 },
	{
	 .id = V4L2_CID_CAMERA_CONTRAST,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Contrast",
	 .minimum = CONTRAST_MINUS_1,
	 .maximum = CONTRAST_PLUS_1,
	 .step = 1,
	 .default_value = CONTRAST_DEFAULT,
	 },
	{
	 .id = V4L2_CID_CAMERA_FLASH_MODE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
#ifdef CONFIG_VIDEO_AS3643
	 .name = "AS3643-flash",
#else
	 .name = "ADP1653-flash",
#endif
	 .minimum = FLASH_MODE_OFF,
	 .maximum = (1 << FLASH_MODE_OFF) | (1 << FLASH_MODE_ON) | (1 << FLASH_MODE_TORCH_OFF) |
	 		(1 << FLASH_MODE_TORCH_ON),
	 .step = 1,
	 .default_value = FLASH_MODE_OFF,
	 },
	{
	 .id = V4L2_CID_CAMERA_EFFECT,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Color Effects",
	 .minimum = IMAGE_EFFECT_NONE,
	 .maximum = (1 << IMAGE_EFFECT_NONE | 1 << IMAGE_EFFECT_SEPIA |
			 1 << IMAGE_EFFECT_BNW | 1 << IMAGE_EFFECT_NEGATIVE),
	 .step = 1,
	 .default_value = IMAGE_EFFECT_NONE,
	 },
	{
	 .id = V4L2_CID_CAMERA_ANTI_BANDING,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Anti Banding",
	 .minimum = ANTI_BANDING_AUTO,
	 .maximum = ANTI_BANDING_60HZ,
	 .step = 1,
	 .default_value = ANTI_BANDING_AUTO,
	 },
	 {
	 .id = V4L2_CID_CAMERA_WHITE_BALANCE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "White Balance",
	 .minimum = WHITE_BALANCE_AUTO,
	 .maximum = WHITE_BALANCE_FLUORESCENT,
	 .step = 1,
	 .default_value = WHITE_BALANCE_AUTO,
	 },
	 {
	 .id = V4L2_CID_CAMERA_FRAME_RATE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Framerate control",
	 .minimum = FRAME_RATE_AUTO,
	 .maximum = (1 << FRAME_RATE_AUTO | 1 << FRAME_RATE_5 |
			 1 << FRAME_RATE_10 | 1 << FRAME_RATE_15 |
			 1 << FRAME_RATE_25 | 1 << FRAME_RATE_30),
	 .step = 1,
	 .default_value = FRAME_RATE_AUTO,
	 },
	{
	 .id = V4L2_CID_CAMERA_FOCUS_MODE,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Focus Modes",
	 .minimum = FOCUS_MODE_AUTO,
	 .maximum = (1 << FOCUS_MODE_AUTO | 1 << FOCUS_MODE_MACRO
			 | 1 << FOCUS_MODE_INFINITY),
	 .step = 1,
	 .default_value = FOCUS_MODE_AUTO,
	 },
	{
	 .id = V4L2_CID_CAMERA_SET_AUTO_FOCUS,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "AF start/stop",
	 .minimum = AUTO_FOCUS_OFF,
	 .maximum = AUTO_FOCUS_ON,
	 .step = 1,
	 .default_value = AUTO_FOCUS_OFF,
	 },
	{
	 .id = V4L2_CID_CAMERA_TOUCH_AF_AREA,
	 .type = V4L2_CTRL_TYPE_INTEGER,
	 .name = "Touch focus areas",
	 .minimum = 0,
	 .maximum = OV5640_MAX_FOCUS_AREAS,
	 .step = 1,
	 .default_value = 1,
	},

};
static int ov5640_reg_writes(struct i2c_client *client,
			const struct ov5640_reg reglist[])
{
	int err = 0, index;

	for (index = 0; ((reglist[index].reg != 0xFFFF) && (err == 0));
								index++) {
		err |=
			ov5640_reg_write(client, reglist[index].reg,
				     reglist[index].val);
		/*  Check for Pause condition */
		if ((reglist[index + 1].reg == 0xFFFF)
			&& (reglist[index + 1].val != 0)) {
			msleep(reglist[index + 1].val);
			index += 1;
		}
	}
	return 0;
}

#ifdef OV5640_DEBUG
static int ov5640_reglist_compare(struct i2c_client *client,
			const struct ov5640_reg reglist[])
{
	int err = 0, index;
	u8 reg;

	for (index = 0; ((reglist[index].reg != 0xFFFF) && (err == 0));
								index++) {
		err |=
			ov5640_reg_read(client, reglist[index].reg,
				&reg);
		if (reglist[index].val != reg) {
			iprintk("reg err:reg=0x%x val=0x%x rd=0x%x  \n",
				reglist[index].reg, reglist[index].val, reg);
		}
		/*  Check for Pause condition */
		if ((reglist[index + 1].reg == 0xFFFF)
			&& (reglist[index + 1].val != 0)) {
			msleep(reglist[index + 1].val);
			index += 1;
		}
	}
	return 0;
}
#endif
static int ov5640_array_write(struct i2c_client *client,
					const u8 *data, u16 size)
{
	int ret;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = size,
		.buf = (u8 *)data,
	};

//	ret = i2c_transfer(client->adapter, &msg, 1);
	return 0;
}

static int ov5640_af_ack(struct i2c_client *client, int num_trys)
{
	int ret = 0;
	u8 af_ack = 0;
	int i;

	for (i = 0; i < num_trys; i++) {
		ov5640_reg_read(client, OV5640_CMD_ACK, &af_ack);
		if (af_ack == 0)
			break;
		msleep(50);
	}
	if (af_ack != 0) {
		return OV5640_AF_FAIL;
	}
	return ret;
}

static int ov5640_af_fw_status(struct i2c_client *client)
{
	u8 af_st = 0;

	ov5640_reg_read(client, OV5640_CMD_FW_STATUS, &af_st);

	iprintk("status=0x%x  \n", af_st);
	return (int)af_st;
}

static int ov5640_af_enable(struct i2c_client *client)
{
	int ret = 0;
#if 0
	ret = ov5640_reg_writes(client, ov5640_afpreinit_tbl);
	if (ret)
		return ret;

	ret = ov5640_array_write(client, ov5640_afinit_data,
		sizeof(ov5640_afinit_data)
		/sizeof(ov5640_afinit_data[0]));
	if (ret)
		return ret;

	ret = ov5640_reg_writes(client, ov5640_afpostinit_tbl);
	if (ret)
		return ret;

	ov5640_af_fw_status(client);
#endif
	return ret;
}

static int ov5640_af_release(struct i2c_client *client)
{
	int ret = 0;

	ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
	if (ret)
		return ret;
	ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x08);
	if (ret)
		return ret;
	ov5640_af_fw_status(client);
	return ret;
}

static int ov5640_af_center(struct i2c_client *client)
{
	int ret = 0;

	ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
	if (ret)
		return ret;
	ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x80);
	if (ret)
		return ret;
	ret = ov5640_af_ack(client, 50);
	if (ret) {
		return OV5640_AF_FAIL;
	}

	return ret;
}

static int ov5640_af_macro(struct i2c_client *client)
{
	int ret = 0;
	u8 reg;

	ret = ov5640_af_release(client);
	if (ret)
		return ret;
	/* move VCM all way out */
	ret = ov5640_reg_read(client, 0x3603, &reg);
	if (ret)
		return ret;
	reg &= ~(0x3f);
	ret = ov5640_reg_write(client, 0x3603, reg);
	if (ret)
		return ret;

	ret = ov5640_reg_read(client, 0x3602, &reg);
	if (ret)
		return ret;
	reg &= ~(0xf0);
	ret = ov5640_reg_write(client, 0x3602, reg);
	if (ret)
		return ret;

	/* set direct mode */
	ret = ov5640_reg_read(client, 0x3602, &reg);
	if (ret)
		return ret;
	reg &= ~(0x07);
	ret = ov5640_reg_write(client, 0x3602, reg);
	if (ret)
		return ret;

	return ret;
}

static int ov5640_af_infinity(struct i2c_client *client)
{
	int ret = 0;
	u8 reg;

	ret = ov5640_af_release(client);
	if (ret)
		return ret;
	/* move VCM all way in */
	ret = ov5640_reg_read(client, 0x3603, &reg);
	if (ret)
		return ret;
	reg |= 0x3f;
	ret = ov5640_reg_write(client, 0x3603, reg);
	if (ret)
		return ret;

	ret = ov5640_reg_read(client, 0x3602, &reg);
	if (ret)
		return ret;
	reg |= 0xf0;
	ret = ov5640_reg_write(client, 0x3602, reg);
	if (ret)
		return ret;

	/* set direct mode */
	ret = ov5640_reg_read(client, 0x3602, &reg);
	if (ret)
		return ret;
	reg &= ~(0x07);
	ret = ov5640_reg_write(client, 0x3602, reg);
	if (ret)
		return ret;

	return ret;
}

/* Set the touch area x,y in VVF coordinates*/
static int ov5640_af_touch(struct i2c_client *client)
{
	int ret = OV5640_AF_SUCCESS;
	struct ov5640 *ov5640 = to_ov5640(client);

	/* verify # zones correct */
	if (ov5640->touch_focus) {

		/* touch zone config */
		ret = ov5640_reg_write(client, 0x3024,
				       (u8) ov5640->touch_area[0].leftTopX);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x3025,
				       (u8) ov5640->touch_area[0].leftTopY);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x81);
		if (ret)
			return ret;
		ret = ov5640_af_ack(client, 50);
		if (ret) {
			return ret;
		}

	}

	return ret;
}

/* Set the touch area, areas can overlap and
are givin in current sensor resolution coords */
static int ov5640_af_area(struct i2c_client *client)
{
	int ret = OV5640_AF_SUCCESS;
	struct ov5640 *ov5640 = to_ov5640(client);
	u8 weight[OV5640_MAX_FOCUS_AREAS];
	int i;

	/* verify # zones correct */
	if ((ov5640->touch_focus) &&
			(ov5640->touch_focus <= OV5640_MAX_FOCUS_AREAS)) {

		/* enable zone config */
		ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x8f);
		if (ret)
			return ret;
		ret = ov5640_af_ack(client, 50);
		if (ret) {
			return ret;
		}

		/* clear all zones */
		for (i = 0; i < OV5640_MAX_FOCUS_AREAS; i++)
			weight[i] = 0;

		/* write area to sensor */
		for (i = 0; i < ov5640->touch_focus; i++) {

			ret = ov5640_reg_write(client, 0x3024,
					       (u8) ov5640->
					       touch_area[i].leftTopX);
			if (ret)
				return ret;
			ret = ov5640_reg_write(client, 0x3025,
					       (u8) ov5640->
					       touch_area[i].leftTopY);
			if (ret)
				return ret;
			ret = ov5640_reg_write(client, 0x3026,
					       (u8) (ov5640->
						     touch_area[i].leftTopX +
						     ov5640->
						     touch_area
						     [i].rightBottomX));
			if (ret)
				return ret;
			ret = ov5640_reg_write(client, 0x3027,
					       (u8) (ov5640->
						     touch_area[i].leftTopY +
						     ov5640->
						     touch_area
						     [i].rightBottomY));
			if (ret)
				return ret;
			ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
			if (ret)
				return ret;
			ret = ov5640_reg_write(client, OV5640_CMD_MAIN,
						(0x90+i));
			if (ret)
				return ret;
			ret = ov5640_af_ack(client, 50);
			if (ret) {
				return ret;
			}
			weight[i] = (u8)ov5640->touch_area[i].weight;
		}

		/* enable zone with weight */
		ret = ov5640_reg_write(client, 0x3024, weight[0]);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x3025, weight[1]);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x3026, weight[2]);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x3027, weight[3]);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x3028, weight[4]);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x98);
		if (ret)
			return ret;
		ret = ov5640_af_ack(client, 50);
		if (ret) {
			return ret;
		}

		/* launch zone configuration */
		ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x9f);
		if (ret)
			return ret;
		ret = ov5640_af_ack(client, 50);
		if (ret) {
			return ret;
		}
	}


	return ret;
}


/* Convert touch area from sensor resolution coords to ov5640 VVF zone */
static int ov5640_af_zone_conv(struct i2c_client *client,
					v4l2_touch_area *zone_area,
					int zone)
{
	int ret = 0;
	u32 x0, y0, x1, y1, weight;
	struct ov5640 *ov5640 = to_ov5640(client);

	/* Reset zone */
	ov5640->touch_area[zone].leftTopX = 0;
	ov5640->touch_area[zone].leftTopY = 0;
	ov5640->touch_area[zone].rightBottomX = 0;
	ov5640->touch_area[zone].rightBottomY = 0;
	ov5640->touch_area[zone].weight = 0;

	/* x y w h are in current sensor resolution dimensions */
	if (((u32) zone_area->leftTopX + (u32) zone_area->rightBottomX)
				> ov5640_frmsizes[ov5640->i_size].width) {

		ret = -EINVAL;
		goto out;
	} else if (((u32) zone_area->leftTopY + (u32) zone_area->rightBottomY)
				> ov5640_frmsizes[ov5640->i_size].height) {

		ret = -EINVAL;
		goto out;
	} else if ((u32) zone_area->weight > 1000) {

		ret = -EINVAL;
		goto out;
	}

	/* conv area to sensor VVF zone */
	x0 = (u32) zone_area->leftTopX / af_zone_scale[ov5640->i_size].x_scale;
	if (x0 > (OV5640_AF_NORMALIZED_W - 8))
		x0 = (OV5640_AF_NORMALIZED_W - 8);
	x1 = ((u32) zone_area->leftTopX + (unsigned int)zone_area->rightBottomX)
		/ af_zone_scale[ov5640->i_size].x_scale;
	if (x1 > OV5640_AF_NORMALIZED_W)
		x1 = OV5640_AF_NORMALIZED_W;
	y0 = (u32) zone_area->leftTopY / af_zone_scale[ov5640->i_size].y_scale;
	if (y0 > (OV5640_AF_NORMALIZED_H - 8))
		y0 = (OV5640_AF_NORMALIZED_H - 8);
	y1 = ((u32) zone_area->leftTopY + (unsigned int)zone_area->rightBottomY)
		/ af_zone_scale[ov5640->i_size].y_scale;
	if (y1 > OV5640_AF_NORMALIZED_H)
		y1 = OV5640_AF_NORMALIZED_H;

	/* weight ranges from 1-1000 */
	/* Convert weight */
	weight = 0;
	if ((zone_area->weight > 0) && (zone_area->weight <= 125))
		weight = 1;
	else if ((zone_area->weight > 125) && (zone_area->weight <= 250))
		weight = 2;
	else if ((zone_area->weight > 250) && (zone_area->weight <= 375))
		weight = 3;
	else if ((zone_area->weight > 375) && (zone_area->weight <= 500))
		weight = 4;
	else if ((zone_area->weight > 500) && (zone_area->weight <= 625))
		weight = 5;
	else if ((zone_area->weight > 625) && (zone_area->weight <= 750))
		weight = 6;
	else if ((zone_area->weight > 750) && (zone_area->weight <= 875))
		weight = 7;
	else if (zone_area->weight > 875)
		weight = 8;

	/* Minimum zone size */
	if (((x1 - x0) >= 8) && ((y1 - y0) >= 8)) {

		ov5640->touch_area[zone].leftTopX = (int)x0;
		ov5640->touch_area[zone].leftTopY = (int)y0;
		ov5640->touch_area[zone].rightBottomX = (int)(x1 - x0);
		ov5640->touch_area[zone].rightBottomY = (int)(y1 - y0);
		ov5640->touch_area[zone].weight = (int) weight;

	} else {
		ret = -EINVAL;
		goto out;
	}

out:

	return ret;
}

static int ov5640_af_status(struct i2c_client *client, int num_trys)
{
	int ret = OV5640_AF_SUCCESS;
	struct ov5640 *ov5640 = to_ov5640(client);
	int af_st = 0;
	u8 af_zone0, af_zone1, af_zone2, af_zone3, af_zone4;

	if (ov5640->focus_mode == FOCUS_MODE_AUTO) {

		/* Wait for Focus Command Ack */
		ret = ov5640_af_ack(client, num_trys);
		if (ret) {
			ret = OV5640_AF_FAIL;
			goto out;
		}
		/* Check if Focused */
		af_st = ov5640_af_fw_status(client);
		if (af_st != 0x10) {
			ret = OV5640_AF_PENDING;
			goto out;
		}

		/* Check if Zones Focused */
		ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		ov5640_reg_write(client, OV5640_CMD_MAIN, 0x07);

		ret = ov5640_af_ack(client, num_trys);
		if (ret) {
			ret = OV5640_AF_FAIL;
			goto out;
		}

		ov5640_reg_read(client, 0x3024, &af_zone0);
		ov5640_reg_read(client, 0x3025, &af_zone1);
		ov5640_reg_read(client, 0x3026, &af_zone2);
		ov5640_reg_read(client, 0x3027, &af_zone3);
		ov5640_reg_read(client, 0x3028, &af_zone4);
		if ((af_zone0 != 0) && (af_zone1 != 0) && (af_zone2 != 0)
				&& (af_zone3 != 0) && (af_zone4 != 0)) {
			ret = OV5640_AF_FAIL;
			goto out;
		}

	}

out:
	return ret;
}

static int ov5640_af_start(struct i2c_client *client)
{
	int ret = 0;
	struct ov5640 *ov5640 = to_ov5640(client);

	if (ov5640->focus_mode == FOCUS_MODE_MACRO) {
		/*
		 * FIXME: Can the af_area be set before af_macro, or does
		 * this need to be inside the af_macro func?
		 ret = ov5640_af_area(client);
		 */
		ret = ov5640_af_macro(client);
	} else if (ov5640->focus_mode == FOCUS_MODE_INFINITY)
		ret = ov5640_af_infinity(client);
	else {
		if (ov5640->touch_focus) {
			if (ov5640->touch_focus == 1)
				ret = ov5640_af_touch(client);
			else
				ret = ov5640_af_area(client);
		} else
			ret = ov5640_af_center(client);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_ACK, 0x01);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, OV5640_CMD_MAIN, 0x03);
		if (ret)
			return ret;
	}

	return ret;
}

static int ov5640_config_timing(struct i2c_client *client)
{
	struct ov5640 *ov5640 = to_ov5640(client);
	int ret, i = ov5640->i_size;
	const struct ov5640_timing_cfg *timing_cfg;

	if (ov5640_fmts[ov5640->i_fmt].code == V4L2_MBUS_FMT_JPEG_1X8)
		timing_cfg = &timing_cfg_jpeg[i];
	else
		timing_cfg = &timing_cfg_yuv[i];

	ret = ov5640_reg_write(client,
				0x3800,
				(timing_cfg->x_addr_start & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3801, timing_cfg->x_addr_start & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3802,
				(timing_cfg->y_addr_start & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3803, timing_cfg->y_addr_start & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3804,
				(timing_cfg->x_addr_end & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client, 0x3805, timing_cfg->x_addr_end & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3806,
				(timing_cfg->y_addr_end & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client, 0x3807, timing_cfg->y_addr_end & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3808,
				(timing_cfg->h_output_size & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3809, timing_cfg->h_output_size & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380A,
				(timing_cfg->v_output_size & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380B, timing_cfg->v_output_size & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380C,
				(timing_cfg->h_total_size & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380D, timing_cfg->h_total_size & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380E,
				(timing_cfg->v_total_size & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x380F, timing_cfg->v_total_size & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3810,
				(timing_cfg->isp_h_offset & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3811, timing_cfg->isp_h_offset & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3812,
				(timing_cfg->isp_v_offset & 0xFF00) >> 8);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
			       0x3813, timing_cfg->isp_v_offset & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3814,
				((timing_cfg->h_odd_ss_inc & 0xF) << 4) |
				(timing_cfg->h_even_ss_inc & 0xF));
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3815,
				((timing_cfg->v_odd_ss_inc & 0xF) << 4) |
				(timing_cfg->v_even_ss_inc & 0xF));
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3821, timing_cfg->out_mode_sel & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client,
				0x3108, timing_cfg->sclk_dividers & 0xFF);
	if (ret)
		return ret;

	ret = ov5640_reg_write(client, 0x3035, timing_cfg->sys_mipi_clk & 0xFF);
	if (ret)
		return ret;

	return ret;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);
	int ret = 0;
static int i =0;
	if (enable)	{
		i++;
	    ret = smb_write(configscript_common0);
	}
	else{
		ret = ov5640_reg_writes(client, ov5640_power_down);
		ov5640->flashmode = FLASH_MODE_OFF;		
	}

	return ret;
}

static int ov5640_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);

	mf->width = ov5640_frmsizes[ov5640->i_size].width;
	mf->height = ov5640_frmsizes[ov5640->i_size].height;
	mf->code = ov5640_fmts[ov5640->i_fmt].code;
	mf->colorspace = ov5640_fmts[ov5640->i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ov5640_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int i_fmt;
	int i_size;

	i_fmt = ov5640_find_datafmt(mf->code);

	mf->code = ov5640_fmts[i_fmt].code;
	mf->colorspace = ov5640_fmts[i_fmt].colorspace;
	mf->field = V4L2_FIELD_NONE;

	i_size = ov5640_find_framesize(mf->width, mf->height);

	mf->width = ov5640_frmsizes[i_size].width;
	mf->height = ov5640_frmsizes[i_size].height;

	return 0;
}

static int ov5640_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);
	int ret = 0;

	ret = ov5640_try_fmt(sd, mf);
	if (ret < 0)
		return ret;

	ov5640->i_size = ov5640_find_framesize(mf->width, mf->height);
	ov5640->i_fmt = ov5640_find_datafmt(mf->code);

	switch ((u32) ov5640_fmts[ov5640->i_fmt].code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		ret = ov5640_reg_writes(client, yuv422_init_common);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x4300, 0x32);
		if (ret)
			return ret;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		ret = ov5640_reg_writes(client, yuv422_init_common);
		if (ret)
			return ret;
		ret = ov5640_reg_write(client, 0x4300, 0x30);
		if (ret)
			return ret;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		ret = ov5640_reg_writes(client, jpeg_init_common);
		if (ret)
			return ret;
		break;
	default:
		/* This shouldn't happen */
		ret = -EINVAL;
		return ret;
	}

	ret = ov5640_config_timing(client);
	if (ret)
		return ret;

	return ret;
}

static int ov5640_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident = V4L2_IDENT_OV5640;
	id->revision = 0;

	return 0;
}

/*
 * return value of this function should be
 * 0 == failed
 * 1 == done
 * 2 == canceled
 */
static int ov5640_get_af_status(struct i2c_client *client, int num_trys)
{
	int ret = 0;
	struct ov5640 *ov5640 = to_ov5640(client);

	if (atomic_read(&ov5640->focus_status)
			== OV5640_FOCUSING) {
		ret = ov5640_af_status(client, num_trys);
		if (ret == 0) {
			ret = 1;
			goto out;
		}
	}
	if (atomic_read(&ov5640->focus_status)
			== OV5640_NOT_FOCUSING) {
		ret = 2;
		goto out;
	}

	ret = 0;
out:
	atomic_set(&ov5640->focus_status, OV5640_NOT_FOCUSING);

	return ret;
}

static int ov5640_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_BRIGHTNESS:
		ctrl->value = ov5640->brightness;
		break;
	case V4L2_CID_CAMERA_CONTRAST:
		ctrl->value = ov5640->contrast;
		break;
	case V4L2_CID_CAMERA_EFFECT:
		ctrl->value = ov5640->colorlevel;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = ov5640->saturation;
		break;
	case V4L2_CID_SHARPNESS:
		ctrl->value = ov5640->sharpness;
		break;
	case V4L2_CID_CAMERA_ANTI_BANDING:
		ctrl->value = ov5640->antibanding;
		break;
	case V4L2_CID_CAMERA_WHITE_BALANCE:
		ctrl->value = ov5640->whitebalance;
		break;
	case V4L2_CID_CAMERA_FRAME_RATE:
		ctrl->value = ov5640->framerate;
		break;
	case V4L2_CID_CAMERA_FOCUS_MODE:
		ctrl->value = ov5640->focus_mode;
		break;
	case V4L2_CID_CAMERA_TOUCH_AF_AREA:
		ctrl->value = ov5640->touch_focus;
		break;
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		/*
		 * this is called from another thread to read AF status
		 */
		ctrl->value = ov5640_get_af_status(client, 100);
		ov5640->touch_focus = 0;
		break;
	case V4L2_CID_CAMERA_FLASH_MODE:
		ctrl->value = ov5640->flashmode;
		break;
	}

	return 0;
}

static int ov5640_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);
	u8 ov_reg;
	int ret = 0;

	dev_dbg(&client->dev, "ov5640_s_ctrl\n");
return 0;
	switch (ctrl->id) {
	case V4L2_CID_CAMERA_BRIGHTNESS:

		if (ctrl->value > EV_PLUS_1)
			return -EINVAL;

		ov5640->brightness = ctrl->value;
		switch (ov5640->brightness) {
		case EV_MINUS_1:
			ret = ov5640_reg_writes(client,
					ov5640_brightness_lv4_tbl);
			break;
		case EV_PLUS_1:
			ret = ov5640_reg_writes(client,
					ov5640_brightness_lv0_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_brightness_lv2_default_tbl);
			break;
		}
		if (ret)
			return ret;
		break;
	case V4L2_CID_CAMERA_CONTRAST:

		if (ctrl->value > CONTRAST_PLUS_1)
			return -EINVAL;

		ov5640->contrast = ctrl->value;
		switch (ov5640->contrast) {
		case CONTRAST_MINUS_1:
			ret = ov5640_reg_writes(client,
					ov5640_contrast_lv5_tbl);
			break;
		case CONTRAST_PLUS_1:
			ret = ov5640_reg_writes(client,
					ov5640_contrast_lv0_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_contrast_default_lv3_tbl);
			break;
		}
		if (ret)
			return ret;
		break;
	case V4L2_CID_CAMERA_EFFECT:

		if (ctrl->value > IMAGE_EFFECT_BNW)
			return -EINVAL;

		ov5640->colorlevel = ctrl->value;

		switch (ov5640->colorlevel) {
		case IMAGE_EFFECT_BNW:
			ret = ov5640_reg_writes(client,
					ov5640_effect_bw_tbl);
			break;
		case IMAGE_EFFECT_SEPIA:
			ret = ov5640_reg_writes(client,
					ov5640_effect_sepia_tbl);
			break;
		case IMAGE_EFFECT_NEGATIVE:
			ret = ov5640_reg_writes(client,
					ov5640_effect_negative_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_effect_normal_tbl);
			break;
		}
		if (ret)
			return ret;
		break;
	case V4L2_CID_SATURATION:

		if (ctrl->value > OV5640_SATURATION_MAX)
			return -EINVAL;

		ov5640->saturation = ctrl->value;
		switch (ov5640->saturation) {
		case OV5640_SATURATION_MIN:
			ret = ov5640_reg_writes(client,
					ov5640_saturation_lv0_tbl);
			break;
		case OV5640_SATURATION_MAX:
			ret = ov5640_reg_writes(client,
					ov5640_saturation_lv5_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_saturation_default_lv3_tbl);
			break;
		}
		if (ret)
			return ret;
		break;
	case V4L2_CID_SHARPNESS:

		if (ctrl->value > OV5640_SHARPNESS_MAX)
			return -EINVAL;

		ov5640->sharpness = ctrl->value;
		switch (ov5640->sharpness) {
		case OV5640_SHARPNESS_MIN:
			ret = ov5640_reg_writes(client,
					ov5640_sharpness_lv0_tbl);
			break;
		case OV5640_SHARPNESS_MAX:
			ret = ov5640_reg_writes(client,
					ov5640_sharpness_lv3_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_sharpness_default_lv2_tbl);
			break;
		}
		if (ret)
			return ret;
		break;

	case V4L2_CID_CAMERA_ANTI_BANDING:

		if (ctrl->value > ANTI_BANDING_60HZ)
			return -EINVAL;

		ov5640->antibanding = ctrl->value;

		switch (ov5640->antibanding) {
		case ANTI_BANDING_50HZ:
			ret = ov5640_reg_writes(client,
					ov5640_antibanding_50z_tbl);
			break;
		case ANTI_BANDING_60HZ:
			ret = ov5640_reg_writes(client,
					ov5640_antibanding_60z_tbl);
			break;
		default:
			ret = ov5640_reg_writes(client,
					ov5640_antibanding_auto_tbl);
			break;
		}
		if (ret)
			return ret;
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:

		if (ctrl->value > WHITE_BALANCE_FLUORESCENT)
			return -EINVAL;

		ov5640->whitebalance = ctrl->value;

		ret = ov5640_reg_read(client, 0x3406, &ov_reg);
		if (ret)
			return ret;

		switch (ov5640->whitebalance) {
		case WHITE_BALANCE_FLUORESCENT:
			ov_reg |= 0x01;
			ret = ov5640_reg_write(client,
					0x3406, ov_reg);
			ret = ov5640_reg_writes(client,
					ov5640_wb_fluorescent);
			break;
		case WHITE_BALANCE_SUNNY:
			ov_reg |= 0x01;
			ret = ov5640_reg_write(client,
					0x3406, ov_reg);
			ret = ov5640_reg_writes(client,
					ov5640_wb_daylight);
			break;
		case WHITE_BALANCE_CLOUDY:
			ov_reg |= 0x01;
			ret = ov5640_reg_write(client,
					0x3406, ov_reg);
			ret = ov5640_reg_writes(client,
				ov5640_wb_cloudy);
			break;
		case WHITE_BALANCE_TUNGSTEN:
			ov_reg |= 0x01;
			ret = ov5640_reg_write(client,
					0x3406, ov_reg);
			ret = ov5640_reg_writes(client,
					ov5640_wb_tungsten);
			break;
		default:
			ov_reg &= ~(0x01);
			ret = ov5640_reg_write(client,
					0x3406, ov_reg);
			ret = ov5640_reg_writes(client,
						ov5640_wb_def);
			break;
		}
		if (ret) {
			return ret;
		}
		break;

	case V4L2_CID_CAMERA_FRAME_RATE:

		if (ctrl->value > FRAME_RATE_30)
			return -EINVAL;

		if ((ov5640->i_size < OV5640_SIZE_QVGA) ||
				(ov5640->i_size > OV5640_SIZE_1280x960)) {
			if (ctrl->value == FRAME_RATE_30 ||
					ctrl->value == FRAME_RATE_AUTO)
				return 0;
			else
				return -EINVAL;
		}

		ov5640->framerate = ctrl->value;

		switch (ov5640->framerate) {
		case FRAME_RATE_5:
			ret = ov5640_reg_writes(client,
					ov5640_fps_5);
			break;
		case FRAME_RATE_7:
			ret = ov5640_reg_writes(client,
					ov5640_fps_7);
			break;
		case FRAME_RATE_10:
			ret = ov5640_reg_writes(client,
					ov5640_fps_10);
			break;
		case FRAME_RATE_15:
			ret = ov5640_reg_writes(client,
					ov5640_fps_15);
			break;
		case FRAME_RATE_20:
			ret = ov5640_reg_writes(client,
					ov5640_fps_20);
			break;
		case FRAME_RATE_25:
			ret = ov5640_reg_writes(client,
					ov5640_fps_25);
			break;
		case FRAME_RATE_30:
		case FRAME_RATE_AUTO:
		default:
			ret = ov5640_reg_writes(client,
					ov5640_fps_30);
			break;
		}
		if (ret)
			return ret;
		break;
#if 0
	case V4L2_CID_CAMERA_FOCUS_MODE:

		if (ctrl->value > FOCUS_MODE_INFINITY)
			return -EINVAL;

		ov5640->focus_mode = ctrl->value;

		/*
		 * Donot start the AF cycle here
		 * AF Start will be called later in
		 * V4L2_CID_CAMERA_SET_AUTO_FOCUS only for auto, macro mode
		 * it wont be called for infinity.
		 * Donot worry about resolution change for now.
		 * From userspace we set the resolution first
		 * and then set the focus mode.
		 */
		switch (ov5640->focus_mode) {
		case FOCUS_MODE_MACRO:
			/*
			 * set the table for macro mode
			 */
			break;
		case FOCUS_MODE_INFINITY:
			/*
			 * set the table for infinity
			 */
			ret = 0;
			break;
		default:
			ret = ov5640_af_enable(client);
			break;
		}

		if (ret)
			return ret;
		break;
#endif
	case V4L2_CID_CAMERA_TOUCH_AF_AREA:

		if (ov5640->touch_focus < OV5640_MAX_FOCUS_AREAS) {
			v4l2_touch_area touch_area;
			if (copy_from_user(&touch_area,
					(v4l2_touch_area *)ctrl->value,
					sizeof(v4l2_touch_area)))
				return -EINVAL;

			iprintk("z=%d x=0x%x y=0x%x w=0x%x h=0x%x weight=0x%x  \n",
				ov5640->touch_focus, touch_area.leftTopX,
				touch_area.leftTopY, touch_area.rightBottomX,
				touch_area.rightBottomY, touch_area.weight);

			ret = ov5640_af_zone_conv(client, &touch_area,
							ov5640->touch_focus);
			if (ret == 0)
				ov5640->touch_focus++;
			ret = 0;

		} else
			dev_dbg(&client->dev,
				"Maximum touch focus areas already set\n");

		break;

	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:

		if (ctrl->value > AUTO_FOCUS_ON)
			return -EINVAL;

		/* start and stop af cycle here */
		switch (ctrl->value) {

		case AUTO_FOCUS_OFF:

			if (atomic_read(&ov5640->focus_status)
					== OV5640_FOCUSING) {
				ret = ov5640_af_release(client);
				atomic_set(&ov5640->focus_status,
						OV5640_NOT_FOCUSING);
			}
			ov5640->touch_focus = 0;
			break;

		case AUTO_FOCUS_ON:
			ret = ov5640_af_enable(client);
			if (ret)
				return ret;
			ret = ov5640_af_start(client);
			atomic_set(&ov5640->focus_status, OV5640_FOCUSING);
			break;

		}

		if (ret)
			return ret;
		break;

	}

	return ret;
}

static long ov5640_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;

	switch (cmd) {
	case VIDIOC_THUMB_SUPPORTED:
		{
			int *p = arg;
			*p = 0;	/* no we don't support thumbnail */
			break;
		}
	case VIDIOC_JPEG_G_PACKET_INFO:
		{
			struct v4l2_jpeg_packet_info *p =
				(struct v4l2_jpeg_packet_info *)arg;
			p->padded = 0;
			p->packet_size = 0x400;
			break;
		}

	case VIDIOC_SENSOR_G_OPTICAL_INFO:
		{
			struct v4l2_sensor_optical_info *p =
				(struct v4l2_sensor_optical_info *)arg;
			/* assuming 67.5 degree diagonal viewing angle */
			p->hor_angle.numerator = 5401;
			p->hor_angle.denominator = 100;
			p->ver_angle.numerator = 3608;
			p->ver_angle.denominator = 100;
			p->focus_distance[0] = 10; /* near focus in cm */
			p->focus_distance[1] = 100; /* optimal focus in cm */
			p->focus_distance[2] = -1; /* infinity */
			p->focal_length.numerator = 342;
			p->focal_length.denominator = 100;
			break;
		}
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5640_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return 0;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->size > 2)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 2;
	if (ov5640_reg_read(client, reg->reg, &reg->val))
		return -EIO;

	return 0;
}

static int ov5640_s_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return 0;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->size > 2)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (ov5640_reg_write(client, reg->reg, reg->val))
		return -EIO;

	return 0;
}
#endif

static int ov5640_init(struct i2c_client *client)
{
	struct ov5640 *ov5640 = to_ov5640(client);
	int ret = 0;
	
#if defined(CONFIG_RHEA_CLOVER_ICS)
	/*Code turn off flash led */
	if (ov5640_reg_write(client, 0x3000, 0x00))
		goto out;
	if (ov5640_reg_write(client, 0x3004, 0xFF))
		goto out;
	if (ov5640_reg_write(client, 0x3016, 0x02))
		goto out;
	if (ov5640_reg_write(client, 0x3b07, 0x0A))
		goto out;
	if (ov5640_reg_write(client, 0x3b00, 0x03))
		goto out;
#endif

	/* Power Up, Start Streaming for AF Init*/
	ret = ov5640_reg_writes(client, ov5640_stream);
	if (ret)
		goto out;
	/* Delay for sensor streaming*/
	msleep(20);

	/* AF Init*/
	ret = ov5640_af_enable(client);
	if (ret)
		goto out;

	/* Stop Streaming, Power Down*/
	ret = ov5640_reg_writes(client, ov5640_power_down);

	/* default brightness and contrast */
	ov5640->brightness = EV_DEFAULT;
	ov5640->contrast = CONTRAST_DEFAULT;
	ov5640->colorlevel = IMAGE_EFFECT_NONE;
	ov5640->antibanding = ANTI_BANDING_AUTO;
	ov5640->whitebalance = WHITE_BALANCE_AUTO;
	ov5640->framerate = FRAME_RATE_AUTO;
	ov5640->focus_mode = FOCUS_MODE_AUTO;
	ov5640->touch_focus = 0;
	atomic_set(&ov5640->focus_status, OV5640_NOT_FOCUSING);
	ov5640->flashmode = FLASH_MODE_OFF;

	dev_dbg(&client->dev, "Sensor initialized\n");

out:
	return ret;
}

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 *this wasn't our capture interface, so, we wait for the right one
 */
static int ov5640_video_probe(struct soc_camera_device *icd,
			      struct i2c_client *client)
{
	unsigned long flags;
	int ret = 0;
	u8 revision = 0;

	ret = ov5640_reg_read(client, 0x302A, &revision);	
	if (ret) {
		goto out;
	}
	revision &= 0xF;
	flags = SOCAM_DATAWIDTH_8;
	/* TODO: Do something like ov5640_init */

out:
	return ret;
}

static void ov5640_video_remove(struct soc_camera_device *icd)
{
	dev_dbg(&icd->pdev, "Video removed: %p, %p\n",
		icd->parent, icd->vdev);
}

static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.g_chip_ident = ov5640_g_chip_ident,
	.g_ctrl = ov5640_g_ctrl,
	.s_ctrl = ov5640_s_ctrl,
	.ioctl = ov5640_ioctl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5640_g_register,
	.s_register = ov5640_s_register,
#endif
};

static int ov5640_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5640_fmts))
		return -EINVAL;

	*code = ov5640_fmts[index].code;
	return 0;
}

static int ov5640_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index >= OV5640_SIZE_LAST)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->pixel_format = V4L2_PIX_FMT_UYVY;

	fsize->discrete = ov5640_frmsizes[fsize->index];

	return 0;
}

/* we only support fixed frame rate */
static int ov5640_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *interval)
{
	int size;

	if (interval->index >= 1)
		return -EINVAL;

	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;

	size = ov5640_find_framesize(interval->width, interval->height);

	switch (size) {
	case OV5640_SIZE_5MP:
		interval->discrete.numerator = 2;
		interval->discrete.denominator = 15;
		break;
	case OV5640_SIZE_QXGA:
	case OV5640_SIZE_UXGA:
		interval->discrete.numerator = 1;
		interval->discrete.denominator = 15;
		break;
	case OV5640_SIZE_VGA:
	case OV5640_SIZE_QVGA:
	case OV5640_SIZE_1280x960:
	case OV5640_SIZE_720P:
	default:
		interval->discrete.numerator = 1;
		interval->discrete.denominator = 24;
		break;
	}
/*	printk(KERN_ERR"%s: width=%d height=%d fi=%d/%d\n", __func__,
			interval->width,
			interval->height, interval->discrete.numerator,
			interval->discrete.denominator);
			*/
	return 0;
}

static int ov5640_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);
	struct v4l2_captureparm *cparm;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	cparm = &param->parm.capture;

	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;

	switch (ov5640->i_size) {
	case OV5640_SIZE_5MP:
		cparm->timeperframe.numerator = 2;
		cparm->timeperframe.denominator = 15;
		break;
	case OV5640_SIZE_QXGA:
	case OV5640_SIZE_UXGA:
		cparm->timeperframe.numerator = 1;
		cparm->timeperframe.denominator = 15;
		break;
	case OV5640_SIZE_1280x960:
	case OV5640_SIZE_720P:
	case OV5640_SIZE_VGA:
	case OV5640_SIZE_QVGA:
	default:
		cparm->timeperframe.numerator = 1;
		cparm->timeperframe.denominator = 24;
		break;
	}

	return 0;
}
static int ov5640_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	/*
	 * FIXME: This just enforces the hardcoded framerates until this is
	 *flexible enough.
	 */
	return ov5640_g_parm(sd, param);
}

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream = ov5640_s_stream,
	.s_mbus_fmt = ov5640_s_fmt,
	.g_mbus_fmt = ov5640_g_fmt,
	.try_mbus_fmt = ov5640_try_fmt,
	.enum_mbus_fmt = ov5640_enum_fmt,
	.enum_mbus_fsizes = ov5640_enum_framesizes,
	.enum_framesizes = ov5640_enum_framesizes,
	.enum_frameintervals = ov5640_enum_frameintervals,
	.g_parm = ov5640_g_parm,
	.s_parm = ov5640_s_parm,
};
static int ov5640_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	/* Quantity of initial bad frames to skip. Revisit. */
	*frames = 4;

	return 0;
}

static struct v4l2_subdev_sensor_ops ov5640_subdev_sensor_ops = {
	.g_skip_frames = ov5640_g_skip_frames,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core = &ov5640_subdev_core_ops,
	.video = &ov5640_subdev_video_ops,
	.sensor = &ov5640_subdev_sensor_ops,
};
static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov5640 *ov5640;
	struct soc_camera_device *icd  ;
	struct soc_camera_link *icl = client->dev.platform_data;
	int ret;

	sm_init();	
	icd = (struct soc_camera_device *)icl->priv;
//    icd = kmalloc(sizeof(struct soc_camera_device), GFP_KERNEL);
//	client->dev.platform_data = icd;
	if (!icd) {
		dev_err(&client->dev, "OV5640: missing soc-camera data!\n");
		return -EINVAL;
	}
//	icd->iface = 0;	
#if 0	
    icl= kmalloc(sizeof(struct soc_camera_link), GFP_KERNEL);
	icd->link = icl;
	icl->bus_id = 0;
	icl->i2c_adapter_id = 0;
	icl->priv = icd;
	icl->module_name = "unicam-camera";
#endif	
//	icd->iface = icl->bus_id;
//	icd->pdev = &client->dev; 
//	icd->parent = client->adapter;	
//	icd->control = client;
	if (!icl) {
		dev_err(&client->dev, "OV5640 driver needs platform data\n");
		return -EINVAL;
	}
	if (!icl->priv) {
		dev_err(&client->dev,
			"OV5640 driver needs i/f platform data\n");
		return -EINVAL;
	}
	ov5640 = kzalloc(sizeof(struct ov5640), GFP_KERNEL);
	if (!ov5640)
		return -ENOMEM;
	v4l2_i2c_subdev_init(&ov5640->subdev, client, &ov5640_subdev_ops);
	/* Second stage probe - when a capture adapter is there */
//	icd->ops = &ov5640_ops;
	ov5640->i_size = OV5640_SIZE_VGA;
	ov5640->i_fmt = 0;	/* First format in the list */
//	ov5640->plat_parms = icl->priv;
	ret = ov5640_video_probe(icd, client);	
	if (ret) {
		kfree(ov5640);
		return ret;
	}
	/* init the sensor here */
//	ret = ov5640_init(client);
	
	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640 *ov5640 = to_ov5640(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	ov5640_video_remove(icd);
	client->driver = NULL;
	kfree(ov5640);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{"ov5640", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		   .name = "iproc camera",	   	
		   .owner = THIS_MODULE,
		   },
	.probe = ov5640_probe,
	.remove = ov5640_remove,
	.id_table = ov5640_id,
};
static int __init ov5640_mod_init(void)
{
	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit ov5640_mod_exit(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_mod_init);
module_exit(ov5640_mod_exit);

MODULE_DESCRIPTION("OmniVision OV5640 Camera driver");
MODULE_AUTHOR("Sergio Aguirre <saaguirre@ti.com>");
MODULE_LICENSE("GPL v2");
