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

#include "iproc_cc_otp.h"
#include "bcm5301x_otp.h"


#include <linux/io.h>
#include <asm/memory.h>

static void * baseAddr;

/* Enabled for simulate write-dword and write-bit commands */
#define OTP_WRITE_SIM 0

#define R_REG(reg)       ioread32(baseAddr + (reg&0x0fff))
#if OTP_WRITE_SIM
#define W_REG(reg, val)        
#else
#define W_REG(reg, val)        iowrite32(val, baseAddr + (reg&0x0fff))
#endif

#define NS_OTP_INIT_CHK  \
    {\
         if (!baseAddr) { \
             return -1; \
		 } \
    }

#define OTP_ERR_VAL	0x0001
#define OTP_MSG_VAL	0x0002
#define OTP_DBG_VAL	0x0004
static u32 __maybe_unused otp_msg_level = OTP_ERR_VAL;

#if defined(BCMDBG) || defined(BCMDBG_ERR)
#define OTP_ERR(args)	do {if (otp_msg_level & OTP_ERR_VAL) printk args;} while (0)
#else
#define OTP_ERR(args)
#endif

#ifdef BCMDBG
#define OTP_MSG(args)	do {if (otp_msg_level & OTP_MSG_VAL) printk args;} while (0)
#define OTP_DBG(args)	do {if (otp_msg_level & OTP_DBG_VAL) printk args;} while (0)
#else
#define OTP_MSG(args)
#define OTP_DBG(args)
#endif

#define OTPP_TRIES	10000000	/* # of tries for OTPP */

#define DEFAULT_OTPCPU_CTRL0 0x00a00600

typedef struct {
	/* OTP section */
	u16		wsize;		/* Size of otp in words */
	u16		rows;		/* Geometry */
	u16		cols;		/* Geometry */
	u16		status;		/* otp status */
} bcm5301x_otpinfo_t;

static bcm5301x_otpinfo_t bcm5301x_otpinfo;

/*
 * ROM accessor to avoid struct in shdat
 */
static bcm5301x_otpinfo_t *
_get_otpinfo(void)
{
	return (bcm5301x_otpinfo_t *)&bcm5301x_otpinfo;
}

/* Initialization of bcmstrbuf structure */
static void
_binit(struct bcmstrbuf *b, char *buf, uint size)
{
	b->origsize = b->size = size;
	b->origbuf = b->buf = buf;
}

/* Buffer sprintf wrapper to guard against buffer overflow */
static int
_bprintf(struct bcmstrbuf *b, const char *fmt, ...)
{
	va_list ap;
	int r;

	va_start(ap, fmt);
	r = vsnprintf(b->buf, b->size, fmt, ap);

	/* Non Ansi C99 compliant returns -1,
	 * Ansi compliant return r >= b->size,
	 * bcmstdlib returns 0, handle all
	 */
	if ((r == -1) || (r >= (int)b->size) || (r == 0)) {
		b->size = 0;
	} else {
		b->size -= r;
		b->buf += r;
	}

	va_end(ap);

	return r;
}

/*
 * OTP Code
 *
 *   Exported functions:
 *	bcm5301x_otp_status()
 *	bcm5301x_otp_size()
 *	bcm5301x_otp_init()
 *	bcm5301x_otp_write_bit()
 *	bcm5301x_otp_read_word()
 *	bcm5301x_otp_write_word()
 *	bcm5301x_otp_dump()
 *
 *   internal functions:
 *	_otp_init()
 *
 */


#define	OTPWSIZE		32	/* word size */

int
bcm5301x_otp_status(void *oh)
{
	bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh;

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	return (int)(oi->status);
}

/* Return size in bytes */
int
bcm5301x_otp_size(void *oh)
{
	bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh;

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	return (int)oi->wsize * 4;
}

/*
 * OTP sizes for 40nm
 */
static int
bcm5301x_otp_otpsize_set_40nm(bcm5301x_otpinfo_t *oi, uint otpsz)
{
	/* Check for otp size */
	switch (otpsz) {
	case 1:	/* 64x32: 2048 bits */
		oi->rows = 64;
		oi->cols = 32;
		break;
	case 2:	/* 96x32: 3072 bits */
		oi->rows = 96;
		oi->cols = 32;
		break;
	case 3:	/* 128x32: 4096 bits */
		oi->rows = 128;
		oi->cols = 32;
		break;
	case 4:	/* 160x32: 5120 bits */
		oi->rows = 160;
		oi->cols = 32;
		break;
	case 5:	/* 192x32: 6144 bits */
		oi->rows = 192;
		oi->cols = 32;
		break;
	case 7:	/* 256x32: 8192 bits */
		oi->rows = 256;
		oi->cols = 32;
		break;
	default:
		/* Don't know the geometry */
		OTP_ERR(("%s: unknown OTP geometry\n", __FUNCTION__));
	}

	oi->wsize = (oi->cols * oi->rows)/OTPWSIZE; 
	return 0;
}

void *
bcm5301x_otp_init(void)
{
	bcm5301x_otpinfo_t *oi;
    /* char buf[1024]; */

	OTP_MSG(("%s: BCM5301x chip level OTP initialization\n", __FUNCTION__));

    if (baseAddr == NULL) {
        baseAddr = ioremap(0x1800c000,0x1000); /* DMU registers' base */
    }
    OTP_DBG(("CHIP OTP baseAddr=%x\n",(uint32_t)baseAddr));

    /* NS_OTP_INIT_CHK; */
	if (!baseAddr)
		return NULL;

	oi = _get_otpinfo();
	memset(oi, 0, sizeof(bcm5301x_otpinfo_t));

	/* 128x32: 4096 bits */
	bcm5301x_otp_otpsize_set_40nm(oi, 3);

	OTP_MSG(("%s: rows %u cols %u wsize %u\n", __FUNCTION__, oi->rows, oi->cols, oi->wsize));

	return (void *)oi;
}

int
bcm5301x_otp_exit(void)
{
	bcm5301x_otpinfo_t *oi;

	OTP_MSG(("%s: BCM5301x chip level OTP initialization\n", __FUNCTION__));

    if (baseAddr) {
		iounmap(baseAddr); 
		baseAddr = NULL;
    }

	oi = _get_otpinfo();
	memset(oi, 0, sizeof(bcm5301x_otpinfo_t));

	return 0;
}

static int
_check_cmd_done(void)
{
    uint k;
	u32 st;
	
    OTP_DBG(("_check_cmd_done()\n"));

	for (k = 0; k < OTPP_TRIES; k++) {
        st = R_REG(DMU_OTP_CPU_STS);
		if (st & OTPCPU_STS_CMD_DONE_MASK) {
		    break;
		}
	}
#if OTP_WRITE_SIM
#else	
	if (k >= OTPP_TRIES) {
		OTP_ERR(("\n%s: BUSY stuck: st=0x%x, count=%d\n", __FUNCTION__, st, k));
		return -1;
	}
#endif

	return 0;
}

static int
_issue_prog_dis(void)
{
    u32 ctrl1, ctrl0;
    u32 start;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;
	int rv;

	OTP_DBG(("OTP: PROG DIS\n"));
	W_REG(DMU_OTP_CPU_ADDR, 0);
	OTP_DBG(("OTP: PROG DIS: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, 0));

	start = (1 << OTPCPU_CTRL1_START_SHIFT) & OTPCPU_CTRL1_START_MASK;
	cmd = (OTPCPU_CMD_PROG_DIS << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (1 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (1 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (2 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;
	
	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG DIS: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
    ctrl0 = DEFAULT_OTPCPU_CTRL0;
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG DIS: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

	ctrl1 &= ~start;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG DIS: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG DIS: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	return 0;
}

static int
_issue_prog_en(void)
{
    u32 ctrl1, ctrl0;
    u32 start;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;
	int rv;

	OTP_DBG(("OTP: PROG EN\n"));

	start = (1 << OTPCPU_CTRL1_START_SHIFT) & OTPCPU_CTRL1_START_MASK;
	cmd = (OTPCPU_CMD_PROG_EN << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (1 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (1 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (2 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;

	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl0 = DEFAULT_OTPCPU_CTRL0;

    /* step1, bitsel = 0xf */
	W_REG(DMU_OTP_CPU_ADDR, 0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, 0));
	W_REG(DMU_OTP_CPU_BITSEL, 0xf);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_BITSEL, 0xf));
    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));
	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}
    ctrl1 &= ~start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

    /* step2, bitsel = 0x4 */
	W_REG(DMU_OTP_CPU_ADDR, 0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, 0));
	W_REG(DMU_OTP_CPU_BITSEL, 0x4);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_BITSEL, 0x4));
    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));
	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}
    ctrl1 &= ~start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

    /* step3, bitsel = 0x8 */
	W_REG(DMU_OTP_CPU_ADDR, 0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, 0));
	W_REG(DMU_OTP_CPU_BITSEL, 0x8);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_BITSEL, 0x8));
    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));
	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}
    ctrl1 &= ~start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

    /* step4, bitsel = 0xd */
	W_REG(DMU_OTP_CPU_ADDR, 0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, 0));
	W_REG(DMU_OTP_CPU_BITSEL, 0xd);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_BITSEL, 0xd));
    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));
	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}
    ctrl1 &= ~start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG EN: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	return 0;
}

static int
_check_prog_ok(void)
{
    uint k;
	u32 st;

    u32 ctrl1, ctrl0;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;

	OTP_DBG(("OTP: PROG OK\n"));

	cmd = (OTPCPU_CMD_PROG_EN << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (1 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (1 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (2 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;

	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl0 = DEFAULT_OTPCPU_CTRL0;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG OK: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG OK: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	
	for (k = 0; k < OTPP_TRIES; k++) {
        st = R_REG(DMU_OTP_CPU_STS);
		if (st & OTPCPU_STS_PROG_OK_MASK) {
		    break;
		}
	}
#if OTP_WRITE_SIM
	OTP_DBG(("PROG OK: pass\n"));
#else
	if (k >= OTPP_TRIES) {
		OTP_ERR(("\n%s: BUSY stuck: st=0x%x, count=%d\n", __FUNCTION__, st, k));
		return -1;
	}
#endif
	return 0;
}

static int
_issue_prog_bit(uint row, uint col)
{
    u32 ctrl1, ctrl0;
    u32 start;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;
	int rv;

	OTP_DBG(("OTP: PROG BIT\n"));

	W_REG(DMU_OTP_CPU_ADDR, row);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, row));
	W_REG(DMU_OTP_CPU_BITSEL, col);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_BITSEL, col));

	start = (1 << OTPCPU_CTRL1_START_SHIFT) & OTPCPU_CTRL1_START_MASK;
	cmd = (OTPCPU_CMD_BIT_PROG << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (1 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (1 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (2 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;
	
	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
    ctrl0 = DEFAULT_OTPCPU_CTRL0;
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

	ctrl1 &= ~start;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG BIT: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	return 0;
}

static int
_issue_read(uint row, u32 *val)
{
    u32 ctrl1, ctrl0;
    u32 start;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;
	int rv;

	OTP_DBG(("OTP: READ row 0x%x\n", row));

	W_REG(DMU_OTP_CPU_ADDR, row);
	OTP_DBG(("OTP: READ row: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, row));

	start = (1 << OTPCPU_CTRL1_START_SHIFT) & OTPCPU_CTRL1_START_MASK;
	cmd = (OTPCPU_CMD_READ << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (2 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (0 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (1 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;
	
	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: READ row: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
    ctrl0 = DEFAULT_OTPCPU_CTRL0;
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: READ row: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

    *val = R_REG(DMU_OTP_CPU_READ_DATA);
	OTP_DBG(("OTP: READ row: read reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_READ_DATA, *val));

	return 0;
}

static int
_issue_prog_word(uint row, u32 data)
{
    u32 ctrl1, ctrl0;
    u32 start;
	u32 cmd;
	u32 fuse;
	u32 cof;
	u32 prog_en;
	u32 mode;
	int rv;

	OTP_DBG(("OTP: PROG WORD\n"));

	W_REG(DMU_OTP_CPU_ADDR, row);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_ADDR, row));

	W_REG(DMU_OTP_CPU_WRITE_DATA, data);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_WRITE_DATA, data));

	start = (1 << OTPCPU_CTRL1_START_SHIFT) & OTPCPU_CTRL1_START_MASK;
	cmd = (OTPCPU_CMD_WORD_PROG << OTPCPU_CTRL1_CMD_SHIFT) & OTPCPU_CTRL1_CMD_MASK;
	fuse = (1 << OTPCPU_CTRL1_2XFUSE_SHIFT) & OTPCPU_CTRL1_2XFUSE_MASK;
	cof = (1 << OTPCPU_CTRL1_COF_SHIFT) & OTPCPU_CTRL1_COF_MASK;
	prog_en = (1 << OTPCPU_CTRL1_PROG_EN_SHIFT) & OTPCPU_CTRL1_PROG_EN_MASK;
	mode = (2 << OTPCPU_CTRL1_ACCESS_MODE_SHIFT) & OTPCPU_CTRL1_ACCESS_MODE_MASK;
	
	ctrl1 = cmd;
	ctrl1 |= fuse;
	ctrl1 |= cof;
	ctrl1 |= prog_en;
	ctrl1 |= mode;

    ctrl1 |= start ;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
    ctrl0 = DEFAULT_OTPCPU_CTRL0;
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

	ctrl1 &= ~start;
	W_REG(DMU_OTP_CPU_CTRL1, ctrl1);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL1, ctrl1));
	W_REG(DMU_OTP_CPU_CTRL0, ctrl0);
	OTP_DBG(("OTP: PROG WORD: write reg(0x%x) val(0x%x)\n", DMU_OTP_CPU_CTRL0, ctrl0));

	return 0;
}

int
bcm5301x_otp_write_bit(void *oh, uint off)
{
	bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh;
	uint row, col;
	u32 cpu_cfg;
	int rv;

    OTP_DBG(("OTP: WRITE BIT(0x%x)\n", off));

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	row = off / oi->cols;
	col = off % oi->cols;
    OTP_DBG(("OTP: WRITE BIT: bit %d (row = %d, column=%d)\n", off, row, col));

	/* Check if CPU mode is enabled */
	cpu_cfg = R_REG(DMU_OTP_CPU_CONFIG);
	if (!(cpu_cfg & OTPCPU_CFG_CPU_MODE_MASK)) {
		cpu_cfg |= ((1 << OTPCPU_CFG_CPU_MODE_SHIFT) & OTPCPU_CFG_CPU_MODE_MASK);
	    W_REG(DMU_OTP_CPU_CONFIG, cpu_cfg);
	}
	OTP_DBG(("OTP: REG CPU_OTP_CFG(0x%x)=0x%x\n", cpu_cfg));

	/* Initial control registers */
	W_REG(DMU_OTP_CPU_CTRL1, 0);
	W_REG(DMU_OTP_CPU_CTRL0, 0);
	W_REG(DMU_OTP_CPU_ADDR, 0);
	W_REG(DMU_OTP_CPU_BITSEL, 0);

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

    /* Issue ProgDisable command */
	rv = _issue_prog_dis();
	if (rv) {
	    return -1;
	}
	
    /* Issue ProgEnable command */
	rv = _issue_prog_en();
	if (rv) {
	    return -1;
	}

    /* Check if ProgOk status bit asserted */
	rv = _check_prog_ok();
	if (rv) {
	    return -1;
	}

    /* Issue BitProg command */
	rv = _issue_prog_bit(row, col);
	if (rv) {
	    return -1;
	}

	return 0;
}

int
bcm5301x_otp_read_dword(void *oh, uint wn, u32 *data)
{
	/* bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh; */
	u32 cpu_cfg;
    int rv;

    OTP_DBG(("OTP: REG DOWRD(0x%x)\n", wn));
    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	/* Check if CPU mode is enabled */
	cpu_cfg = R_REG(DMU_OTP_CPU_CONFIG);
	if (!(cpu_cfg & OTPCPU_CFG_CPU_MODE_MASK)) {
		cpu_cfg |= ((1 << OTPCPU_CFG_CPU_MODE_SHIFT) & OTPCPU_CFG_CPU_MODE_MASK);
	    W_REG(DMU_OTP_CPU_CONFIG, cpu_cfg);
	}
	
    /* Issue ProgDisable command */
	rv = _issue_prog_dis();
	if (rv) {
	    return -1;
	}
	
    /* Issue ReadWord command */
	rv = _issue_read(wn, data);
	if (rv) {
	    return -1;
	}

	return 0;
}

int
bcm5301x_otp_write_dword(void *oh, uint wn, u32 data)
{
	u32 cpu_cfg;
	int rv;

    OTP_DBG(("OTP: WRITE WORD(0x%x)=0x%x\n", wn, data));

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	/* Check if CPU mode is enabled */
	cpu_cfg = R_REG(DMU_OTP_CPU_CONFIG);
	if (!(cpu_cfg & OTPCPU_CFG_CPU_MODE_MASK)) {
		cpu_cfg |= ((1 << OTPCPU_CFG_CPU_MODE_SHIFT) & OTPCPU_CFG_CPU_MODE_MASK);
	    W_REG(DMU_OTP_CPU_CONFIG, cpu_cfg);
	}

	/* Initial control registers */
	W_REG(DMU_OTP_CPU_CTRL1, 0);
	W_REG(DMU_OTP_CPU_CTRL0, 0);
	W_REG(DMU_OTP_CPU_ADDR, 0);
	W_REG(DMU_OTP_CPU_BITSEL, 0);

	/* Check if cmd_done bit is asserted */
	rv = _check_cmd_done();
	if (rv) {
	    return -1;
	}

    /* Issue ProgDisable command */
	rv = _issue_prog_dis();
	if (rv) {
	    return -1;
	}
	
    /* Issue ProgEnable command */
	rv = _issue_prog_en();
	if (rv) {
	    return -1;
	}

    /* Check if ProgOk status bit asserted */
	rv = _check_prog_ok();
	if (rv) {
	    return -1;
	}

    /* Issue WordProg command */
	rv = _issue_prog_word(wn, data);
	if (rv) {
	    return -1;
	}

	return 0;
}

int
bcm5301x_otp_dump(void *oh, int arg, char *buf, uint size)
{
	bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh;
	uint i, count;
	u32 val;
	struct bcmstrbuf b;

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	if (buf == NULL) {
		OTP_ERR(("otp driver: dump buffer not ready.\n"));
		return BCME_ERROR;
	}

    /* 
     * only dump the meaningful first 96 bytes(768 bits) for better view of the log.
     */
    count = 96;

    _binit(&b, buf, size);
	for (i = 0; i < count / 4; i++) {
		if (!(i % 4)) {
			_bprintf(&b, "\n0x%04x:", 4 * i);
		}
        bcm5301x_otp_read_dword(oi, i,&val);
		_bprintf(&b, " 0x%08x", val);
	}
	_bprintf(&b, "\n");

	return ((int)(b.buf - b.origbuf));
}

int
bcm5301x_otp_dumpstats(void *oh, int arg, char *buf, uint size)
{
	bcm5301x_otpinfo_t *oi = (bcm5301x_otpinfo_t *)oh;
	struct bcmstrbuf b;

    NS_OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return -1;
	}

	if (buf == NULL) {
		OTP_ERR(("otp driver: dump buffer not ready.\n"));
		return BCME_ERROR;
	}

	_binit(&b, buf, size);

	_bprintf(&b, "wsize %d rows %d cols %d\n", oi->wsize, oi->rows, oi->cols);
	_bprintf(&b, "status %x\n", oi->status);
	_bprintf(&b, "\n");

	return 200;	/* real buf length, pick one to cover above print */
}

int ns_otp_module_init(void)
{
	void *ptr;

	ptr = bcm5301x_otp_init();
	if (!ptr) {
	    OTP_ERR(("Northstar OTP module init failed.\n"));
		return -1;
	}

    return 0;
}

void ns_otp_module_exit(void)
{
	bcm5301x_otp_exit();
}

module_init(ns_otp_module_init);
module_exit(ns_otp_module_exit);

EXPORT_SYMBOL(bcm5301x_otp_status);
EXPORT_SYMBOL(bcm5301x_otp_size);
EXPORT_SYMBOL(bcm5301x_otp_init);
EXPORT_SYMBOL(bcm5301x_otp_exit);
EXPORT_SYMBOL(bcm5301x_otp_write_bit);
EXPORT_SYMBOL(bcm5301x_otp_read_dword);
EXPORT_SYMBOL(bcm5301x_otp_write_dword);
EXPORT_SYMBOL(bcm5301x_otp_dump);
EXPORT_SYMBOL(bcm5301x_otp_dumpstats);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM5301X Chip OTP Device Driver");
MODULE_LICENSE("GPL");
