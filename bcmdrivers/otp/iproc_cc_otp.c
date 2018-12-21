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
#ifdef ARCH_NORTHSTAR
#include <mach/socregs.h>
#else
#include <mach/iproc_regs.h>
#endif

#include "iproc_cc_otp.h"

#include <linux/io.h>
#include <mach/io_map.h>

static void * baseAddr;

#define R_REG(reg)       ioread32(baseAddr + (reg&0xffff))
#define W_REG(reg, val)        iowrite32(val, baseAddr + (reg&0xffff))

#define OTP_INIT_CHK  \
    {\
         if (!baseAddr) { \
             return BCME_ERROR; \
		 } \
    }

/* register base compatibilty */
#if !defined(CCA_OTP_STS)
#define CCA_OTP_STS ChipcommonA_OTPStatus
#endif
#if !defined(CCA_OTP_CTRL)
#define CCA_OTP_CTRL ChipcommonA_OTPCtrl
#endif
#if !defined(CCA_OTP_PROG)
#define CCA_OTP_PROG ChipcommonA_OTPProg
#endif
#if !defined(CCA_OTP_LAYOUT)
#define CCA_OTP_LAYOUT ChipcommonA_OTPLayout
#endif
#if !defined(CCA_CORE_CAP)
#define CCA_CORE_CAP ChipcommonA_CoreCapabilities
#endif


/*
 * Supported OTP controller:
 *     Northstar : IPX
 *
 */

#if !defined(BCMIPXOTP)
#define BCMIPXOTP	1
#endif

#define OTP_ERR_VAL	0x0001
#define OTP_MSG_VAL	0x0002
#define OTP_DBG_VAL	0x0004
u32 cca_otp_msg_level = OTP_ERR_VAL;

#if defined(BCMDBG) || defined(BCMDBG_ERR)
#define OTP_ERR(args)	do {if (cca_otp_msg_level & OTP_ERR_VAL) printk args;} while (0)
#else
#define OTP_ERR(args)
#endif

#ifdef BCMDBG
#define OTP_MSG(args)	do {if (cca_otp_msg_level & OTP_MSG_VAL) printk args;} while (0)
#define OTP_DBG(args)	do {if (cca_otp_msg_level & OTP_DBG_VAL) printk args;} while (0)
#else
#define OTP_MSG(args)
#define OTP_DBG(args)
#endif

#define OTPP_TRIES	10000000	/* # of tries for OTPP */

#ifdef BCMIPXOTP
/* Maximum OTP redundancy entries, 3 tables * 3 entries each */
#define MAXNUMRDES		9		
#endif

/* OTP common function type */
typedef int	(*otp_status_t)(void *oh);
typedef int	(*otp_size_t)(void *oh);
typedef void*	(*otp_init_t)(void);
typedef u16	(*otp_read_bit_t)(void *oh, uint off);
typedef int	(*otp_read_region_t)(void *oh, int region, u16 *data, uint *wlen);
typedef int	(*otp_dump_t)(void *oh, int arg, char *buf, uint size);
typedef int	(*otp_read_word_t)(void *oh, uint wn, u16 *data);

/* OTP function struct */
typedef struct otp_fn_s {
	otp_size_t		size;
	otp_read_bit_t		read_bit;
	otp_dump_t		dump;
	otp_status_t		status;
	otp_init_t		init;

	otp_read_region_t	read_region;
	otp_read_word_t		read_word;
} otp_fn_t;

typedef struct {
	otp_fn_t	*fn;		/* OTP functions */

#ifdef BCMIPXOTP
	/* IPX OTP section */
	u16		wsize;		/* Size of otp in words */
	u16		rows;		/* Geometry */
	u16		cols;		/* Geometry */
	u32		status;		/* Flag bits (lock/prog/rv).
					 * (Reflected only when OTP is power cycled)
					 */
	u16		hwbase;		/* hardware subregion offset */
	u16		hwlim;		/* hardware subregion boundary */
	u16		swbase;		/* software subregion offset */
	u16		swlim;		/* software subregion boundary */
	u16		fbase;		/* fuse subregion offset */
	u16		flim;		/* fuse subregion boundary */
	int		otpgu_base;	/* offset to General Use Region */
	u16		fusebits;	/* num of fusebits */
	struct {
		u8 width;		/* entry width in bits */
		u8 val_shift;	/* value bit offset in the entry */
		u8 offsets;		/* # entries */
		u8 stat_shift;	/* valid bit in otpstatus */
		u16 offset[MAXNUMRDES];	/* entry offset in OTP */
	} rde_cb;			/* OTP redundancy control blocks */
#endif /* BCMIPXOTP */

} otpinfo_t;

static otpinfo_t otpinfo;

/*
 * ROM accessor to avoid struct in shdat
 */
static otpinfo_t *
get_otpinfo(void)
{
	return (otpinfo_t *)&otpinfo;
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
 * IPX OTP Code
 *
 *   Exported functions:
 *	ipxotp_status()
 *	ipxotp_size()
 *	ipxotp_init()
 *	ipxotp_read_bit()
 *	ipxotp_read_region()
 *	ipxotp_read_word()
 *	ipxotp_dump()
 *
 *   IPX internal functions:
 *	_ipxotp_init()
 *	ipxotp_max_rgnsz()
 *	ipxotp_otprb16()
 *
 */

#ifdef BCMIPXOTP

#define	OTPWSIZE		16	/* word size */
#define HWSW_RGN(rgn)		(((rgn) == OTP_HW_RGN) ? "h/w" : "s/w")


/* Subregion word offsets in General Use region */
#define OTPGU_HSB_OFF		0
#define OTPGU_SFB_OFF		1
#define OTPGU_CI_OFF		2
#define OTPGU_P_OFF		3
#define OTPGU_UPPER_OFF 4 /* Upper GUR, offset bit 512 */



/* OTP Size */
#define OTP_SZ_CHECKSUM		(16/8)		/* 16 bits */
#define OTP_2K_RGN_FU_LIM 2031 /* fuse region end offset(bit) */


static u16
ipxotp_read_bit(void *oh, uint off)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	uint k, row, col;
	u32 otpp, st;

	row = off / oi->cols;
	col = off % oi->cols;

	otpp = OTPP_START_BUSY |
	        ((OTPPOC_40NM_READ << OTPP_OC_SHIFT) & OTPP_OC_MASK) |
	        ((row << OTPP_ROW_SHIFT) & OTPP_ROW_MASK) |
	        ((col << OTPP_COL_SHIFT) & OTPP_COL_MASK);
	OTP_DBG(("%s: off = %d, row = %d, col = %d, otpp = 0x%x",
	         __FUNCTION__, off, row, col, otpp));
	W_REG(CCA_OTP_PROG, otpp);

	for (k = 0;
	     ((st = R_REG(CCA_OTP_PROG)) & OTPP_START_BUSY) && (k < OTPP_TRIES);
	     k ++)
		;
	if (k >= OTPP_TRIES) {
		OTP_ERR(("\n%s: BUSY stuck: st=0x%x, count=%d\n", __FUNCTION__, st, k));
		return 0xffff;
	}
	if (st & OTPP_READERR) {
		OTP_ERR(("\n%s: Could not read OTP bit %d\n", __FUNCTION__, off));
		return 0xffff;
	}
	st = (st & OTPP_VALUE_MASK) >> OTPP_VALUE_SHIFT;

	OTP_DBG((" => %d\n", st));
	return (int)st;
}

static u16
ipxotp_otprb16(void *oh, uint wn)
{
	uint base, i;
	u16 val;
	u16 bit;

	base = wn * 16;

	val = 0;
	for (i = 0; i < 16; i++) {
		if ((bit = ipxotp_read_bit(oh, base + i)) == 0xffff)
			break;
		val = val | (bit << i);
	}
	if (i < 16)
		val = 0xffff;

	return val;
}

static int
ipxotp_status(void *oh)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	return (int)(oi->status);
}

/* Return size in bytes */
static int
ipxotp_size(void *oh)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	return (int)oi->wsize * 2;
}


/* Calculate max HW/SW region byte size by substracting fuse region and checksum size,
 * osizew is oi->wsize (OTP size - GU size) in words
 */
static int
ipxotp_max_rgnsz(otpinfo_t *oi)
{
	int osizew = oi->wsize;
	int ret = 0;
	u16 checksum;

    u16 sf_boundary;
    u16 val;

	checksum = OTP_SZ_CHECKSUM;

    sf_boundary = ipxotp_otprb16(oi, oi->otpgu_base + OTPGU_SFB_OFF); /* S/F boundry */

	val = OTP_2K_RGN_FU_LIM - sf_boundary + 1; /* fuse region size in bits */

    oi->fusebits = (val / 8);

	ret = osizew*2 - oi->fusebits - checksum;
	
	OTP_MSG(("max region size %d bytes\n", ret));
	return ret;
}

/*
 * OTP sizes for 40nm
 */
static int
ipxotp_otpsize_set_40nm(otpinfo_t *oi, uint otpsz)
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

	oi->wsize = (oi->cols * oi->rows)/OTPWSIZE; /*128 (=2048 bits/16)*/
	return 0;
}

static void
_ipxotp_init(otpinfo_t *oi)
{
	oi->otpgu_base = (R_REG(CCA_OTP_LAYOUT) & OTPL_HWRGN_OFF_MASK)
		>> OTPL_HWRGN_OFF_SHIFT; /* otpgu_base = 0x200 (512) */
	oi->otpgu_base >>= 4; /* words */ /* otpgu_base = 32 */
	oi->otpgu_base -= OTPGU_UPPER_OFF; /* otpgu_base = 28 (start bit: 448) */

	/* Read OTP lock bits and subregion programmed indication bits */
	oi->status = R_REG(CCA_OTP_STS);

	OTP_DBG(("%s: status 0x%x\n", __FUNCTION__, oi->status));

	/*
	 * h/w region base and fuse region limit are fixed to the top and
	 * the bottom of the general use region. Everything else can be flexible.
	 */
	oi->hwbase = oi->otpgu_base + OTPGU_UPPER_OFF; /* bit 512 */
	oi->hwlim = oi->wsize; /* bit 2048 */
	oi->flim = oi->wsize;  /* bit 2048 */


	/* Update hwlim and swbase */
	if (oi->status & OTPS_GUP_HW) {
		OTP_DBG(("%s: hw region programmed\n", __FUNCTION__));

		oi->hwlim = ipxotp_otprb16(oi, oi->otpgu_base + OTPGU_HSB_OFF) / 16;

		oi->swbase = oi->hwlim;
	} else
		oi->swbase = oi->hwbase;

	/* Update swlim and fbase */
	/* subtract fuse and checksum from beginning */
    oi->swlim = ipxotp_max_rgnsz(oi) / 2;

	if (oi->status & OTPS_GUP_SW) {
		OTP_DBG(("%s: hw region programmed\n", __FUNCTION__));

		oi->swlim = ipxotp_otprb16(oi, oi->otpgu_base + OTPGU_SFB_OFF) / 16;
		oi->fbase = oi->swlim;
	}
	else
		oi->fbase = oi->swbase;


	OTP_DBG(("%s: OTP limits---\n"
		"hwbase %d/%d hwlim %d/%d\n"
		"swbase %d/%d swlim %d/%d\n"
		"fbase %d/%d flim %d/%d\n", __FUNCTION__,
		oi->hwbase, oi->hwbase * 16, oi->hwlim, oi->hwlim * 16,
		oi->swbase, oi->swbase * 16, oi->swlim, oi->swlim * 16,
		oi->fbase, oi->fbase * 16, oi->flim, oi->flim * 16));
}

static void *
ipxotp_init(void)
{
	uint otpsz, otpwt;
	otpinfo_t *oi = NULL;
    u16 offset[] = {269, 286, 303, 333, 350, 367, 397, 414, 431};

	OTP_MSG(("%s: Use IPX OTP controller\n", __FUNCTION__));


	/* Retrieve OTP region info */
    otpsz = (R_REG(CCA_CORE_CAP) & CC_CAP_OTPSIZE) >> CC_CAP_OTPSIZE_SHIFT;
	if (otpsz == 0) {
		OTP_ERR(("%s: No OTP\n", __FUNCTION__));
		goto exit;
	}

	oi = get_otpinfo();
	otpwt = (R_REG(CCA_OTP_LAYOUT) & OTPL_WRAP_TYPE_MASK) >> OTPL_WRAP_TYPE_SHIFT;

	if (otpwt == OTPL_WRAP_TYPE_40NM) {
		ipxotp_otpsize_set_40nm(oi, otpsz);
	} else {
		OTP_ERR(("%s: Unknown or unsupported wrap type: %d\n", __FUNCTION__, otpwt));
	}

	OTP_MSG(("%s: rows %u cols %u wsize %u\n", __FUNCTION__, oi->rows, oi->cols, oi->wsize));

    memcpy(offset, &oi->rde_cb.offset, sizeof(offset));
    oi->rde_cb.offsets = ARRAYSIZE(offset);
    oi->rde_cb.width = 17;
    oi->rde_cb.val_shift = 13;
    oi->rde_cb.stat_shift = 16;

	_ipxotp_init(oi);

exit:

	return (void *)oi;
}

static int
ipxotp_read_region(void *oh, int region, u16 *data, uint *wlen)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	uint base, i, sz;

	if (oh == NULL) {
		OTP_ERR(("otp_init failed.\n"));
		return BCME_ERROR;
	}

	/* Validate region selection */
	switch (region) {
	case OTP_HW_RGN:
		sz = (uint)oi->hwlim - oi->hwbase;
		if (!(oi->status & OTPS_GUP_HW)) {
			OTP_ERR(("%s: h/w region not programmed\n", __FUNCTION__));
			*wlen = sz;
			return BCME_NOTFOUND;
		}
		if (*wlen < sz) {
			OTP_ERR(("%s: buffer too small, should be at least %u\n",
			         __FUNCTION__, oi->hwlim - oi->hwbase));
			*wlen = sz;
			return BCME_BUFTOOSHORT;
		}
		base = oi->hwbase;
		break;
	case OTP_SW_RGN:
		sz = ((uint)oi->swlim - oi->swbase);
		if (!(oi->status & OTPS_GUP_SW)) {
			OTP_ERR(("%s: s/w region not programmed\n", __FUNCTION__));
			*wlen = sz;
			return BCME_NOTFOUND;
		}
		if (*wlen < sz) {
			OTP_ERR(("%s: buffer too small should be at least %u\n",
			         __FUNCTION__, oi->swlim - oi->swbase));
			*wlen = sz;
			return BCME_BUFTOOSHORT;
		}
		base = oi->swbase;
		break;
	case OTP_CI_RGN:
		sz = OTPGU_CI_SZ;
		if (!(oi->status & OTPS_GUP_CI)) {
			OTP_ERR(("%s: chipid region not programmed\n", __FUNCTION__));
			*wlen = sz;
			return BCME_NOTFOUND;
		}
		if (*wlen < sz) {
			OTP_ERR(("%s: buffer too small, should be at least %u\n",
			         __FUNCTION__, OTPGU_CI_SZ));
			*wlen = sz;
			return BCME_BUFTOOSHORT;
		}
		base = oi->otpgu_base + OTPGU_CI_OFF;
		break;
	case OTP_FUSE_RGN:
		sz = (uint)oi->flim - oi->fbase;
		if (!(oi->status & OTPS_GUP_FUSE)) {
			OTP_ERR(("%s: fuse region not programmed\n", __FUNCTION__));
			*wlen = sz;
			return BCME_NOTFOUND;
		}
		if (*wlen < sz) {
			OTP_ERR(("%s: buffer too small, should be at least %u\n",
			         __FUNCTION__, oi->flim - oi->fbase));
			*wlen = sz;
			return BCME_BUFTOOSHORT;
		}
		base = oi->fbase;
		break;
	case OTP_ALL_RGN:
		sz = ((uint)oi->flim - oi->hwbase);
		if (!(oi->status & (OTPS_GUP_HW | OTPS_GUP_SW))) {
			OTP_ERR(("%s: h/w & s/w region not programmed\n", __FUNCTION__));
			*wlen = sz;
			return BCME_NOTFOUND;
		}
		if (*wlen < sz) {
			OTP_ERR(("%s: buffer too small, should be at least %u\n",
				__FUNCTION__, oi->hwlim - oi->hwbase));
			*wlen = sz;
			return BCME_BUFTOOSHORT;
		}
		base = oi->hwbase;
		break;
	default:
		OTP_ERR(("%s: reading region %d is not supported\n", __FUNCTION__, region));
		return BCME_BADARG;
	}

	/* Read the data */
	for (i = 0; i < sz; i ++) {
	    data[i] = ipxotp_otprb16(oi, base + i);
	}

	*wlen = sz;
	return 0;
}

static int
ipxotp_read_word(void *oh, uint wn, u16 *data)
{
	otpinfo_t *oi = (otpinfo_t *)oh;

	if (oh == NULL) {
		OTP_ERR(("otp_init failed.\n"));
		return BCME_ERROR;
	}

	/* Read the data */
	*data = ipxotp_otprb16(oi, wn);

    if (*data == 0xffff) {
	    return -1;
    } else {
    	return 0;
    }
}

static int
ipxotp_dump(void *oh, int arg, char *buf, uint size)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	uint i, count;
	u16 val;
	struct bcmstrbuf b;

	if (oh == NULL) {
		OTP_ERR(("otp_init failed.\n"));
		return BCME_ERROR;
	}

	count = ipxotp_size(oh);

	_binit(&b, buf, size);
	for (i = 0; i < count / 2; i++) {
		if (!(i % 4)) {
			_bprintf(&b, "\n0x%04x:", 2 * i);
		}
        val = ipxotp_otprb16(oi, i);
		_bprintf(&b, " 0x%04x", val);
	}
	_bprintf(&b, "\n");

	return ((int)(b.buf - b.origbuf));
}

static otp_fn_t ipxotp_fn = {
	(otp_size_t)ipxotp_size,
	(otp_read_bit_t)ipxotp_read_bit,
	(otp_dump_t)NULL,		/* Assigned in otp_init */
	(otp_status_t)ipxotp_status,
	(otp_init_t)ipxotp_init,

	(otp_read_region_t)ipxotp_read_region,
	(otp_read_word_t)ipxotp_read_word,
};

#endif /* BCMIPXOTP */


/*
 * Exported drivers:
 *	cca_otp_status()
 *	cca_otp_size()
 *	cca_otp_read_bit()
 *	cca_otp_init()
 * 	cca_otp_read_region()
 * 	cca_otp_read_word()
 * 	cca_otp_dump()
 *  cca_otp_dumpstats()
 */

int
cca_otp_status(void *oh)
{
	otpinfo_t *oi = (otpinfo_t *)oh;

    OTP_INIT_CHK;
	
	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return BCME_ERROR;
	}

	if (oi->fn == NULL) {
		OTP_ERR(("otp driver not initialized.\n"));
		return BCME_ERROR;
	}

	return oi->fn->status(oh);
}

int
cca_otp_size(void *oh)
{
	otpinfo_t *oi = (otpinfo_t *)oh;

    OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return BCME_ERROR;
	}

	if (oi->fn == NULL) {
		OTP_ERR(("otp driver not initialized.\n"));
		return BCME_ERROR;
	}

	return oi->fn->size(oh);
}

u16
cca_otp_read_bit(void *oh, uint offset)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	u16 readBit;

    OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return BCME_ERROR;
	}

	if (oi->fn == NULL) {
		OTP_ERR(("otp driver not initialized.\n"));
		return BCME_ERROR;
	}

	readBit = (u16)oi->fn->read_bit(oi, offset);
	return readBit;
}

void *
cca_otp_init(void)
{
	otpinfo_t *oi;
	/* void *ret = NULL; */

    if (baseAddr == NULL) {
        baseAddr = ioremap(IPROC_CCA_REG_BASE, 0x1000);
        OTP_DBG(("CCA OTP baseAddr=%x\n",baseAddr));
    } else {
        /* already init */
        return get_otpinfo();
    }

    /* OTP_INIT_CHK; */
	if (!baseAddr)
		return NULL;

	oi = get_otpinfo();
	memset(oi, 0, sizeof(otpinfo_t));

	ipxotp_fn.dump = ipxotp_dump;
	oi->fn = &ipxotp_fn;

	if (oi->fn == NULL) {
		OTP_ERR(("otp_init: unsupported OTP type\n"));
		return NULL;
	}

    oi->fn->init();

	return (void *)oi;
}

int
cca_otp_exit(void)
{
	otpinfo_t *oi;

    if (baseAddr) {
	    iounmap(baseAddr);
		baseAddr = NULL;
    }

	oi = get_otpinfo();
	memset(oi, 0, sizeof(otpinfo_t));

	return 0;
}

int
cca_otp_read_region(int region, u16 *data, uint *wlen)
{
	void *oh;
	int err = 0;

    OTP_INIT_CHK;

	oh = cca_otp_init();
	if (oh == NULL) {
		OTP_ERR(("otp_init failed.\n"));
		err = BCME_ERROR;
		goto out;
	}

	err = (((otpinfo_t*)oh)->fn->read_region)(oh, region, data, wlen);

out:
	return err;
}

int
cca_otp_read_word(uint wn, u16 *data)
{
	void *oh;
	int err = 0;

    OTP_INIT_CHK;

	oh = cca_otp_init();

	if (oh == NULL) {
		OTP_ERR(("otp_init failed.\n"));
		err = BCME_ERROR;
		goto out;
	}

	if (((otpinfo_t*)oh)->fn->read_word == NULL) {
		err = BCME_UNSUPPORTED;
		goto out;
	}
	err = (((otpinfo_t*)oh)->fn->read_word)(oh, wn, data);
out:
	return err;
}

int
cca_otp_dump(void *oh, int arg, char *buf, uint size)
{
	otpinfo_t *oi = (otpinfo_t *)oh;

    OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return BCME_ERROR;
	}

	if (oi->fn == NULL) {
		OTP_ERR(("otp driver not initialized.\n"));
		return BCME_ERROR;
	}

	if (buf == NULL) {
		OTP_ERR(("otp driver: dump buffer not ready.\n"));
		return BCME_ERROR;
	}

	if (oi->fn->dump == NULL)
		return BCME_UNSUPPORTED;
	else
		return oi->fn->dump(oh, arg, buf, size);
}

int
cca_otp_dumpstats(void *oh, int arg, char *buf, uint size)
{
	otpinfo_t *oi = (otpinfo_t *)oh;
	struct bcmstrbuf b;

    OTP_INIT_CHK;

	if (oh == NULL) {
		OTP_ERR(("otp driver: invalid handler.\n"));
		return BCME_ERROR;
	}

	if (buf == NULL) {
		OTP_ERR(("otp driver: dump buffer not ready.\n"));
		return BCME_ERROR;
	}

	_binit(&b, buf, size);

	_bprintf(&b, "wsize %d rows %d cols %d\n", oi->wsize, oi->rows, oi->cols);
	_bprintf(&b, "hwbase %d hwlim %d swbase %d swlim %d fusebits %d\n",
		oi->hwbase, oi->hwlim, oi->swbase, oi->swlim, oi->fbase, oi->flim, oi->fusebits);
	_bprintf(&b, "otpgu_base %d status %x\n", oi->otpgu_base, oi->status);
	_bprintf(&b, "\n");

	return 200;	/* real buf length, pick one to cover above print */
}

int cca_otp_module_init(void)
{
	void *ptr;

    OTP_DBG(("CCA OTP module init\n"));

	ptr = cca_otp_init();
	if (!ptr) {
	    OTP_ERR(("CCA OTP module init failed.\n"));
		return -1;
	}

    return 0;
}

void cca_otp_module_exit(void)
{
    OTP_DBG(("CCA OTP module exit\n"));

	cca_otp_exit();
}

module_init(cca_otp_module_init);
module_exit(cca_otp_module_exit);

EXPORT_SYMBOL(cca_otp_status);
EXPORT_SYMBOL(cca_otp_size);
EXPORT_SYMBOL(cca_otp_read_bit);
EXPORT_SYMBOL(cca_otp_init);
EXPORT_SYMBOL(cca_otp_exit);
EXPORT_SYMBOL(cca_otp_read_region);
EXPORT_SYMBOL(cca_otp_read_word);
EXPORT_SYMBOL(cca_otp_dump);
EXPORT_SYMBOL(cca_otp_dumpstats);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("IPROC OTP Device Driver");
MODULE_LICENSE("GPL");
