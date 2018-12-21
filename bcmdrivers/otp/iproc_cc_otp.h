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

#ifndef	_bcm5301x_cca_otp_h_
#define	_bcm5301x_cca_otp_h_

/* OTP regions */
#define OTP_HW_RGN	1
#define OTP_SW_RGN	2
#define OTP_CI_RGN	4
#define OTP_FUSE_RGN	8
#define OTP_ALL_RGN	0xf	/* From h/w region to end of OTP including checksum */

/* OTP Size */
#define OTP_SZ_MAX		(6144/8)	/* maximum bytes in one CIS */

/* Fixed size subregions sizes in words */
#define OTPGU_CI_SZ		2

/* Supported OpCode for 40nm */
#define OTPPOC_40NM_READ OTPPOC_READ

/* Exported functions */
extern int	cca_otp_status(void *oh);
extern int	cca_otp_size(void *oh);
extern u16	cca_otp_read_bit(void *oh, uint offset);
extern void*	cca_otp_init(void);
extern int cca_otp_exit(void);

extern int	cca_otp_read_region(int region, u16 *data, uint *wlen);
extern int	cca_otp_read_word(uint wn, u16 *data);

extern int	cca_otp_dump(void *oh, int arg, char *buf, uint size);
extern int	cca_otp_dumpstats(void *oh, int arg, char *buf, uint size);

/* Fields in the otpstatus register in rev >= 21 */
#define OTPS_OL_MASK		0x000000ff
#define OTPS_OL_MFG		0x00000001	/* manuf row is locked */
#define OTPS_OL_OR1		0x00000002	/* otp redundancy row 1 is locked */
#define OTPS_OL_OR2		0x00000004	/* otp redundancy row 2 is locked */
#define OTPS_OL_GU		0x00000008	/* general use region is locked */
#define OTPS_GUP_MASK		0x00000f00
#define OTPS_GUP_SHIFT		8
#define OTPS_GUP_HW		0x00000100	/* h/w subregion is programmed */
#define OTPS_GUP_SW		0x00000200	/* s/w subregion is programmed */
#define OTPS_GUP_CI		0x00000400	/* chipid/pkgopt subregion is programmed */
#define OTPS_GUP_FUSE		0x00000800	/* fuse subregion is programmed */
#define OTPS_READY		0x00001000
#define OTPS_RV(x)		(1 << (16 + (x)))	/* redundancy entry valid */
#define OTPS_RV_MASK		0x0fff0000

/* Fields in the otpcontrol register in rev >= 21 */
#define OTPC_PROGSEL		0x00000001
#define OTPC_PCOUNT_MASK	0x0000000e
#define OTPC_PCOUNT_SHIFT	1
#define OTPC_VSEL_MASK		0x000000f0
#define OTPC_VSEL_SHIFT		4
#define OTPC_TMM_MASK		0x00000700
#define OTPC_TMM_SHIFT		8
#define OTPC_ODM		0x00000800
#define OTPC_PROGEN		0x80000000

/* Fields in otpprog in rev >= 21 and HND OTP */
#define OTPP_COL_MASK		0x000000ff
#define OTPP_COL_SHIFT		0
#define OTPP_ROW_MASK		0x0000ff00
#define OTPP_ROW_SHIFT		8
#define OTPP_OC_MASK		0x0f000000
#define OTPP_OC_SHIFT		24
#define OTPP_READERR		0x10000000
#define OTPP_VALUE_MASK		0x20000000
#define OTPP_VALUE_SHIFT	29
#define OTPP_START_BUSY		0x80000000
#define	OTPP_READ		0x40000000	/* HND OTP */

/* Fields in otplayout register */
#define OTPL_HWRGN_OFF_MASK	0x00000FFF
#define OTPL_HWRGN_OFF_SHIFT	0
#define OTPL_WRAP_REVID_MASK	0x00F80000
#define OTPL_WRAP_REVID_SHIFT	19
#define OTPL_WRAP_TYPE_MASK	0x00070000
#define OTPL_WRAP_TYPE_SHIFT	16
#define OTPL_WRAP_TYPE_65NM	0
#define OTPL_WRAP_TYPE_40NM	1

/* otplayout reg corerev >= 36 */
#define OTP_CISFORMAT_NEW	0x80000000

/* Opcodes for OTPP_OC field */
#define OTPPOC_READ		0
#define OTPPOC_BIT_PROG		1
#define OTPPOC_VERIFY		3
#define OTPPOC_INIT		4
#define OTPPOC_SET		5
#define OTPPOC_RESET		6
#define OTPPOC_OCST		7
#define OTPPOC_ROW_LOCK		8
#define OTPPOC_PRESCN_TEST	9

/* Fields in otplayoutextension */
#define OTPLAYOUTEXT_FUSE_MASK	0x3FF

/*
 * error codes could be added but the defined ones shouldn't be changed/deleted
 * these error codes are exposed to the user code
 * when ever a new error code is added to this list
 * please update errorstring table with the related error string and
 * update osl files with os specific errorcode map
*/

#define BCME_OK				0	/* Success */
#define BCME_ERROR			-1	/* Error generic */
#define BCME_BADARG			-2	/* Bad Argument */
#define BCME_BADOPTION			-3	/* Bad option */
#define BCME_NOTUP			-4	/* Not up */
#define BCME_NOTDOWN			-5	/* Not down */
#define BCME_NOTAP			-6	/* Not AP */
#define BCME_NOTSTA			-7	/* Not STA  */
#define BCME_BADKEYIDX			-8	/* BAD Key Index */
#define BCME_RADIOOFF 			-9	/* Radio Off */
#define BCME_NOTBANDLOCKED		-10	/* Not  band locked */
#define BCME_NOCLK			-11	/* No Clock */
#define BCME_BADRATESET			-12	/* BAD Rate valueset */
#define BCME_BADBAND			-13	/* BAD Band */
#define BCME_BUFTOOSHORT		-14	/* Buffer too short */
#define BCME_BUFTOOLONG			-15	/* Buffer too long */
#define BCME_BUSY			-16	/* Busy */
#define BCME_NOTASSOCIATED		-17	/* Not Associated */
#define BCME_BADSSIDLEN			-18	/* Bad SSID len */
#define BCME_OUTOFRANGECHAN		-19	/* Out of Range Channel */
#define BCME_BADCHAN			-20	/* Bad Channel */
#define BCME_BADADDR			-21	/* Bad Address */
#define BCME_NORESOURCE			-22	/* Not Enough Resources */
#define BCME_UNSUPPORTED		-23	/* Unsupported */
#define BCME_BADLEN			-24	/* Bad length */
#define BCME_NOTREADY			-25	/* Not Ready */
#define BCME_EPERM			-26	/* Not Permitted */
#define BCME_NOMEM			-27	/* No Memory */
#define BCME_ASSOCIATED			-28	/* Associated */
#define BCME_RANGE			-29	/* Not In Range */
#define BCME_NOTFOUND			-30	/* Not Found */
#define BCME_WME_NOT_ENABLED		-31	/* WME Not Enabled */
#define BCME_TSPEC_NOTFOUND		-32	/* TSPEC Not Found */
#define BCME_ACM_NOTSUPPORTED		-33	/* ACM Not Supported */
#define BCME_NOT_WME_ASSOCIATION	-34	/* Not WME Association */
#define BCME_SDIO_ERROR			-35	/* SDIO Bus Error */
#define BCME_DONGLE_DOWN		-36	/* Dongle Not Accessible */
#define BCME_VERSION			-37 	/* Incorrect version */
#define BCME_TXFAIL			-38 	/* TX failure */
#define BCME_RXFAIL			-39	/* RX failure */
#define BCME_NODEVICE			-40 	/* Device not present */
#define BCME_NMODE_DISABLED		-41 	/* NMODE disabled */
#define BCME_NONRESIDENT		-42 /* access to nonresident overlay */
#define BCME_LAST			BCME_NONRESIDENT

/* These are collection of BCME Error strings */
#define BCMERRSTRINGTABLE {		\
	"OK",				\
	"Undefined error",		\
	"Bad Argument",			\
	"Bad Option",			\
	"Not up",			\
	"Not down",			\
	"Not AP",			\
	"Not STA",			\
	"Bad Key Index",		\
	"Radio Off",			\
	"Not band locked",		\
	"No clock",			\
	"Bad Rate valueset",		\
	"Bad Band",			\
	"Buffer too short",		\
	"Buffer too long",		\
	"Busy",				\
	"Not Associated",		\
	"Bad SSID len",			\
	"Out of Range Channel",		\
	"Bad Channel",			\
	"Bad Address",			\
	"Not Enough Resources",		\
	"Unsupported",			\
	"Bad length",			\
	"Not Ready",			\
	"Not Permitted",		\
	"No Memory",			\
	"Associated",			\
	"Not In Range",			\
	"Not Found",			\
	"WME Not Enabled",		\
	"TSPEC Not Found",		\
	"ACM Not Supported",		\
	"Not WME Association",		\
	"SDIO Bus Error",		\
	"Dongle Not Accessible",	\
	"Incorrect version",		\
	"TX Failure",			\
	"RX Failure",			\
	"Device Not Present",		\
	"NMODE Disabled",		\
	"Nonresident overlay access", \
}

/* CC capabilities */
#define	CC_CAP_UARTS_MASK	0x00000003	/* Number of UARTs */
#define CC_CAP_MIPSEB		0x00000004	/* MIPS is in big-endian mode */
#define CC_CAP_UCLKSEL		0x00000018	/* UARTs clock select */
#define CC_CAP_UINTCLK		0x00000008	/* UARTs are driven by internal divided clock */
#define CC_CAP_UARTGPIO		0x00000020	/* UARTs own GPIOs 15:12 */
#define CC_CAP_EXTBUS_MASK	0x000000c0	/* External bus mask */
#define CC_CAP_EXTBUS_NONE	0x00000000	/* No ExtBus present */
#define CC_CAP_EXTBUS_FULL	0x00000040	/* ExtBus: PCMCIA, IDE & Prog */
#define CC_CAP_EXTBUS_PROG	0x00000080	/* ExtBus: ProgIf only */
#define	CC_CAP_FLASH_MASK	0x00000700	/* Type of flash */
#define	CC_CAP_PLL_MASK		0x00038000	/* Type of PLL */
#define CC_CAP_PWR_CTL		0x00040000	/* Power control */
#define CC_CAP_OTPSIZE		0x00380000	/* OTP Size (0 = none) */
#define CC_CAP_OTPSIZE_SHIFT	19		/* OTP Size shift */
#define CC_CAP_OTPSIZE_BASE	5		/* OTP Size base */
#define CC_CAP_JTAGP		0x00400000	/* JTAG Master Present */
#define CC_CAP_ROM		0x00800000	/* Internal boot rom active */
#define CC_CAP_BKPLN64		0x08000000	/* 64-bit backplane */
#define	CC_CAP_PMU		0x10000000	/* PMU Present, rev >= 20 */
#define	CC_CAP_ECI		0x20000000	/* ECI Present, rev >= 21 */
#define	CC_CAP_SROM		0x40000000	/* Srom Present, rev >= 32 */
#define	CC_CAP_NFLASH		0x80000000	/* Nand flash present, rev >= 35 */

#define	CC_CAP2_SECI		0x00000001	/* SECI Present, rev >= 36 */
#define	CC_CAP2_GSIO		0x00000002	/* GSIO (spi/i2c) present, rev >= 37 */

/* types */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FASLE
#define FALSE 0
#endif

#ifndef ARRAYSIZE
#define ARRAYSIZE(a)		(sizeof(a)/sizeof(a[0]))
#endif


struct bcmstrbuf {
	char *buf;	
	unsigned int size;	
	char *origbuf;	
	unsigned int origsize;	
};

#endif /* _bcm5301x_cca_otp_h_ */
