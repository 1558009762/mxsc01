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

#ifndef	_bcm5301xotp_h_
#define	_bcm5301xotp_h_

/* Exported functions */
extern int	bcm5301x_otp_status(void *oh);
extern int	bcm5301x_otp_size(void *oh);
extern void*	bcm5301x_otp_init(void);
extern int	bcm5301x_otp_exit(void);

extern int	bcm5301x_otp_write_bit(void *oh, uint offset);
extern int	bcm5301x_otp_read_dword(void *oh, uint wn, u32 *data);
extern int	bcm5301x_otp_write_dword(void *oh, uint wn, u32 data);

extern int	bcm5301x_otp_dump(void *oh, int arg, char *buf, uint size);
extern int	bcm5301x_otp_dumpstats(void *oh, int arg, char *buf, uint size);
/*
struct bcmstrbuf {
	char *buf;	
	unsigned int size;	
	char *origbuf;	
	unsigned int origsize;	
};
*/

/* DMU register definition */
#define  DMU_OTP_CPU_CTRL0                                            0x1800c240
#define  DMU_OTP_CPU_CTRL0_BASE                                            0x240
#define  DMU_OTP_CPU_CTRL0_DATAMASK                                   0xffffffff
#define  DMU_OTP_CPU_CTRL0_RDWRMASK                                   0x00000000
#define  DMU_OTP_CPU_CTRL1                                            0x1800c244
#define  DMU_OTP_CPU_CTRL1_BASE                                            0x244
#define  DMU_OTP_CPU_CTRL1_DATAMASK                                   0xffffffff
#define  DMU_OTP_CPU_CTRL1_RDWRMASK                                   0x00000000
#define  DMU_OTP_CPU_ADDR                                             0x1800c24c
#define  DMU_OTP_CPU_ADDR_BASE                                             0x24c
#define  DMU_OTP_CPU_ADDR_DATAMASK                                    0x0000ffff
#define  DMU_OTP_CPU_ADDR_RDWRMASK                                    0x00000000
#define  DMU_OTP_CPU_BITSEL                                           0x1800c250
#define  DMU_OTP_CPU_BITSEL_BASE                                           0x250
#define  DMU_OTP_CPU_BITSEL_DATAMASK                                  0x0000ffff
#define  DMU_OTP_CPU_BITSEL_RDWRMASK                                  0x00000000
#define  DMU_OTP_CPU_WRITE_DATA                                       0x1800c254
#define  DMU_OTP_CPU_WRITE_DATA_BASE                                       0x254
#define  DMU_OTP_CPU_WRITE_DATA_DATAMASK                              0xffffffff
#define  DMU_OTP_CPU_WRITE_DATA_RDWRMASK                              0x00000000
#define  DMU_OTP_CPU_CONFIG                                           0x1800c258
#define  DMU_OTP_CPU_CONFIG_BASE                                           0x258
#define  DMU_OTP_CPU_CONFIG_DATAMASK                                  0x00000003
#define  DMU_OTP_CPU_CONFIG_RDWRMASK                                  0x00000000
#define  DMU_OTP_CPU_READ_DATA                                        0x1800c25c
#define  DMU_OTP_CPU_READ_DATA_BASE                                        0x25c
#define  DMU_OTP_CPU_READ_DATA_DATAMASK                               0xffffffff
#define  DMU_OTP_CPU_READ_DATA_RDWRMASK                               0x00000000
#define  DMU_OTP_CPU_STS                                              0x1800c260
#define  DMU_OTP_CPU_STS_BASE                                              0x260
#define  DMU_OTP_CPU_STS_DATAMASK                                     0x0000ffff
#define  DMU_OTP_CPU_STS_RDWRMASK                                     0x00000000

/* fields in DMU_OTP_CPU_CTRL1 */
#define OTPCPU_CTRL1_START_MASK		0x00000001
#define OTPCPU_CTRL1_START_SHIFT	0
#define OTPCPU_CTRL1_CMD_MASK		0x0000003e
#define OTPCPU_CTRL1_CMD_SHIFT		1
#define OTPCPU_CTRL1_2XFUSE_MASK	0x00001000
#define OTPCPU_CTRL1_2XFUSE_SHIFT	12
#define OTPCPU_CTRL1_COF_MASK		0x00080000
#define OTPCPU_CTRL1_COF_SHIFT		19
#define OTPCPU_CTRL1_PROG_EN_MASK		0x00200000
#define OTPCPU_CTRL1_PROG_EN_SHIFT		21
#define OTPCPU_CTRL1_ACCESS_MODE_MASK		0x00c00000
#define OTPCPU_CTRL1_ACCESS_MODE_SHIFT		22

/* Opcodes for OTPP_OC field */
#define OTPCPU_CMD_READ		0
#define OTPCPU_CMD_PROG_EN		1
#define OTPCPU_CMD_PROG_DIS		2
#define OTPCPU_CMD_BIT_PROG		10
#define OTPCPU_CMD_WORD_PROG		11

/* fields in DMU_OTP_CPU_CONFIG */
#define OTPCPU_CFG_CPU_MODE_MASK		0x00000001
#define OTPCPU_CFG_CPU_MODE_SHIFT		0
#define OTPCPU_CFG_CPU_DISABLE_OTP_ACCESS_MASK		0x00000002
#define OTPCPU_CFG_CPU_DISABLE_OTP_ACCESS_SHIFT		1

/* fields in DMU_OTP_CPU_STS */
#define OTPCPU_STS_CMD_DONE_MASK		0x00000001
#define OTPCPU_STS_CMD_DONE_SHIFT		0
#define OTPCPU_STS_PROG_OK_MASK		0x00001000
#define OTPCPU_STS_PROG_OK_SHIFT		12

#endif /* _bcm5301xotp_h_ */
