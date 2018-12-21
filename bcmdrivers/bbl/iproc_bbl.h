#ifndef __IPROC_BBL_H__
#define __IPROC_BBL_H__

#define IPROC_BBL_REG_BASE  0x03026000
#define IPROC_BBL_POWER_STS 0x0301c02c
#define IPROC_BBL_AUTH_REG  0x03024c74

/*KEK related defines*/
#define BBL_KEK_LENGTH          128 /* 128 Bit length */
#define BBL_KEK_START_SEC_MEM   BBL_SEC0_MEM /* start of KEK SEC_MEM bank number */
#define BBL_TOTAL_KEK_BANK      (BBL_KEK_LENGTH/32)
#define BBL_KEK_SHA256_OFFSET   (BBL_KEK_START_SEC_MEM + BBL_TOTAL_KEK_BANK)
#define BBL_SEC0                0x00000200

#define BBL_KEK_INVALID_ENTRY_1 0x00000000
#define BBL_KEK_INVALID_ENTRY_2 0xFFFFFFFF
#define BBL_KEK_VALID   	    0x5A5A5A5A
#define BBL_KEK_INVALID 	    0xDEADC0DE

#define sec1280_clr_bit     (1<<1)
#define BBL_CONFIG1         0x0000006c

/* SEC1280 Bank Register defines */
#define BBL_SEC0_MEM	0x00
#define BBL_SEC1_MEM	0x01
#define BBL_SEC2_MEM	0x02
#define BBL_SEC3_MEM	0x03
#define BBL_SEC4_MEM	0x04
#define BBL_SEC5_MEM	0x05
#define BBL_SEC6_MEM	0x06
#define BBL_SEC7_MEM	0x07
#define BBL_SEC8_MEM	0x08
#define BBL_SEC9_MEM	0x09
#define BBL_SEC10_MEM	0x0A
#define BBL_SEC11_MEM	0x0B
#define BBL_SEC12_MEM	0x0C
#define BBL_SEC13_MEM	0x0D
#define BBL_SEC14_MEM	0x0E
#define BBL_SEC15_MEM	0x0F
#define BBL_SEC16_MEM	0x10
#define BBL_SEC17_MEM	0x11
#define BBL_SEC18_MEM	0x12
#define BBL_SEC19_MEM	0x13
#define BBL_SEC20_MEM	0x14
#define BBL_SEC21_MEM	0x15
#define BBL_SEC22_MEM	0x16
#define BBL_SEC23_MEM	0x17
#define BBL_SEC24_MEM	0x18
#define BBL_SEC25_MEM	0x19
#define BBL_SEC26_MEM	0x1A
#define BBL_SEC27_MEM	0x1B
#define BBL_SEC28_MEM	0x1C
#define BBL_SEC29_MEM	0x1D
#define BBL_SEC30_MEM	0x1E
#define BBL_SEC31_MEM	0x1F
#define BBL_SEC32_MEM	0x20
#define BBL_SEC33_MEM	0x21
#define BBL_SEC34_MEM	0x22
#define BBL_SEC35_MEM	0x23
#define BBL_SEC36_MEM	0x24
#define BBL_SEC37_MEM	0x25
#define BBL_SEC38_MEM	0x26
#define BBL_SEC39_MEM	0x27

extern int iproc_bbl_1280b_mem_write
    (
    unsigned int mem_addr, 
    unsigned int *p_dat, 
    unsigned int len
    );

extern int iproc_bbl_1280b_mem_read
    (
    unsigned int mem_addr, 
    unsigned int *p_dat, 
    unsigned int len
    );

extern int iproc_bbl_1280b_mem_clr(void);

extern void iproc_bbl_reg_write(unsigned int indirect_reg_addr, unsigned int v);

extern unsigned int iproc_bbl_reg_read(unsigned int indirect_reg_addr);

extern int iproc_bbl_tamper_p0n0_enable(void);

extern int iproc_bbl_tamper_p0n0_disable(void);

extern int iproc_bbl_emesh_enable(void);

extern int iproc_bbl_emesh_disable(void);

extern int iproc_bbl_tmon_enable(void);

extern int iproc_bbl_tmon_disable(void);

extern int iproc_bbl_fmon_enable(void);

extern int iproc_bbl_fmon_disable(void);

extern int iproc_bbl_vmon_enable(void);

extern int iproc_bbl_vmon_disable(void);

#endif
