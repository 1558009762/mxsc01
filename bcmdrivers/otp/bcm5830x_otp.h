#ifndef __BCM5830X_OTP_H__
#define __BCM5830X_OTP_H__

#define IPROC_OTP_REG_BASE  SOTP_REGS_OTP_PROG_CONTROL
#define IPROC_OTP_REG_LEN   0x70
#define OTP_CDEV_NAME       "otp"

#define SOTP_ECC_ERR_DETECT 0x8000000000000000

#define SOTP_READ         0
#define SOTP_READ_BURST   1
#define SOTP_PROG_ENABLE  2
#define SOTP_PROG_DISABLE 3
#define SOTP_PROG_WORD    10

#define SOTP_REGS_OTP_STATUS__CMDDONE 1
#define SOTP_REGS_OTP_STATUS__PROGOK 2
#define SOTP_REGS_OTP_STATUS__FDONE 3

typedef enum _OTPKeyType
{
	OTPKey_AES,
	OTPKey_HMAC_SHA256,
	OTPKey_DAUTH,
	OTPKey_DevIdentity,
	OTPKey_Binding
} OTPKeyType;

typedef struct _otp_reg_t
{
    unsigned int reg_SOTP_REGS_OTP_PROG_CONTROL;
    unsigned int reg_SOTP_REGS_OTP_WRDATA_0;
    unsigned int reg_SOTP_REGS_OTP_WRDATA_1;
    unsigned int reg_SOTP_REGS_OTP_ADDR;
    unsigned int reg_SOTP_REGS_OTP_CTRL_0;
    unsigned int rsvd0;
    unsigned int reg_SOTP_REGS_OTP_STATUS_0;
    unsigned int reg_SOTP_REGS_OTP_STATUS_1;
    unsigned int reg_SOTP_REGS_OTP_RDDATA_0;
    unsigned int reg_SOTP_REGS_OTP_RDDATA_1;
    unsigned int reg_SOTP_REGS_SOTP_CHIP_STATES;
    unsigned int rsvd1;
    unsigned int reg_SOTP_REGS_OTP_ECCCNT;
    unsigned int reg_SOTP_REGS_OTP_BAD_ADDR;
    unsigned int reg_SOTP_REGS_OTP_WR_LOCK;
    unsigned int reg_SOTP_REGS_OTP_RD_LOCK;
    unsigned int reg_SOTP_REGS_ROM_BLOCK_START;
    unsigned int reg_SOTP_REGS_ROM_BLOCK_END;
    unsigned int reg_SOTP_REGS_SMAU_CTRL;
    unsigned int reg_SOTP_REGS_CHIP_CTRL;
    unsigned int reg_SOTP_REGS_SR_STATE_0;
    unsigned int reg_SOTP_REGS_SR_STATE_1;
    unsigned int reg_SOTP_REGS_SR_STATE_2;
    unsigned int reg_SOTP_REGS_SR_STATE_3;
    unsigned int reg_SOTP_REGS_SR_STATE_4;
    unsigned int reg_SOTP_REGS_SR_STATE_5;
    unsigned int reg_SOTP_REGS_SR_STATE_6;
    unsigned int reg_SOTP_REGS_SR_STATE_7;
} otp_reg_t;

typedef struct _dev_config {
	unsigned short BRCMRevisionID;
	unsigned short devSecurityConfig;
	unsigned short ProductID;
	unsigned short Reserved;
	unsigned short SBLConfiguration;
} otp_dev_cfg_t;

typedef struct _otp_info 
{
    volatile otp_reg_t *regs;
    struct mutex otp_lock;
    volatile unsigned int *reg_APBZ_IDM_IDM_RESET_CONTROL;
} otp_info_t;

void                iproc_otp_reset(void);
unsigned long long  iproc_otp_row_read(unsigned int row_addr, int ecc_enable);
void                iproc_otp_row_write(unsigned int row_addr, int ecc_enable , unsigned long long data);
int                 iproc_otp_readDevCfg(otp_dev_cfg_t* devCfg);
int                 iproc_otp_readCustomerID(unsigned int *customerID);
int                 iproc_otp_readCustomerConfig(unsigned short *CustomerRevisionID, 
                                                 unsigned short *SBIRevisionID);
int                 iproc_otp_readKeys(unsigned char *key, unsigned short *keySize, OTPKeyType type);

#endif
