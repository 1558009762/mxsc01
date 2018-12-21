#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "../../kernel/linux-3.6.5/arch/arm/mach-iproc/include/mach/socregs-cygnus.h"
#include "bcm5830x_otp.h"

otp_info_t otp_info;
#define OTP_REG_READ(addr, v)                   \
    do                                          \
    {                                           \
        v = *((volatile unsigned int *)addr);   \
    } while(0)

#define OTP_REG_WRITE(addr, v)                  \
    do                                          \
    {                                           \
        *((volatile unsigned int *)addr) = v;   \
    } while(0)

#define OTP_REG_AND(addr, v)                    \
    do                                          \
    {                                           \
        *((volatile unsigned int *)addr) &= v;  \
    } while(0)

#define OTP_REG_OR(addr, v)                     \
    do                                          \
    {                                           \
        *((volatile unsigned int *)addr) |= v;  \
    } while(0)

void iproc_otp_reset(void)
{
    unsigned int v, mask;
    OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, ~(1<<SOTP_REGS_OTP_PROG_CONTROL__CRC_DISABLE));
    OTP_REG_WRITE(otp_info.reg_APBZ_IDM_IDM_RESET_CONTROL, 0);
    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, v);
    mask = 1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_HW_INIT_DONE;
   	while((v&mask) != mask)
    {
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, v);
    }
	printk("sotp enable");
}
EXPORT_SYMBOL(iproc_otp_reset);

unsigned long long iproc_otp_row_read(unsigned int row_addr, int ecc_enable)
{
	unsigned int v, mask;
    unsigned long long read_data, read_data0, read_data1;

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0,v);
    mask = 1<<SOTP_REGS_OTP_STATUS__FDONE;
    while((v&mask) != mask)
    {   /*Check for FDONE status*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0, v);
    }

    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, 1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_CPU_MODE_EN);   /*Enable OTB acces by CPU*/

    if(ecc_enable)
        OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, ~(1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_DISABLE_ECC));
    else
        OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, 1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_DISABLE_ECC);

    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_ADDR, (row_addr&0x3FF)<<SOTP_REGS_OTP_ADDR__OTP_ROW_ADDR_R);   /*10 bit row address*/
    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, SOTP_READ<<SOTP_REGS_OTP_CTRL_0__OTP_CMD_R);       /*cmd set*/

    /* Start bit to tell SOTP to send command to the OTP controller*/
    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, 1<<SOTP_REGS_OTP_CTRL_0__START); 

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    mask = 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE;
	while((v&mask) != mask)
    {   /*Wait for SOTP command done to be set*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    }

    OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, ~(1<<SOTP_REGS_OTP_CTRL_0__START));

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    mask = 1<<SOTP_REGS_OTP_STATUS_1__ECC_DET;
	if(v&mask)
    {   /*ECC Det*/
		printk("SOTP ECC ERROR Detected ROW %d\n", row_addr);
		read_data = SOTP_ECC_ERR_DETECT;
	}
	else{
		OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_RDDATA_0, read_data0);
		OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_RDDATA_1, read_data1);
		read_data = (((read_data1 & 0x1ff)<<32) | (read_data0&0xFFFFFFFF));
	}

    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE); /*Command done is cleared*/
    OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, ~(1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_CPU_MODE_EN)); /*disable OTP acces by CPU*/

	return read_data ;
}
EXPORT_SYMBOL(iproc_otp_row_read);

void iproc_otp_row_write(unsigned int row_addr, int ecc_enable , unsigned long long data)
{
	int loop ;
	unsigned char prog_array[4] = {0x0F, 0x04, 0x08, 0x0D};
    unsigned int v, mask;

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0, v);
    mask = 1<<SOTP_REGS_OTP_STATUS__FDONE;
    while((v&mask) != mask)
    {   /*Check for FDONE status*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0, v);
    }

    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, 1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_CPU_MODE_EN); /*Enable OTP acces by CPU*/

    if(ecc_enable)
        OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, ~(1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_DISABLE_ECC));
    else
        OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, 1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_DISABLE_ECC);

    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, SOTP_PROG_ENABLE<<SOTP_REGS_OTP_CTRL_0__OTP_CMD_R);

	// In order to avoid unintentional writes / programming of the OTP array, the OTP Controller must be put into programming mode before it will accept program commands.
	// This is done by writing 0xF, 0x4, 0x8, 0xD with program commands prior to starting the actual programming sequence
	for (loop = 0 ; loop < 4 ; loop++)
	{
	    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_WRDATA_0, prog_array[loop]);
        /*Start bit to tell SOTP to send command to the OTP controller*/
        OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, 1<<SOTP_REGS_OTP_CTRL_0__START);

        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
        mask = 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE;
		while((v&mask) != mask)
        {   /*Wait for SOTP command done to be set*/   
            OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
        }

        /*Command done is cleared w1c*/
        OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE);
        OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, ~(1<<SOTP_REGS_OTP_CTRL_0__START));
	}

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0, v);
    mask = 1 <<SOTP_REGS_OTP_STATUS__PROGOK;
	while((v&mask) != mask)
    {   /*Check for PROGOK*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_0, v);
    }

    /*Set  10 bit row address*/
    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_ADDR, (row_addr&0x3FF)<<SOTP_REGS_OTP_ADDR__OTP_ROW_ADDR_R);
    /*Set SOTP Row data*/
    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_WRDATA_0, data&0xFFFFFFFF);
    /*Set SOTP ECC and error bits*/
    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_WRDATA_1, (data&0x1ff00000000)>>32);
    /*Set prog_word command*/
    OTP_REG_WRITE(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, SOTP_PROG_WORD<<SOTP_REGS_OTP_CTRL_0__OTP_CMD_R);
    /*Start bit to tell SOTP to send command to the OTP controller*/
    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, 1<<SOTP_REGS_OTP_CTRL_0__START);

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    mask = 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE;
	while((v&mask) != mask)
    {   /*Wait for SOTP command done to be set*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    }

    /*Command done is cleared w1c*/
    OTP_REG_OR(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, 1<<SOTP_REGS_OTP_STATUS_1__CMD_DONE);
    /*disable OTP acces by CPU*/
    OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_PROG_CONTROL, ~(1<<SOTP_REGS_OTP_PROG_CONTROL__OTP_CPU_MODE_EN));
    /*Clr Start bit after command done*/
    OTP_REG_AND(&otp_info.regs->reg_SOTP_REGS_OTP_CTRL_0, ~(1<<SOTP_REGS_OTP_CTRL_0__START));
}
EXPORT_SYMBOL(iproc_otp_row_write);

int iproc_otp_readDevCfg(otp_dev_cfg_t* devCfg)
{
    /*Section 4: Region 4: Rows 16 to 19*/
	unsigned long long row_data;
	int status = 0;
    unsigned int v, mask;

    OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
    mask = 1<<SOTP_REGS_OTP_STATUS_1__SECTOR4_CRC_FAIL;
	if(v&mask) /*Section 4 CRC failed*/
		return -1;

	row_data = iproc_otp_row_read(16,1);    /*Read Row 16*/
	if((row_data == 0)||(row_data == SOTP_ECC_ERR_DETECT ))
		status = -1;
	else{
		devCfg->BRCMRevisionID = row_data&0xFFFF;
		devCfg->devSecurityConfig = (row_data>>16)&0xFFFF;
	}

	row_data = iproc_otp_row_read(17,1);         /*Read Row 17*/
	if((row_data == 0)||(row_data == SOTP_ECC_ERR_DETECT ))
		status = -1;
	else
		devCfg->ProductID = row_data&0xFFFF;

	row_data = iproc_otp_row_read(18,1);         /*Read Row 18*/
	if((row_data == 0)||(row_data == SOTP_ECC_ERR_DETECT ))
		status = -1;
	else
		devCfg->SBLConfiguration = row_data&0xFFFFFFFF;
	
	return status;
}
EXPORT_SYMBOL(iproc_otp_readDevCfg);

int iproc_otp_readCustomerID(unsigned int *customerID)
{
    /*Section 5: Region 5: Rows 20 to 23*/
    /*Fail bits, ECC bits, Row 23: CRC*/
	unsigned long long row_data;
	uint32_t row = 20;

	do{
		row_data = iproc_otp_row_read(row, 1);
		row++;
	}while(((row_data & SOTP_ECC_ERR_DETECT)    /*(cpu_rd_single(SOTP_REGS_OTP_STATUS_1,4)& 0x20000) //ECC Det*/
    		|| (row_data & 0x18000000000))      /*FAIL*/
    		&&(row<24));                        /*Wait for SOTP command done to be set*/
 	if(!((row_data&SOTP_ECC_ERR_DETECT)||(row_data&0x18000000000)))
    {
        /*returning the entire 32 bits;*/
        /*If only 24 bits of valid CID is returned, how to identify whether dev or production code.*/
        *customerID = (row_data&0xFFFFFFFF);
        printk("Customer ID = %d\n", *customerID);
        return 0;
    }
    else
    {
        printk("Customer ID Invalid\n");
        return -1;
    }
}
EXPORT_SYMBOL(iproc_otp_readCustomerID);

int iproc_otp_readCustomerConfig
    (
    unsigned short *CustomerRevisionID, 
    unsigned short *SBIRevisionID
    )
{
    /*Section 6: Region 6: Rows 24 to 27*/
    /*Fail bits, ECC bits */
    /*No CRC, Only Redundancy*/
	unsigned long long row_data[4],rowdata;
	int i=0,j=0, k=0;
	unsigned int row = 24;
	do{
		rowdata = iproc_otp_row_read(row,1);
		if(!((rowdata & 0x18000000000)||(rowdata & SOTP_ECC_ERR_DETECT)))
        {   //FAIL 
			row_data[i] = rowdata;
			i++;
		}
		row++;
	}while(row < 28);
    
	if(i == 0)
		return -1;
	else if(i == 1){
		*CustomerRevisionID = (row_data[0]>>16)&0xFFFF;
		*SBIRevisionID = row_data[0]&0xFFFF;
	}
	else if(i == 2){
		rowdata = row_data[0]|row_data[1];
		*CustomerRevisionID = (rowdata>>16) & 0xFFFF;
		*SBIRevisionID = rowdata & 0xFFFF;
	}
	else {
		rowdata = 0x0;			
		for(j=0;j<=i-2;j++){
			for(k=j+1;k<=i-1;k++){
				rowdata = rowdata | (row_data[j]&row_data[k]);
			}	
		}
		*CustomerRevisionID = (rowdata>>16) & 0xFFFF;
		*SBIRevisionID = rowdata & 0xFFFF;
	}
    
	return 0;
}
EXPORT_SYMBOL(iproc_otp_readCustomerConfig);

int iproc_otp_readKeys(unsigned char *key, unsigned short *keySize, OTPKeyType type)
{
	unsigned int i, status = 0;
    unsigned int status2 = 0xFFFFFFFF;
	unsigned long long row_data;
	unsigned int row;
	unsigned int *temp_key = (unsigned int *)key;
    unsigned int v, mask;
	
	if(type == OTPKey_DAUTH)
	{
	    /*DAUTH: Section 7, Region 13,14,15 : Rows 52-63*/
	    /*Row60 - CRC, Rows 61,62,63 - Redundant rows*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
        mask = 1<<SOTP_REGS_OTP_STATUS_1__SECTOR7_CRC_FAIL;
		if(v&mask)
        {   /*Wait for SOTP command done to be set*/
			return -1;
        }

		*keySize = 32;
		row = 28;
		while(row<40)
		{	
			row_data = iproc_otp_row_read(row,1);
			if(!(row_data & SOTP_ECC_ERR_DETECT)&&!(row_data & 0x18000000000))
            {   /*Not fail*/ 
                *temp_key = row_data & 0xFFFFFFFF;
                status |= *temp_key;
                status2 &= *temp_key++;
			}
            else
                return -1;
			row++;
		}

    	return 0;
	}
	else if(type == OTPKey_HMAC_SHA256)
	{
	    /*KHMAC : Section 8, Region 7,8,9 : Rows 28-39*/
	    /*Row36 - CRC, Rows 37,38,39 - Redundant rows*/
		OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
        mask = 1<<SOTP_REGS_OTP_STATUS_1__SECTOR8_CRC_FAIL;
		if(v&mask)
        {   /*Wait for SOTP command done to be set*/
			return -1;
        }

		*keySize = 32;
		row = 40;
		i=0;
		while(row<52)
		{	
			row_data = iproc_otp_row_read(row,1);
			if(!(row_data & SOTP_ECC_ERR_DETECT)&& !(row_data & 0x18000000000))
            {
					*temp_key = row_data & 0xFFFFFFFF;
					status |= *temp_key;
					status2 &= *temp_key++;
			}
            else
                return -1;
			row++;
		}

        return 0;
	}
	else if (type == OTPKey_AES)
	{
	    /*KAES : Section 9, Region 10,11,12 : Rows 40-51*/
    	/*Row48 - CRC, Rows 49,50,51- Redundant rows*/
        OTP_REG_READ(&otp_info.regs->reg_SOTP_REGS_OTP_STATUS_1, v);
        mask = 1<<SOTP_REGS_OTP_STATUS_1__SECTOR9_CRC_FAIL;
		if(v&mask)
        {   /*Wait for SOTP command done to be set*/
			return -1;
        }

		*keySize = 32;
		row = 52;
		i=0;
		while(row<64)
		{	
			row_data = iproc_otp_row_read(row,1);
			if(!(row_data & SOTP_ECC_ERR_DETECT)&& !(row_data & 0x18000000000))
            {
					*temp_key = row_data & 0xFFFFFFFF;
					status |= *temp_key;
					status2 &= *temp_key++;
					i++;
			}
            else
                return -1;
            
			row++;
		}

        return 0;
	}	
	
	return -1;
}
EXPORT_SYMBOL(iproc_otp_readKeys);

static int __init iproc_otp_init(void)
{
	int ret = -ENOMEM;

    mutex_init(&otp_info.otp_lock);
    memset(&otp_info, 0, sizeof(otp_info_t));
    otp_info.regs = (volatile otp_reg_t *)ioremap(IPROC_OTP_REG_BASE, IPROC_OTP_REG_LEN);
    if(!otp_info.regs)
    {
        ret = -ENXIO;
        goto fail;
    }
    printk("otp_regs = 0x%08x\n", (unsigned int)otp_info.regs);
    
    otp_info.reg_APBZ_IDM_IDM_RESET_CONTROL = (volatile unsigned int *)ioremap(APBZ_IDM_IDM_RESET_CONTROL, 4);
    if(!otp_info.reg_APBZ_IDM_IDM_RESET_CONTROL)
    {
        ret = -ENXIO;
        goto fail0;
    }
    printk("APBZ_IDM_IDM_RESET_CONTROL = 0x%08x\n", (unsigned int)otp_info.reg_APBZ_IDM_IDM_RESET_CONTROL);

    return 0;
fail0:
    iounmap(otp_info.regs);
fail:    
    return ret;
}

static void __exit iproc_otp_exit(void)
{
	iounmap(otp_info.reg_APBZ_IDM_IDM_RESET_CONTROL);
    iounmap(otp_info.regs);
    printk("otp exit\n");
}

module_init(iproc_otp_init);
module_exit(iproc_otp_exit);
MODULE_DESCRIPTION("IPROC OTP driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");


