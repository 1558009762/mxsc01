/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/


/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>



/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
static struct i2c_client *nxp8026_i2c_datap = NULL;

/* ---- Private Function Prototypes -------------------------------------- */
static int nxp8026_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);


/* ---- Public Variables ------------------------------------------------- */
int nxp8026_i2c_init(void);
int nxp8026_i2c_remove(void);

/* Debug trace */
#define I2C_ENABLE_LOG           0

#if I2C_ENABLE_LOG
#define I2CLOG(f, ...)           printk(KERN_INFO f, ##__VA_ARGS__)
#else
#define I2CLOG(f, ...)
#endif

/* 8026 has 3 I2C addresses */
static const struct i2c_device_id nxp8026_i2c_ids[] = {
	{"coupler_bank0",     0},
	{"coupler_bank1reg0", 1},
	{"coupler_bank1reg1", 2},
	{}
};
MODULE_DEVICE_TABLE(i2c, nxp8026_i2c_ids);

struct i2c_client *coupler_clients[3];

struct i2c_driver nxp8026_i2c_driver =
{
	.driver = {
		.name  = "iproc_smartcardcoupler",
		.owner = THIS_MODULE,
		},
	.probe     = nxp8026_i2c_probe,
//	.remove    = __devexit_p(nxp8026_i2c_remove),
	.id_table  = nxp8026_i2c_ids,
};

/* ---- extern Functions ----------------------------------------------- */
extern int NXP8026_Get_Product_Version(void);


/****************************************************************************
*
*  nxp8026_i2c_read
*
*  reg: address of register to read
*
*  returns: data read (8 bits) or -1 on error
*
***************************************************************************/
int nxp8026_i2c_read(unsigned char reg)
{
   /* Guard against calling in an atomic context */
   //might_sleep();

   if ( nxp8026_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for smart card coupler\n" );
      return -1;
   }
   return i2c_smbus_read_byte_data(nxp8026_i2c_datap, reg);
}

/****************************************************************************
*
*  nxp8026_i2c_write
*
*  reg: address of register to write
*  value: value to be written
*
*  returns: 0 on success, -1 on error
*
***************************************************************************/
int nxp8026_i2c_write(unsigned char reg, unsigned char value)
{
   if ( nxp8026_i2c_datap == NULL )
   {
      printk( "i2c bus not configured for smart card coupler\n" );
      return -1;
   }
   return i2c_smbus_write_byte_data(nxp8026_i2c_datap, reg, value);
}

/****************************************************************************
*
*  nxp8026_i2c_probe
*
***************************************************************************/
static int nxp8026_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	I2CLOG("nxp8026_i2c_probe");

	coupler_clients[id->driver_data] = client;

	nxp8026_i2c_datap = client;
	i2c_set_clientdata( client, nxp8026_i2c_datap );

	I2CLOG("client %x created @ %ld \n", (uint32_t)coupler_clients[id->driver_data], id->driver_data);

	/* get product version */
	/*
	if (id->driver_data == 2)
	{
		i = NXP8026_Get_Product_Version();
		I2CLOG("Got coupler product version %x\n", i);
	}
	*/

	return 0;
}


/****************************************************************************
*
*  nxp8026_i2c_detach_client
*
***************************************************************************/
int nxp8026_i2c_remove(void)
{
	I2CLOG("nxp8026_i2c_remove");

	i2c_del_driver( &nxp8026_i2c_driver);

	nxp8026_i2c_datap = NULL;

	return 0;
}

int nxp8026_i2c_init(void)
{
	int rc;

	I2CLOG("nxp8026_i2c_init");

	/* set up the I2C */
	rc = i2c_add_driver(&nxp8026_i2c_driver);
	if (rc != 0)
	{
		printk("SCI_I2C - Failed to initialize I2C\n");
	}
	return( rc );
}
/*
module_init(nxp8026_i2c_init);
module_exit(nxp8026_i2c_remove);

MODULE_DESCRIPTION("8026 I2C Driver");

*/
