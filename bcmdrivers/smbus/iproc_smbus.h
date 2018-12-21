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

#ifndef __IPROC_SMBUS_H__
#define __IPROC_SMBUS_H__

#define IPROC_I2C_INVALID_ADDR 0xFF

#define MAX_PROC_BUF_SIZE       256
#define MAX_PROC_NAME_SIZE      15
#define PROC_GLOBAL_PARENT_DIR  "iproc-i2c"
#define PROC_ENTRY_DEBUG        "iproc-i2c-dbg"

#define IPROC_SMB_MAX_RETRIES   35

#define GETREGFLDVAL(regval, mask, startbit) (((regval) & (mask)) >> (startbit))

#define SETREGFLDVAL(regval, fldval, mask, startbit) regval = \
                                                      (regval & ~(mask)) | \
                                                      ((fldval) << (startbit))

/* Enum to specify clock speed. The user will provide it during initialization.
 * If needed, it can be changed dynamically
 */
typedef enum iproc_smb_clk_freq {
    I2C_SPEED_100KHz = 0,
    I2C_SPEED_400KHz = 1,
    I2C_SPEED_INVALID = 255
} smb_clk_freq_t;

/* This enum will be used to notify the user of status of a data transfer
 * request 
 */
typedef enum iproc_smb_error_code {
    I2C_NO_ERR = 0,
    I2C_TIMEOUT_ERR = 1,
    I2C_INVALID_PARAM_ERR = 2, /* Invalid parameter(s) passed to the driver */
    I2C_OPER_IN_PROGRESS = 3, /* The driver API was called before the present
                                 transfer was completed */
    I2C_OPER_ABORT_ERR = 4, /* Transfer aborted unexpectedly, for example a NACK
                               received, before last byte was read/written */
    I2C_FUNC_NOT_SUPPORTED = 5, /* Feature or function not supported 
	                               (e.g., 10-bit addresses, or clock speeds
                                   other than 100KHz, 400KHz) */
} iproc_smb_error_code_t;

/* Counters will be used mainly for testing and debugging */
struct iproc_smb_counters {
    unsigned int num_read_requests;
    unsigned int num_write_requests;
    unsigned int num_read_errors;
    unsigned int num_write_errors;
    unsigned int mstr_rx_evt_cnt; /* ISR counter to check recv event */
    unsigned int mstr_start_busy_cnt; /* ISR counter to checking xact sts */
    unsigned int mstr_rx_fifo_full_cnt; /* ISR counter to detect rx fifo full */
    unsigned int last_int_sts; /* last value of intr status reg */
};


/* This enum may be used in a call back function to provide the user of the 
 * type of request sent by the user. It can also be used for testing and 
 * debugging purposes
 */
typedef enum iproc_smb_message_type {
    I2C_DISABLE_MSG = 0, /* To be used after hardware initialization. 
                            Driver will _not_ respond to API calls */
    I2C_ENABLE_MSG = 1, /* Used after hardware initialization, if required. 
                           Driver will start responding to API calls. 
                           Will not (re-)program the hardware. */
    I2C_READ_MSG = 2, /* I2C read request from application */
    I2C_WRITE_MSG = 3 /* I2C write request from application */
} iproc_smb_message_type_t;

/* For debugging purposes, we will store the information about the last
 * (latest) transfer request from the client application
 */
struct iproc_smb_dbg_trans_info
{
    iproc_smb_message_type_t i2c_last_mesg_type;
    unsigned int i2c_last_dev_addr;
    unsigned int i2c_last_num_bytes_xfer_req;
};

struct procfs {
    char name[MAX_PROC_NAME_SIZE];
    struct proc_dir_entry *parent;
};

/* This structure will be used internally by the driver to maintain its
 * configuration information as well as information programmed in to the 
 * hardware
 */
struct iproc_smb_drv_int_data {
    struct device *dev;
    struct iproc_smb_drv_int_data *next;

    int irq;

    unsigned int drv_state_init; /* 1 = Initialized, 0 = not initialized */

    unsigned int drv_state_open; /* 1 = Accepting transaction requests, 
                                    0 = Not accepting transaction requests */
    smb_clk_freq_t clk_speed;

    void __iomem *block_base_addr; /* iomapped virtual base address for 
                                      register access */

    struct i2c_adapter adapter;

    unsigned int i2c_slave_addr; /* Up to four 7-bit SMB slave addresses can be
                                    assigned, we will assume only one for now.
                                    Valid only if SMBus will act as a slave
                                    device */

    struct semaphore xfer_lock; /* Lock for data transfer */

	struct completion ses_done; /* To signal the command completion */

    struct procfs proc;

    volatile int debug;

    unsigned int master_rx_fifo_thr; /* Master FIFO threshold. Interrupt will be
                                       generated if the threshold is exceeded */

    unsigned int slave_rx_fifo_thr; /* Slave FIFO threshold. Interrupt will be 
                                       generated if the threshold is exceeded */

    unsigned int enable_evts; /* If true, enable interrupts. If false, 
                                 disable interrupts. Default is false */
    unsigned int evt_enable_bmap; /* Bit map of events enabled by the driver */

    struct iproc_smb_counters smb_counters; /* Statistics maintained by driver. A caller
                                               can request them through an API */
};

/* Data to be supplied by the platform to initialise the IPROC SMBus (I2C).
 * block
 * init: Function called during driver initialization.  Used by platform to 
 * configure GPIO functions and similar.
 */
struct iproc_smb_platform_data {
    int (*init)(struct iproc_smb_drv_int_data *iproc_i2c_info_ptr, int flags);

    unsigned int flags;
};

/* This structure will be used by the user during driver initialization to pass
 * initial configuration information to the driver
 */
struct iproc_smb_init_params {
    unsigned int intr_mode; /* TRUE (1) for enabling interrupt mode,
                               FALSE (0) for polling mode */
    unsigned int clock_freq; /* 0=100KHz, 1=400KHz */
    void  (*i2c_callback_func)(unsigned char *data); /* Application can 
                                                         register a callback
                                                         function for driver to
                                                         notify the application
                                                         of any asynchronous 
                                                         event(s), or exception.
                                                         Can be NULL */
};

/* Structure used to pass information to read/write functions. */
struct iproc_xact_info {
    bool cmd_valid; /* true if command field below is valid. Otherwise, false */
    unsigned short command; /* Passed by caller to send SMBus command code */
    unsigned char *data; /* actual data pased by the caller */
    unsigned int size; /* Size of data buffer passed */
    unsigned short flags; /* Sent by caller specifying PEC, 10-bit addresses */
    unsigned char smb_proto; /* SMBus protocol to use to perform transaction */
};

#define XACT_TIMEOUT (msecs_to_jiffies(100)) /* Verify if 100 is OK */

#endif /* __IPROC_SMBUS_H__ */
