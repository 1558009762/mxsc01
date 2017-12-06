/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifdef BCM_GMAC_DEBUG
#ifndef DEBUG
#define DEBUG
#endif
#endif

#include <common.h>
#include <asm/io.h>
#include <phy.h>
#include <asm/arch/socregs.h>
#include <asm/iproc-common/reg_utils.h>

#define SPINWAIT(exp, us) { \
	uint countdown = (us) + 9; \
	while ((exp) && (countdown >= 10)) {\
		udelay(10); \
		countdown -= 10; \
	} \
}

#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
/* Implement the PHY read/write via CMICD */
#define CMICD_BASE_ADDRESS              0x3200000
#define CMIC_COMMON_OFFSET              0x10000

#define MIIM_PARAM_REG                  CMIC_COMMON_MIIM_PARAM_BASE
#define MIIM_PARAM__MIIM_CYCLE_SHIFT    CMIC_COMMON_MIIM_PARAM__MIIM_CYCLE_R
#define MIIM_PARAM__MIIM_CYCLE_MASK     ((1 << CMIC_COMMON_MIIM_PARAM__MIIM_CYCLE_WIDTH) - 1)
#define MIIM_PARAM__INTERNAL_SEL_SHIFT  CMIC_COMMON_MIIM_PARAM__INTERNAL_SEL
#define MIIM_PARAM__INTERNAL_SEL_MASK   ((1 << CMIC_COMMON_MIIM_PARAM__INTERNAL_SEL_WIDTH) - 1)
#define MIIM_PARAM__BUS_ID_SHIFT        CMIC_COMMON_MIIM_PARAM__BUS_ID_R
#define MIIM_PARAM__BUS_ID_MASK         ((1 << CMIC_COMMON_MIIM_PARAM__BUS_ID_WIDTH) - 1)
#define MIIM_PARAM__C45_SEL_SHIFT       CMIC_COMMON_MIIM_PARAM__C45_SEL
#define MIIM_PARAM__C45_SEL_MASK        ((1 << CMIC_COMMON_MIIM_PARAM__C45_SEL_WIDTH) - 1)
#define MIIM_PARAM__PHY_ID_SHIFT        CMIC_COMMON_MIIM_PARAM__PHY_ID_R
#define MIIM_PARAM__PHY_ID_MASK         ((1 << CMIC_COMMON_MIIM_PARAM__PHY_ID_WIDTH) - 1)
#define MIIM_PARAM__PHY_DATA_SHIFT      CMIC_COMMON_MIIM_PARAM__PHY_DATA_R
#define MIIM_PARAM__PHY_DATA_MASK       ((1 << CMIC_COMMON_MIIM_PARAM__PHY_DATA_WIDTH) - 1)

#define MIIM_READ_DATA_REG              CMIC_COMMON_MIIM_READ_DATA_BASE
#define MIIM_READ_DATA__DATA_SHIFT      CMIC_COMMON_MIIM_READ_DATA__DATA_R
#define MIIM_READ_DATA__DATA_MASK       ((1 << CMIC_COMMON_MIIM_READ_DATA__DATA_WIDTH) - 1)

#define MIIM_ADDRESS_REG                        CMIC_COMMON_MIIM_ADDRESS_BASE
#define MIIM_ADDRESS__CLAUSE_45_DTYPE_SHIFT     CMIC_COMMON_MIIM_ADDRESS__CLAUSE_45_DTYPE_R
#define MIIM_ADDRESS__CLAUSE_45_DTYPE_MASK      ((1 << CMIC_COMMON_MIIM_ADDRESS__CLAUSE_45_DTYPE_WIDTH) - 1)
#define MIIM_ADDRESS__CLAUSE_45_REGADR_SHIFT    CMIC_COMMON_MIIM_ADDRESS__CLAUSE_45_REGADR_R
#define MIIM_ADDRESS__CLAUSE_45_REGADR_MASK     ((1 << CMIC_COMMON_MIIM_ADDRESS__CLAUSE_45_REGADR_WIDTH) - 1)
#define MIIM_ADDRESS__CLAUSE_22_REGADR_SHIFT    CMIC_COMMON_MIIM_ADDRESS__CLAUSE_22_REGADR_R
#define MIIM_ADDRESS__CLAUSE_22_REGADR_MASK     ((1 << CMIC_COMMON_MIIM_ADDRESS__CLAUSE_22_REGADR_WIDTH) - 1)

#define MIIM_CTRL_REG                   CMIC_COMMON_MIIM_CTRL_BASE
#define MIIM_CTRL__MIIM_RD_START_SHIFT  CMIC_COMMON_MIIM_CTRL__MIIM_RD_START
#define MIIM_CTRL__MIIM_RD_START_MASK   ((1 << CMIC_COMMON_MIIM_CTRL__MIIM_RD_START_WIDTH) - 1)
#define MIIM_CTRL__MIIM_WR_START_SHIFT  CMIC_COMMON_MIIM_CTRL__MIIM_WR_START
#define MIIM_CTRL__MIIM_WR_START_MASK   ((1 << CMIC_COMMON_MIIM_CTRL__MIIM_WR_START_WIDTH) - 1)

#define MIIM_STAT_REG                   CMIC_COMMON_MIIM_STAT_BASE
#define MIIM_STAT__MIIM_OPN_DONE_SHIFT  CMIC_COMMON_MIIM_STAT__MIIM_OPN_DONE
#define MIIM_STAT__MIIM_OPN_DONE_MASK   ((1 << CMIC_COMMON_MIIM_STAT__MIIM_OPN_DONE_WIDTH) - 1)

#define SET_REG_FIELD(reg_value, fshift, fmask, fvalue) \
            (reg_value) = ((reg_value) & ~((fmask) << (fshift))) |  \
            (((fvalue) & (fmask)) << (fshift))
#define ISET_REG_FIELD(reg_value, fshift, fmask, fvalue) \
            (reg_value) = (reg_value) | (((fvalue) & (fmask)) << (fshift))
#define GET_REG_FIELD(reg_value, fshift, fmask) \
            (((reg_value) & ((fmask) << (fshift))) >> (fshift))

#define MIIM_OP_MAX_HALT_USEC   500

enum {
    MIIM_OP_MODE_READ,
    MIIM_OP_MODE_WRITE,
    MIIM_OP_MODE_MAX
};

struct cmicd_miim_cmd {
    int bus_id;
    int int_sel;
    int phy_id;
    int regnum;
    int c45_sel;
    uint16_t op_mode;
    uint16_t val;
};

static inline uint32_t
cmicd_miim_reg_read(uint32_t reg)
{
    uint32_t addr;
    volatile uint32_t value;

    addr = (CMICD_BASE_ADDRESS + CMIC_COMMON_OFFSET) + reg;
    value = reg32_read((volatile uint32_t *)addr);

    return  (uint32_t)value;
}

static inline void
cmicd_miim_reg_write(uint32_t reg, uint32_t data)
{
    uint32_t addr;

    addr = (CMICD_BASE_ADDRESS + CMIC_COMMON_OFFSET) + reg;
    reg32_write((volatile uint32_t *)addr, data);
}

static inline void
cmicd_miim_set_op_read(uint32_t *data, uint32_t set)
{
    SET_REG_FIELD(*data, MIIM_CTRL__MIIM_RD_START_SHIFT,
            MIIM_CTRL__MIIM_RD_START_MASK, set);
}

static inline void
cmicd_miim_set_op_write(uint32_t *data, uint32_t set)
{
    SET_REG_FIELD(*data, MIIM_CTRL__MIIM_WR_START_SHIFT,
            MIIM_CTRL__MIIM_WR_START_MASK, set);
}

static inline int
do_cmicd_miim_op(uint32_t op, uint32_t param, uint32_t addr, uint16_t *reg_val)
{
    uint32_t val, op_done;
    int ret = 0;
    int usec = MIIM_OP_MAX_HALT_USEC;

    if (op >= MIIM_OP_MODE_MAX) {
        error("%s : invalid op code %d\n",__FUNCTION__,op);
        return -1;
    }

    cmicd_miim_reg_write(MIIM_PARAM_REG, param);
    cmicd_miim_reg_write(MIIM_ADDRESS_REG, addr);
    val = cmicd_miim_reg_read(MIIM_CTRL_REG);
    if(op == MIIM_OP_MODE_READ) {
        cmicd_miim_set_op_read(&val, 1);
    } else {
        cmicd_miim_set_op_write(&val, 1);
    }
    cmicd_miim_reg_write(MIIM_CTRL_REG, val);

    do {
        op_done = GET_REG_FIELD(cmicd_miim_reg_read(MIIM_STAT_REG),
                MIIM_STAT__MIIM_OPN_DONE_SHIFT, MIIM_STAT__MIIM_OPN_DONE_MASK);
        if (op_done) {
            break;
        }

        udelay(1);
        usec--;
    } while (usec > 0);

    if (op_done) {
        if(op == MIIM_OP_MODE_READ) {
            *reg_val = cmicd_miim_reg_read(MIIM_READ_DATA_REG);
        }
    } else {
        ret = -1;
    }

    val = cmicd_miim_reg_read(MIIM_CTRL_REG);
    if(op == MIIM_OP_MODE_READ) {
        cmicd_miim_set_op_read(&val, 0);
    } else {
        cmicd_miim_set_op_write(&val, 0);
    }
    cmicd_miim_reg_write(MIIM_CTRL_REG, val);

    return ret;
}
static int
cmicd_miim_op(struct cmicd_miim_cmd *cmd)
{
    uint32_t miim_param =0, miim_addr = 0;
    int rv = 0;

    ISET_REG_FIELD(miim_param, MIIM_PARAM__BUS_ID_SHIFT,
            MIIM_PARAM__BUS_ID_MASK, cmd->bus_id);

    if (cmd->int_sel) {
        ISET_REG_FIELD(miim_param, MIIM_PARAM__INTERNAL_SEL_SHIFT,
                MIIM_PARAM__INTERNAL_SEL_MASK, 1);
    }

    ISET_REG_FIELD(miim_param, MIIM_PARAM__PHY_ID_SHIFT,
            MIIM_PARAM__PHY_ID_MASK, cmd->phy_id);

    if (cmd->op_mode == MIIM_OP_MODE_WRITE) {
        ISET_REG_FIELD(miim_param, MIIM_PARAM__PHY_DATA_SHIFT,
                MIIM_PARAM__PHY_DATA_MASK, cmd->val);
    }

    if (cmd->c45_sel) {
        ISET_REG_FIELD(miim_param, MIIM_PARAM__C45_SEL_SHIFT,
                MIIM_PARAM__C45_SEL_MASK, 1);

        ISET_REG_FIELD(miim_addr, MIIM_ADDRESS__CLAUSE_45_REGADR_SHIFT,
                MIIM_ADDRESS__CLAUSE_45_REGADR_MASK, cmd->regnum);
        ISET_REG_FIELD(miim_addr, MIIM_ADDRESS__CLAUSE_45_DTYPE_SHIFT,
                MIIM_ADDRESS__CLAUSE_45_REGADR_MASK, cmd->regnum >> 16);
    } else {
        ISET_REG_FIELD(miim_addr, MIIM_ADDRESS__CLAUSE_22_REGADR_SHIFT,
                MIIM_ADDRESS__CLAUSE_22_REGADR_MASK, cmd->regnum);
    }

    rv = do_cmicd_miim_op(cmd->op_mode, miim_param, miim_addr, &cmd->val);

    return rv;
}
void
chip_phy_wr(uint ext, uint phyaddr, uint reg, uint16_t v)
{
    struct cmicd_miim_cmd cmd = {0};
    int rv;

    cmd.bus_id = CONFIG_EXTERNAL_PHY_BUS_ID;
    if (!ext) {
        cmd.int_sel = 1;
    }
    cmd.phy_id = phyaddr;
    cmd.regnum = reg;
    cmd.val = v;

    cmd.op_mode = MIIM_OP_MODE_WRITE;

    rv = cmicd_miim_op(&cmd);
    if (rv < 0) {
        error("phyaddr %x: PHY register write is failed! error code is %d\n", phyaddr, rv);
    }
    return;
}
uint16_t
chip_phy_rd(uint ext, uint phyaddr, uint reg)
{
    struct cmicd_miim_cmd cmd = {0};
    int rv;

    cmd.bus_id = CONFIG_EXTERNAL_PHY_BUS_ID;
    if (!ext) {
        cmd.int_sel = 1;
    }
    cmd.phy_id = phyaddr;
    cmd.regnum = reg;

    cmd.op_mode = MIIM_OP_MODE_READ;

    rv = cmicd_miim_op(&cmd);
    if (rv < 0) {
        error("phyaddr %x: PHY register read is failed! error code is %d\n", phyaddr, rv);
    }
    return cmd.val;
}
#else /* !(defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2)) */
void
chip_phy_wr(uint ext, uint phyaddr, uint reg, uint16_t v)
{
    uint32_t tmp;
    volatile uint32_t *phy_ctrl = (uint32_t *)ChipcommonB_MII_Management_Control;
    volatile uint32_t *phy_data = (uint32_t *)ChipcommonB_MII_Management_Command_Data;

    /* Wait until Mii mgt interface not busy */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: busy\n", phyaddr);
    }

    /* Set preamble and MDCDIV */
    tmp = (1 << ChipcommonB_MII_Management_Control__PRE) | 0x1a; /* MDCDIV */
    if (ext) {  /* Ext phy */
        tmp |= (1 << ChipcommonB_MII_Management_Control__EXT);
    }  else {
        tmp &= ~(1 << ChipcommonB_MII_Management_Control__EXT);
    }
    reg32_write(phy_ctrl, tmp);

    /* Wait for it to complete */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: ChipcommonB_MII_Management_Control did not complete\n", phyaddr);
    }

    /* Issue the write */
    /* Set start bit, write op, phy addr, phy reg & data */
    tmp = ((1 << ChipcommonB_MII_Management_Command_Data__SB_R) |       /* SB */
           (1 << ChipcommonB_MII_Management_Command_Data__OP_R) |       /* OP - wrt */
           (phyaddr << ChipcommonB_MII_Management_Command_Data__PA_R) | /* PA */
           (reg << ChipcommonB_MII_Management_Command_Data__RA_R) |     /* RA */
           (2 << ChipcommonB_MII_Management_Command_Data__TA_R) |       /* TA */
            v);                                                         /* Data */
    reg32_write(phy_data, tmp);

    /* Wait for it to complete */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: ChipcommonB_MII_Management_Command_Data did not complete\n", phyaddr);
    }
}

uint16_t
chip_phy_rd(uint ext, uint phyaddr, uint reg)
{
    uint32_t tmp;
    volatile uint32_t *phy_ctrl = (uint32_t *)ChipcommonB_MII_Management_Control;
    volatile uint32_t *phy_data = (uint32_t *)ChipcommonB_MII_Management_Command_Data;

    /* Wait until Mii mgt interface not busy */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: busy\n", phyaddr);
    }

    /* Set preamble and MDCDIV */
    tmp = (1 << ChipcommonB_MII_Management_Control__PRE) | 0x1a; /* MDCDIV */
    if (ext) {  /* ext phy */
        tmp |= (1 << ChipcommonB_MII_Management_Control__EXT);
    }  else {
        tmp &= ~(1 << ChipcommonB_MII_Management_Control__EXT);
    }
    reg32_write(phy_ctrl, tmp);

    /* Wait for it to complete */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: ChipcommonB_MII_Management_Control did not complete\n", phyaddr);
    }

    /* Issue the read */
    /* Set start bit, write op, phy addr, phy reg & data */
    tmp =   (1 << ChipcommonB_MII_Management_Command_Data__SB_R) |          /* SB */
            (2 << ChipcommonB_MII_Management_Command_Data__OP_R) |          /* OP - rd*/
            (phyaddr << ChipcommonB_MII_Management_Command_Data__PA_R) |    /* PA */
            (reg << ChipcommonB_MII_Management_Command_Data__RA_R) |        /* RA */
            (2 << ChipcommonB_MII_Management_Command_Data__TA_R);           /* TA */
    reg32_write(phy_data, tmp);

    /* Wait for it to complete */
    SPINWAIT((reg32_read(phy_ctrl) & (1 << ChipcommonB_MII_Management_Control__BSY)), 1000);
    tmp = reg32_read(phy_ctrl);
    if (tmp & (1 << ChipcommonB_MII_Management_Control__BSY)) {
        error("phyaddr %x: ChipcommonB_MII_Management_Command_Data did not complete\n", phyaddr);
    }

    /* Read data */
    tmp = reg32_read(phy_data);

    return (tmp & 0xffff);
}
#endif /* (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2)) */
