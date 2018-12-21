/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

/****************************************************************************/
/**
*  @file    reg.h
*
*  @brief   Generic register defintions used in CSP
*/
/****************************************************************************/

#ifndef CSP_REG_H
#define CSP_REG_H

/* ---- Include Files ---------------------------------------------------- */

#include "stdint.h"

/* ---- Public Constants and Types --------------------------------------- */


/* Read/write register */
#define  __READ_REG32(reg)                  ( __REG32(reg) )
#define  __WRITE_REG32(reg, value)          ( __REG32(reg)  = (uint32_t) (value) )

/* Bit manipulation with a 1-bit mask*/
#define  __TEST_REG32_BIT(reg, bit)         ( pifHw_READ_REG32((reg)) & (bit))
#define  __CLEAR_REG32_BIT(reg, bit)        ( pifHw_WRITE_REG32((reg), (pifHw_READ_REG32((reg)) & (~bit))))
#define  __SET_REG32_BIT(reg, bit)          ( __REG32(reg) |=  ((uint32_t)(bit)));

/* Set/Clear a specific bit location*/
#define  __CLEAR_REG32_BIT_NUM(reg, bitNo)  ( pifHw_WRITE_REG32((reg), (pifHw_READ_REG32((reg)) & (~(1 << (bitNo))))))
#define  __SET_REG32_BIT_NUM(reg, bitNo)    ( __REG32(reg) |=  ((uint32_t)(1 << (bitNo))));

#define __WRITE_REG32_BITS(reg, bits)       ( __REG32(reg) |=  (uint32_t)(bits))

/*
 * Macros used to define a sequence of reserved registers. The start / end
 * are byte offsets in the particular register definition, with the "end"
 * being the offset of the next un-reserved register. E.g. if offsets
 * 0x10 through to 0x1f are reserved, then this reserved area could be
 * specified as follows.
 */
/*
 *  typedef struct
 *  {
 *      uint32_t reg1;          // offset 0x00
 *      uint32_t reg2;          // offset 0x04
 *      uint32_t reg3;          // offset 0x08
 *      uint32_t reg4;          // offset 0x0c
 *      REG32_RSVD(0x10, 0x20);
 *      uint32_t reg5;          // offset 0x20
 *      ...
 *  } EXAMPLE_REG_t;
 */
#define REG8_RSVD(start, end)   uint8_t rsvd_##start[ (end - start) / sizeof(uint8_t) ]
#define REG16_RSVD(start, end)  uint16_t rsvd_##start[ (end - start) / sizeof(uint16_t) ]
#define REG32_RSVD(start, end)  uint32_t rsvd_##start[ (end - start) / sizeof(uint32_t) ]

#ifdef REG_DEBUG

    #ifndef REG_DEBUG_PRINT
        #include <stdio.h>
        #define REG_DEBUG_PRINT             printf
    #endif

#endif

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

#ifdef REG_DEBUG
static inline void reg32_print( volatile uint32_t *reg )
{
    REG_DEBUG_PRINT("0x%x :  0x%x\n", (unsigned)reg, *reg );
}
#endif

static inline void reg32_modify_and( volatile uint32_t *reg, uint32_t value )
{
    #ifdef REG_DEBUG
    REG_DEBUG_PRINT("0x%x &= 0x%x\n", (unsigned)reg, value);
    #endif

    *reg = *reg & value;

    #ifdef REG_DEBUG
    reg32_print( reg );
    #endif
}

static inline void reg32_modify_or( volatile uint32_t *reg, uint32_t value )
{
    #ifdef REG_DEBUG
    REG_DEBUG_PRINT("0x%x |= 0x%x\n", (unsigned)reg, value);
    #endif

    *reg = *reg | value;

    #ifdef REG_DEBUG
    reg32_print( reg );
    #endif
}

static inline void reg32_modify_mask( volatile uint32_t *reg, uint32_t mask, uint32_t value )
{
    #ifdef REG_DEBUG
    REG_DEBUG_PRINT("0x%x =  (reg & 0x%x) | 0x%x\n", (unsigned)reg, mask, value);
    #endif

    *reg = (*reg & mask) | value;

    #ifdef REG_DEBUG
    reg32_print( reg );
    #endif
}

static inline void reg32_write( volatile uint32_t *reg, uint32_t value )
{
    #ifdef REG_DEBUG
    REG_DEBUG_PRINT("0x%x =  0x%x\n", (unsigned)reg, value);
    #endif

    *reg = value;

    #ifdef REG_DEBUG
    reg32_print( reg );
    #endif
}

#endif /* CSP_REG_H */
