/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef REG_UTILS
#define REG_UTILS

/* ---- Include Files ---------------------------------------------------- */

#include <linux/types.h>

/* ---- Public Constants and Types --------------------------------------- */

#define __REG32(x)      (*((volatile uint32_t *)(x)))
#define __REG16(x)      (*((volatile uint16_t *)(x)))
#define __REG8(x)       (*((volatile uint8_t *) (x)))

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/****************************************************************************/
/*
 *   32-bit register access functions
 */
/****************************************************************************/
#if __BYTE_ORDER == __ORDER_LITTLE_ENDIAN__
#pragma message "Register access is normal"
#else
#pragma message "Register access swapped for big endian compile"
#pragma GCC push_options
#pragma GCC optimize ("O0")
#endif

extern uint32_t reg_debug;
#define	REG_DEBUG(val) (reg_debug = val)

//#define DEBUG_REG

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
static uint32_t swap_u32(uint32_t i) {
    uint8_t c1, c2, c3, c4;    
 
    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;
 
    return ((uint32_t)c1 << 24) + ((uint32_t)c2 << 16) + ((uint32_t)c3 << 8) + c4;
}
#endif
static inline void 
reg32_clear_bits(volatile uint32_t *reg, uint32_t value)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x): 0x%x 0x%x\n", __FUNCTION__, (uint32_t)reg, *reg, (*reg & ~(value)));
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg &= ~(value);
#else
    *reg &= ~(swap_u32(value));
#endif
}

static inline void 
reg32_set_bits(volatile uint32_t *reg, uint32_t value)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x): 0x%x 0x%x\n", __FUNCTION__, (uint32_t)reg, *reg, (*reg | value));
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg |= value;
#else
    *reg |= swap_u32(value);
#endif
}

static inline void 
reg32_toggle_bits(volatile uint32_t *reg, uint32_t value)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x): 0x%x 0x%x\n", __FUNCTION__, (uint32_t)reg, *reg, (*reg ^ value));
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg ^= value;
#else
    *reg ^= swap_u32(value);
#endif
}

static inline void 
reg32_write_masked(volatile uint32_t *reg, uint32_t mask, 
                                      uint32_t value)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x): 0x%x 0x%x\n", __FUNCTION__, (uint32_t)reg, *reg, (*reg & (~mask)) | (value & mask));
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg = (*reg & (~mask)) | (value & mask);
#else
    *reg = (*reg & swap_u32(~mask)) | swap_u32(value & mask);
#endif
}

static inline void 
reg32_write(volatile uint32_t *reg, uint32_t value)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x, 0x%x)\n", __FUNCTION__, (uint32_t)reg, value);
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg = value;
#else
    *reg = swap_u32(value);
#endif
}

static inline uint32_t 
reg32_read(volatile uint32_t *reg)
{
#ifdef DEBUG_REG
	if (reg_debug)
		printf("%s reg (0x%x): 0x%x\n", __FUNCTION__, (uint32_t)reg, *reg);
#endif
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return *reg;
#else
    return swap_u32(*reg);
#endif
}

/****************************************************************************/
/*
 *   16-bit register access functions
 */
/****************************************************************************/

static inline uint16_t swap_u16(uint16_t i) {
    uint8_t c1, c2;
 
    c1 = i & 255;
    c2 = (i >> 8) & 255;
 
    return (c1 << 8) + c2;
}

static inline void 
reg16_clear_bits(volatile uint16_t *reg, uint16_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg &= ~(value);
#else
    *reg &= swap_u16(~(value));
#endif
}

static inline void 
reg16_set_bits(volatile uint16_t *reg, uint16_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg |= value;
#else
    *reg |= swap_u16(value);
#endif
}

static inline void 
reg16_toggle_bits(volatile uint16_t *reg, uint16_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg ^= value;
#else
    *reg ^= swap_u16(value);
#endif
}

static inline void 
reg16_write_masked(volatile uint16_t *reg, uint16_t mask, uint16_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg = (*reg & (~mask)) | (value & mask);
#else
    *reg = (*reg & swap_u16(~mask)) | swap_u16(value & mask);
#endif
}

static inline void 
reg16_write(volatile uint16_t *reg, uint16_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    *reg = value;
#else
    *reg = swap_u16(value);
#endif
}

static inline uint16_t 
reg16_read(volatile uint16_t *reg)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return *reg;
#else
    return swap_u16(*reg);
#endif
}

/****************************************************************************/
/*
 *   8-bit register access functions
 */
/****************************************************************************/

static inline void 
reg8_clear_bits(volatile uint8_t *reg, uint8_t value)
{
    *reg &= ~(value);
}

static inline void 
reg8_set_bits(volatile uint8_t *reg, uint8_t value)
{
    *reg |= value;
}

static inline void 
reg8_toggle_bits(volatile uint8_t *reg, uint8_t value)
{
    *reg ^= value;
}

static inline void 
reg8_write_masked(volatile uint8_t *reg, uint8_t mask, uint8_t value)
{
    *reg = (*reg & (~mask)) | (value & mask);
}

static inline void 
reg8_write(volatile uint8_t *reg, uint8_t value)
{
    *reg = value;
}

static inline uint8_t 
reg8_read(volatile uint8_t *reg)
{
    return *reg;
}
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#else
#pragma GCC pop_options
#endif
#endif /* REG_UTILS */

