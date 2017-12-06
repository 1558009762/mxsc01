/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __DDR40_PHY_CUSTOM_H__

#define __DDR40_PHY_CUSTOM_H__  

// #ifdef MIPS_CODE
#if 0
//all print/error messages and options parsing is disabled
  #define  DDR40_PHY_RegWr(addr,value)   IO(addr) = value 
  #define  DDR40_PHY_RegRd(addr)         IO(addr)
  #define  DDR40_PHY_Print(args...)    
  #define  DDR40_PHY_Fatal(args...)    
  #define  DDR40_PHY_Timeout(args...)  
  typedef unsigned int32_t  uint32;
  typedef unsigned char uint8;
  
  #include "chip.h"
  #include "chip_global_addr.h"
  #include "fgx_hw_util.h"

#else

#ifndef DDR40_TYPES__
#define DDR40_TYPES__
typedef unsigned long   ddr40_addr_t;
typedef u32             ddr40_uint32_t;
#endif

  #define  DDR40_PHY_RegWr(addr,value)   tb_w(addr,value)
  #define  DDR40_PHY_RegRd(addr)         tb_r(addr)

  #define  DDR40_PHY_Print(args...)      print_log(args)
  #define  DDR40_PHY_Error(args...)      error_log(args)
  #define  DDR40_PHY_Fatal(args...)      fatal_log(args)

  #define  DDR40_PHY_Timeout(args...)      timeout_ns(args)

  #define FUNC_PROTOTYPE_PREFIX inline
  #define FUNC_PROTOTYPE_SUFFIX
  #define FUNC_PREFIX inline
  #define FUNC_SUFFIX

#endif

#endif


/*
**
** $Log: $
**
*/
