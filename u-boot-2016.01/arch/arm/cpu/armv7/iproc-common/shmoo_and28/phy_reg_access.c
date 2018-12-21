/***************************************************************************
 * $Copyright Broadcom Corporation Dual License $
 *
 ***************************************************************************/
 
#include <common.h>
#include "phy_reg_access.h"

uint32 REGRD (uint32 address) {

  volatile unsigned long data;
  
  data = cpu_to_le32(* (volatile uint32 *) ( ((uint32)GLOBAL_REG_RBUS_START) | (address)));
  //printf("REGRD %08X=%08X\n", address, data);
  return data;
}

uint32 REGWR (uint32 address, uint32 data) {

  ((* (volatile uint32 *) ( ((uint32)GLOBAL_REG_RBUS_START) | (address))) = cpu_to_le32(data));
  //printf("REGWR %08X=%08X\n", address, data);
//  return SOC_E_NONE;
   return 0;
}
