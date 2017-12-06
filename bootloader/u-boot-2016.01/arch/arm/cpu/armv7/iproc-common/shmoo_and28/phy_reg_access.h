/***************************************************************************
 * $Copyright Broadcom Corporation Dual License $
 *
 ***************************************************************************/
 
#define GLOBAL_REG_RBUS_START   0x0000000000000000LL
#define uint32                  unsigned long

uint32 REGRD (uint32 address) ;
uint32 REGWR (uint32 address, uint32 data) ;

#define DDR_PHY_REG_READ(_unit, _pc, flags, _reg_addr, _val) \
            *(uint32 *)_val = REGRD((_pc) + (_reg_addr))
/*            soc_ddr40_phy_reg_ci_read((_unit), (_pc), (_reg_addr), (_val)) */
#define DDR_PHY_REG_WRITE(_unit, _pc, _flags, _reg_addr, _val) \
            REGWR((_pc) + (_reg_addr), (_val))
/*            soc_ddr40_phy_reg_ci_write((_unit), (_pc), (_reg_addr), (_val)) */
#define DDR_PHY_REG_MODIFY(_unit, _pc, _flags, _reg_addr, _val, _mask) \
            REGWR((_pc) + (_reg_addr), (REGRD((_pc) + (_reg_addr)) & ~(_mask)) | ((_val) & (_mask)))
/*            soc_ddr40_phy_reg_ci_modify((_unit), (_pc), (_reg_addr), (_val), (_mask)) */
#define DDR_PHY_GET_FIELD(m,c,r,f) \
            GET_FIELD(m,c,r,f)
#define DDR_PHY_SET_FIELD(m,c,r,f,d) \
            SET_FIELD(m,c,r,f,d)
