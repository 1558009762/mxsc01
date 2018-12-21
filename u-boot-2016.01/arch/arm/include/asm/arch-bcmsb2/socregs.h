/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
 
#ifndef __SOCREGS_H
#define __SOCREGS_H

#define ICFG_CHIP_ID_REG                                    0x18000000
#define ICFG_CHIP_REVISION_ID                               0x18000004
#define ChipcommonG_SMBus0_SMBus_Config                     0x18008000
#define ChipcommonG_SMBus1_SMBus_Config                     0x1800b000
#define CMIC_DEV_REV_ID                                     0x03210224
#define DMU_PCU_IPROC_STRAPS_CAPTURED                       0x1800f028
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_boot_dev_R     9
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_type_R    5
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_page_R    3
#define DMU_CRU_RESET                                       0x1800f200

#define NAND_IDM_IDM_IO_CONTROL_DIRECT                      0xf8105408
#define NAND_nand_flash_INTFC_STATUS                        0x18046014
#define NAND_ro_ctlr_ready                                  0x18046f10
#define NAND_nand_flash_CMD_ADDRESS                         0x1804600c
#define NAND_nand_flash_CMD_EXT_ADDRESS                     0x18046008
#define NAND_nand_flash_INIT_STATUS                         0x18046144
#define NAND_nand_flash_FLASH_DEVICE_ID                     0x18046194
#define NAND_nand_flash_ONFI_STATUS                         0x18046148
#define NAND_nand_flash_CONFIG_CS1                          0x18046064
#define NAND_nand_flash_CONFIG_CS0                          0x18046054
#define NAND_nand_flash_ACC_CONTROL_CS1                     0x18046060
#define NAND_nand_flash_ACC_CONTROL_CS0                     0x18046050
#define NAND_nand_flash_FLASH_CACHE0                        0x18046400
#define NAND_nand_flash_UNCORR_ERROR_COUNT                  0x180460fc
#define NAND_nand_flash_CORR_ERROR_COUNT                    0x18046100
#define NAND_nand_flash_ECC_CORR_EXT_ADDR                   0x1804610c
#define NAND_nand_flash_ECC_CORR_ADDR                       0x18046110
#define NAND_nand_flash_ECC_UNC_ADDR                        0x18046118
#define NAND_nand_flash_CS_NAND_SELECT                      0x18046018
#define NAND_nand_flash_SPARE_AREA_READ_OFS_0               0x18046200
#define NAND_nand_flash_SPARE_AREA_WRITE_OFS_0              0x18046280
#define NAND_nand_flash_FLASH_DEVICE_ID_EXT                 0x18046198
#define NAND_nand_flash_CMD_START                           0x18046004
#define NAND_IDM_IDM_RESET_CONTROL                          0xf8105800

#define QSPI_bspi_registers_REVISION_ID                     0x18047000
#define CRU_control                                         0x1800e000

#define ICFG_CHIP_ID_REG	                                0x18000000

#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL                   0x18000c8c
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel            1
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel_sw_ovwr    0
#define ICFG_PNOR_STRAPS                                    0x18000a5c
#define ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R                    0
#define PNOR_set_opmode                                     0x18045018
#define PNOR_set_opmode__set_mw_R                           0
#define PNOR_direct_cmd                                     0x18045010
#define PNOR_direct_cmd__cmd_type_R                         21

#define AMAC_IDM0_IO_CONTROL_DIRECT	                        0x18110408
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL            6
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE       5
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN      3
#define AMAC_IDM1_IO_CONTROL_DIRECT	                        0x1811f408
#define AMAC_IDM1_IO_CONTROL_DIRECT__CLK_250_SEL            6
#define AMAC_IDM1_IO_CONTROL_DIRECT__DIRECT_GMII_MODE       5
#define AMAC_IDM1_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN      3

#define AMAC_IDM0_IDM_RESET_CONTROL							0x18110800
#define AMAC_IDM1_IDM_RESET_CONTROL							0x1811f800

#define ChipcommonG_MII_Management_Control	                0x18002000
#define ChipcommonG_MII_Management_Command_Data             0x18002004
#define ChipcommonB_MII_Management_Control					ChipcommonG_MII_Management_Control
#define ChipcommonB_MII_Management_Command_Data				ChipcommonG_MII_Management_Command_Data
#define ChipcommonB_MII_Management_Control__BSY				8
#define ChipcommonB_MII_Management_Control__PRE				7
#define ChipcommonB_MII_Management_Control__EXT				9
#define ChipcommonB_MII_Management_Command_Data__SB_R		30
#define ChipcommonB_MII_Management_Command_Data__PA_R		23
#define ChipcommonB_MII_Management_Command_Data__RA_R		18
#define ChipcommonB_MII_Management_Command_Data__TA_R		16
#define ChipcommonB_MII_Management_Command_Data__OP_R		28

#define IPROC_WRAP_MISC_CONTROL                             0x1800fc40
#define IPROC_WRAP_MISC_CONTROL__IPROC_MDIO_SEL             1
#define IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL           3
#define IPROC_WRAP_MISC_STATUS                              0x1800fc44
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK             1

#define IPROC_WRAP_IPROC_STRAP_CTRL                         0x1800fc70
#define IPROC_WRAP_IPROC_STRAP_CTRL__DISABLE_USB2D          10

#define IPROC_WRAP_PCIE_SERDES_CONTROL                      0x1800fc48
#define IPROC_WRAP_PCIE_SERDES_CONTROL_REFCLK_IN_SEL        3

#define IPROC_WRAP_USBPHY_CTRL_0                            0x1800fc28
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO                   27
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB                25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB                    24
#define IPROC_WRAP_USBPHY_CTRL_5                            0x1800fc3c
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_R                   0

#define DMU_CRU_RESET                                       0x1800f200

#define DDR_S0_IDM_RESET_CONTROL                            0xf8101800
#define DDR_S1_IDM_IO_CONTROL_DIRECT                        0xf8102408
#define DDR_S1_IDM_IO_STATUS                                0xf8102500
#define DDR_S1_IDM_IO_STATUS__o_phy_ready                   2
#define DDR_S1_IDM_IO_STATUS__o_phy_pwrup_rsb               3
#define DDR_S1_IDM_RESET_CONTROL                            0xf8102800
#define DDR_S2_IDM_RESET_CONTROL                            0xf8103800
#define DDR_S2_IDM_IO_CONTROL_DIRECT                        0xf8103408
#define DDR_S2_IDM_IO_CONTROL_DIRECT__mode_32b              1
#define ROM_S0_IDM_IO_STATUS                                0xf8104500

#define DDR_BistConfig                                      0x18010c00
#define DDR_BistConfig__axi_port_sel                        1
#define DDR_BistGeneralConfigurations                       0x18010c08
#define DDR_BistConfigurations                              0x18010c0c
#define DDR_BistConfigurations__WriteWeight_R               0
#define DDR_BistConfigurations__ReadWeight_R                8
#define DDR_BistConfigurations__IndWrRdAddrMode             19
#define DDR_BistConfigurations__ConsAddr8Banks              21
#define DDR_BistConfigurations__BistEn                      25
#define DDR_BistNumberOfActions                             0x18010c10
#define DDR_BistStartAddress                                0x18010c14
#define DDR_BistEndAddress                                  0x18010c18
#define DDR_BistEndAddress__BistEndAddress_WIDTH            26
#define DDR_BistPatternWord7                                0x18010c20
#define DDR_BistPatternWord6                                0x18010c24
#define DDR_BistPatternWord5                                0x18010c28
#define DDR_BistPatternWord4                                0x18010c2c
#define DDR_BistPatternWord3                                0x18010c30
#define DDR_BistPatternWord2                                0x18010c34
#define DDR_BistPatternWord1                                0x18010c38
#define DDR_BistPatternWord0                                0x18010c3c
#define DDR_BistStatuses                                    0x18010c60
#define DDR_BistStatuses__BistFinished                      0
#define DDR_BistErrorOccurred                               0x18010c6c

#define DDR_DENALI_CTL_00                                   0x18010000
#define DDR_DENALI_CTL_146                                  0x18010248
#define DDR_DENALI_CTL_146__ECC_EN                          0
#define DDR_DENALI_CTL_148                                  0x18010250
#define DDR_DENALI_CTL_148__ECC_DISABLE_W_UC_ERR            0
#define DDR_DENALI_CTL_175                                  0x180102bc
#define DDR_DENALI_CTL_177                                  0x180102c4
#define DDR_DENALI_CTL_177_ECC_MASK                         0x78

#define CRU_ddrphy_pwr_ctrl                                 0x1800e024
#define CRU_ddrphy_pwr_ctrl__i_hw_reset_n                   5
#define CRU_ddrphy_pwr_ctrl__i_pwrokin_phy                  4
#define CRU_ddrphy_pwr_ctrl__i_pwronin_phy                  3
#define CRU_ddrphy_pwr_ctrl__i_iso_phy_dfi                  2
#define CRU_ddrphy_pwr_ctrl__i_iso_phy_regs                 1
#define CRU_ddrphy_pwr_ctrl__i_iso_phy_pll                  0

#define DDR_PHY_CONTROL_REGS_DRIVE_PAD_CTL                  0x18011038
#define DDR_PHY_CONTROL_REGS_STATIC_PAD_CTL                 0x18011040
#define DDR_PHY_CONTROL_REGS_DRAM_CONFIG	                0x18011044
#define DDR_PHY_CONTROL_REGS_VDL_CALIB_STATUS1              0x1801105c
#define DDR_PHY_CONTROL_REGS_VDL_CONTROL_PAR                0x180110e0
#define DDR_PHY_BYTE_LANE_0_VDL_CONTROL_RD_EN_CS0           0x1801147c
#define DDR_PHY_BYTE_LANE_0_VDL_CONTROL_RD_EN_CS1           0x18011480
#define DDR_PHY_BYTE_LANE_0_RD_EN_DLY_CYC                   0x1801148c
#define DDR_PHY_BYTE_LANE_0_READ_CONTROL	                0x18011494
#define DDR_PHY_BYTE_LANE_0_DRIVE_PAD_CTL                   0x180114b0
#define DDR_PHY_BYTE_LANE_0_DQSP_DRIVE_PAD_CTL              0x180114b4
#define DDR_PHY_BYTE_LANE_0_DQSN_DRIVE_PAD_CTL              0x180114b8
#define DDR_PHY_BYTE_LANE_0_RD_EN_DRIVE_PAD_CTL             0x180114c0
#define DDR_PHY_BYTE_LANE_0_WR_PREAMBLE_MODE                0x180114c8
#define DDR_PHY_CONTROL_REGS_VDL_CALIBRATE	                0x18011058
#define DDR_PHY_CONTROL_REGS_VDL_CONTROL_CKE                0x180110ec
#define DDR_PHY_CONTROL_REGS_STANDBY_CONTROL	              0x180111d0
#define DDR_PHY_CONTROL_REGS_DFI_CNTRL                      0x180111dc
#define DDR_PHY_CONTROL_REGS_VREF_DAC_CONTROL               0x1801119c
#define DDR_PHY_CONTROL_REGS_REVISION                       0x18011000
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS                   0x18011018
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__MDIV_R           20
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__MDIV_WIDTH       8
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__PDIV_R           12
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__PDIV_WIDTH       4
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__NDIV_INT_R       0
#define DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__NDIV_INT_WIDTH   10

#define ICFG_ROM_STRAPS                                     0x18000aa0
#define IPROC_WRAP_IPROC_DDR_PLL_CTRL_0                     0x1800FC00
#define IPROC_DDR_PLL_SW_OVWR                               26
#define IPROC_DDR_PLL_POST_RESETB                           25
#define IPROC_DDR_PLL_RESETB                                24
#define IPROC_WRAP_IPROC_DDR_PLL_CTRL_1                     0x1800FC04
#define IPROC_WRAP_IPROC_DDR_PLL_CTRL_2                     0x1800FC08
#define IPROC_WRAP_IPROC_DDR_PLL_CTRL_3                     0x1800FC0C
#define IPROC_WRAP_IPROC_DDR_PLL_CTRL_6                     0x1800FC18
#define IPROC_WRAP_IPROC_DDR_PLL_STATUS                     0x1800FC24
#define IPROC_DDR_PLL_LOCK                                  12
#define IPROC_WRAP_IPROC_PLL_CTRL_0                         0x1800fc50
#define IPROC_WRAP_IPROC_PLL_CTRL_0__FAST_LOCK              27
#define IPROC_WRAP_IPROC_PLL_CTRL_1                         0x1800fc54
#define IPROC_WRAP_IPROC_PLL_CTRL_1__PDIV_R                 27
#define IPROC_WRAP_IPROC_PLL_CTRL_1__PDIV_WIDTH             4
#define IPROC_WRAP_IPROC_PLL_CTRL_1__NDIV_FRAC_R            7
#define IPROC_WRAP_IPROC_PLL_CTRL_1__NDIV_FRAC_WIDTH	    20
#define IPROC_WRAP_IPROC_PLL_CTRL_2                         0x1800fc58
#define IPROC_WRAP_IPROC_PLL_CTRL_3                         0x1800fc5c
#define IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_R             20
#define IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_WIDTH         10
#define IPROC_WRAP_IPROC_PLL_CTRL_4                         0x1800fc60
#define IPROC_WRAP_IPROC_PLL_CTRL_5                         0x1800fc64
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_R	            8
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_WIDTH	        8

#define IHOST_SCU_INVALIDATE_ALL                            0x1902000c
#define IHOST_SCU_CONTROL                                   0x19020000
#define IHOST_L2C_CACHE_ID                                  0x19022000

#define GMAC0_DEVCONTROL                                    0x18042000
#define GMAC1_DEVCONTROL                                    0x1804a000

/* USB */
#define USB2_IDM_IDM_IO_CONTROL_DIRECT                      0x18115408
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__Bypass_CT           29
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__CT                  28
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__arcache_R           6
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__awcache_R           2
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable          0
#define USB2_IDM_IDM_RESET_CONTROL                          0x18115800
#define USB2_IDM_IDM_RESET_CONTROL__RESET                   0

#define USBH_HCCAPBASE                                      0x18048000
#define USBH_Phy_Ctrl_P0                                    0x18049200
#define USBH_Phy_Ctrl_P0__PHY_PLL_Power_Down_R              0
#define USBH_Phy_Ctrl_P0__PHY_Test_port_UTMI_Power_Down_R   2
#define USBH_Phy_Ctrl_P0__PHY_Test_port_Power_Down_R        4
#define USBH_Phy_Ctrl_P0__PHY_Soft_Reset_R                  6
#define USBH_Phy_Ctrl_P0__Core_Reset                        8
#define USBH_Phy_Ctrl_P0__Phy_Hard_Reset                    9


/* GPIO */
#define ChipcommonG_GP_DATA_OUT                             0x1800a004
#define ChipcommonG_GP_OUT_EN                               0x1800a008

/* SRAB */
#define ChipcommonG_SRAB_CMDSTAT                            0x1800702c
#define ChipcommonG_SRAB_WDH                                0x18007030
#define ChipcommonG_SRAB_WDL                                0x18007034
#define ChipcommonG_SRAB_RDH                                0x18007038
#define ChipcommonG_SRAB_RDL                                0x1800703c
#define ChipcommonG_SRAB_SW_IF                              0x18007040
 
#define PAXB_0_STRAP_STATUS                                 0x18012f10
#define PAXB_0_STRAP_STATUS_BASE                                 0xf10
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_REPLAY_BUF_TM_L 13
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_REPLAY_BUF_TM_R 4
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_REPLAY_BUF_TM_WIDTH 10
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_REPLAY_BUF_TM_RESETVALUE 0x0
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_GEN1 3
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_GEN1_WIDTH 1
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_GEN1_RESETVALUE 0x0
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_1LANE 2
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_1LANE_WIDTH 1
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_FORCE_1LANE_RESETVALUE 0x0
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_IF_ENABLE 1
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_IF_ENABLE_WIDTH 1
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_IF_ENABLE_RESETVALUE 0x0
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_RC_MODE 0
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_RC_MODE_WIDTH 1
#define PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_RC_MODE_RESETVALUE 0x0

#define CMIC_SBUS_RING_MAP_0_7                              0x03210098
#define CMIC_SBUS_RING_MAP_8_15                             0x0321009C
#define CMIC_SBUS_RING_MAP_16_23                            0x032100A0
#define CMIC_SBUS_RING_MAP_24_31                            0x032100A4
#define CMIC_COMMON_UC0_PIO_ENDIANESS                       0x032101f0
#define CMIC_CMC0_SCHAN_CTRL                                0x03231000
#define CMIC_CMC0_SCHAN_MESSAGE0                            0x0323100C
#define CMIC_CMC0_SCHAN_MESSAGE1                            0x03231010
#define CMIC_CMC0_SCHAN_MESSAGE2                            0x03231014

#define CMIC_OVERRIDE_STRAP  0x3210234
#define CMIC_OVERRIDE_STRAP__ENABLE_OVERRIDE_I2C_MASTER_SLAVE_MODE 4
#define CMIC_OVERRIDE_STRAP__I2C_MASTER_SLAVE_MODE 1

#define TOP_LCPLL_SOFT_RESET_REG                            0x02001400
#define TOP_XGXS0_PLL_CONTROL_2                             0x02001900
#define TOP_XGXS0_PLL_CONTROL_3                             0x02001a00
#define TOP_XGXS0_PLL_CONTROL_4                             0x02001b00
#define TOP_XGXS0_PLL_CONTROL_7                             0x02001e00
#define TOP_XGXS1_PLL_CONTROL_1                             0x02002000
#define TOP_XGXS1_PLL_CONTROL_2                             0x02002100
#define TOP_XGXS1_PLL_CONTROL_3                             0x02002200
#define TOP_XGXS1_PLL_CONTROL_4                             0x02002300
#define TOP_XGXS1_PLL_CONTROL_7                             0x02002600
#define TOP_CORE_PLL0_CTRL_REGISTER_5                       0x02003500
#define TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_2                 0x02004800
#define TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_3                 0x02004900
#define TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_4                 0x02004a00
#define TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_6                 0x02004c00
#define TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_2                 0x02005100
#define TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_3                 0x02005200
#define TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_4                 0x02005300
#define TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_6                 0x02005500
#define TOP_MASTER_LCPLL_FBDIV_CTRL_1                       0x02009100
#define TOP_SERDES_LCPLL_FBDIV_CTRL_1                       0x02009300
#define TOP_BROAD_SYNC0_LCPLL_FBDIV_CTRL_1                  0x02009500
#define TOP_BROAD_SYNC1_LCPLL_FBDIV_CTRL_1                  0x02009700
#define TOP_UC_TAP_CONTROL                                  0x0202c000
#define TOP_UC_TAP_WRITE_DATA                               0x0202c100
#define TOP_MISC_CONTROL_3                                  0x02007300
#define TOP_SWITCH_FEATURE_ENABLE_3                         0x0200A400
#define USB_DEV_MODE_STRAP                                  24

#define IPROC_QSPI_MEM_BASE             (0xF0000000)
#define IPROC_NAND_MEM_BASE             (0xE0000000)

#endif /* __SOCREGS_H */
