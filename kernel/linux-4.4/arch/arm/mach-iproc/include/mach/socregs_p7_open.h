/*
 * $Copyright Open Broadcom Corporation$
 */


#ifndef __SOCREGS_P7_OPEN_H
#define __SOCREGS_P7_OPEN_H

#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_GH2)

#define DMU_PCU_IPROC_CONTROL                               0x1800f000
#define DMU_CRU_RESET_BASE                                  0x200
#define DMU_PCU_IPROC_STRAPS_CAPTURED_BASE                  0x028

#if defined(CONFIG_MACH_GH)
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_boot_dev_R     9
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_type_R    5
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_page_R    3
#else
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_boot_dev_R     10
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_type_R    6
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_page_R    4
#endif

#define IPROC_WRAP_GEN_PLL_STATUS__GEN_PLL_LOCK             0
#define IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_R                0
#define IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_WIDTH            10
#define IPROC_WRAP_GEN_PLL_CTRL1__PDIV_R                    10
#define IPROC_WRAP_GEN_PLL_CTRL1__PDIV_WIDTH                4
#define IPROC_WRAP_GEN_PLL_CTRL1__CH0_MDIV_R                14
#define IPROC_WRAP_GEN_PLL_CTRL1__CH1_MDIV_R                22
#define IPROC_WRAP_GEN_PLL_CTRL2__CH2_MDIV_R                0
#define IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R                8
#define IPROC_WRAP_GEN_PLL_CTRL2__CH4_MDIV_R                16

#define IPROC_WRAP_TOP_STRAP_STATUS                         0x1800fca4
#define IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL               17
#define IPROC_WRAP_TOP_STRAP_STATUS_1                       0x1800fca8

#define IPROC_WRAP_USBPHY_CTRL_0                            0x1800fc44
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ                  26
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB                25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB                    24
#define IPROC_WRAP_USBPHY_CTRL_2                            0x1800fc4c
#define IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO                   17
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0                  0
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11                 11
#define IPROC_WRAP_MISC_STATUS                              0x1800fc58
#define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG          2
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK             1

#endif /* End of Greyhound only registers */


#if defined(CONFIG_MACH_SB2)

#define DMU_PCU_IPROC_CONTROL                               0x1800f000
#define DMU_CRU_RESET_BASE                                  0x200

#define DMU_PCU_IPROC_STRAPS_CAPTURED_BASE                  0x028
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_boot_dev_R     10
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_type_R    6
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_page_R    4

#define IPROC_WRAP_IPROC_PLL_STATUS__IPROC_PLL_LOCK         12
#define IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_R             20
#define IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_WIDTH         10
#define IPROC_WRAP_IPROC_PLL_CTRL_1__PDIV_R                 27
#define IPROC_WRAP_IPROC_PLL_CTRL_1___PDIV_WIDTH            4
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH3_MDIV_R             24
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH3_MDIV_WITH          8
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH2_MDIV_R             16
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH2_MDIV_WITH          8
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_R             8
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_WITH          8
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH0_MDIV_R             0
#define IPROC_WRAP_IPROC_PLL_CTRL_5__CH0_MDIV_WITH          8
#define IPROC_WRAP_IPROC_PLL_CTRL_6__CH5_MDIV_R             8
#define IPROC_WRAP_IPROC_PLL_CTRL_6__CH5_MDIV_WITH          8
#define IPROC_WRAP_IPROC_PLL_CTRL_6__CH4_MDIV_R             0
#define IPROC_WRAP_IPROC_PLL_CTRL_6__CH4_MDIV_WITH          8

#define AMAC_IDM1_IO_CONTROL_DIRECT                         0x1811f408
#define AMAC_IDM1_IO_CONTROL_DIRECT__CLK_250_SEL            6
#define AMAC_IDM1_IO_CONTROL_DIRECT__DIRECT_GMII_MODE       5
#define AMAC_IDM1_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN      3

#define IPROC_WRAP_MISC_CONTROL                             0x1800fc40
#define IPROC_WRAP_MISC_CONTROL__IPROC_MDIO_SEL             1
#define IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL1          3
#define IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL2          4
#define IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL3          5

#define IPROC_WRAP_USBPHY_CTRL_0                            0x1800fc28
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO                   27
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB                25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB                    24
#define IPROC_WRAP_USBPHY_CTRL_2                            0x1800fc30
#define IPROC_WRAP_USBPHY_CTRL_5                            0x1800fc3c
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B0                  0
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B11                 11
#define IPROC_WRAP_MISC_STATUS                              0x1800fc44
#define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG          2
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK             1

#define IPROC_WRAP_TOP_STRAP_CTRL                           0x1800fc70
#define IPROC_WRAP_TOP_STRAP_CTRL__USB_DEVICE               10

/* Regsters required by Pl022 SPI */
#define ChipcommonG_SPI2_SSPCR0                             0x1802a000
#endif /* End of Saber2 only registers */


#define ICFG_CHIP_ID_REG                                    0x18000000
#define ChipcommonG_MII_Management_Control                  0x18002000
#define ChipcommonG_SMBus0_SMBus_Config                     0x18008000
#define ChipcommonG_UART0_UART_RBR_THR_DLL                  0x18020000
#define ChipcommonG_UART1_UART_RBR_THR_DLL                  0x18021000

#define QSPI_mspi_SPCR0_LSB                                 0x18047200
#define QSPI_mspi_DISABLE_FLUSH_GEN                         0x18047384
#define QSPI_bspi_registers_REVISION_ID                     0x18047000
#define QSPI_bspi_registers_BSPI_PIO_DATA                   0x1804704c
#define QSPI_raf_START_ADDR                                 0x18047100
#define QSPI_raf_CURR_ADDR                                  0x18047120
#define QSPI_raf_interrupt_LR_fullness_reached              0x180473a0
#define QSPI_mspi_interrupt_MSPI_halt_set_transaction_done  0x180473b8

#if defined(CONFIG_MACH_GH2)
#define QSPI_IDM_IDM_IO_CONTROL_DIRECT                      0xf8106408
#else
#define QSPI_IDM_IDM_IO_CONTROL_DIRECT                      0x1811c408
#endif
#define CRU_control                                         0x1800e000

#define NAND_nand_flash_REVISION                            0x18046000
#define NAND_direct_read_rd_miss                            0x18046f00
#define NAND_IDM_IDM_IO_CONTROL_DIRECT                      0xf8105408

#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL                   0x18000c8c
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel   1
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel_sw_ovwr 0
#define ICFG_PNOR_STRAPS                                    0x18000a5c
#define ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R                    0
#define PNOR_set_opmode                                     0x18045018
#define PNOR_set_opmode__set_mw_R                           0
#define PNOR_direct_cmd                                     0x18045010
#define PNOR_direct_cmd__cmd_type_R                         21

#define ChipcommonG_tim0_TIM_TIMER1Load                     0x18003000
#define ChipcommonG_tim1_TIM_TIMER1Load                     0x18004000
#define ChipcommonG_tim2_TIM_TIMER1Load                     0x18005000
#define ChipcommonG_tim3_TIM_TIMER1Load                     0x18006000
#define ChipcommonG_SMBus0_SMBus_Config                     0x18008000
#define ChipcommonG_SMBus1_SMBus_Config                     0x1800b000

#define ChipcommonS_RNG_CTRL                                0x18032000

#define USBH_Phy_Ctrl_P0                                    0x18049200
#define USBH_Phy_Ctrl_P1                                    0x18049204

#define IHOST_S0_IDM_ERROR_LOG_CONTROL                      0x18107900
#define IHOST_S0_IDM_ERROR_LOG_COMPLETE                     0x18107904
#define IHOST_S0_IDM_ERROR_LOG_STATUS                       0x18107908
#define IHOST_S0_IDM_ERROR_LOG_ADDR_LSB                     0x1810790c
#define IHOST_S0_IDM_ERROR_LOG_ID                           0x18107914
#define IHOST_S0_IDM_ERROR_LOG_FLAGS                        0x1810791c
#define IHOST_S0_IDM_INTERRUPT_STATUS                       0x18107a00

#define IHOST_S1_IDM_ERROR_LOG_CONTROL                      0x18106900
#define IHOST_S1_IDM_ERROR_LOG_COMPLETE                     0x18106904
#define IHOST_S1_IDM_ERROR_LOG_STATUS                       0x18106908
#define IHOST_S1_IDM_ERROR_LOG_ADDR_LSB                     0x1810690c
#define IHOST_S1_IDM_ERROR_LOG_ID                           0x18106914
#define IHOST_S1_IDM_ERROR_LOG_FLAGS                        0x1810691c
#define IHOST_S1_IDM_INTERRUPT_STATUS                       0x18106a00

#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL               0x18108900
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_COMPLETE              0x18108904
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS                0x18108908
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ADDR_LSB              0x1810890c
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ID                    0x18108914
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_FLAGS                 0x1810891c
#define AXI_PCIE_S0_IDM_IDM_INTERRUPT_STATUS                0x18108a00

#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_CONTROL               0x18109900
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_COMPLETE              0x18109904
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_STATUS                0x18109908
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_ADDR_LSB              0x1810990c
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_ID                    0x18109914
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_FLAGS                 0x1810991c
#define AXI_PCIE_S1_IDM_IDM_INTERRUPT_STATUS                0x18109a00

#define CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL                  0x1810a900
#define CMICD_S0_IDM_IDM_ERROR_LOG_COMPLETE                 0x1810a904
#define CMICD_S0_IDM_IDM_ERROR_LOG_STATUS                   0x1810a908
#define CMICD_S0_IDM_IDM_ERROR_LOG_ADDR_LSB                 0x1810a90c
#define CMICD_S0_IDM_IDM_ERROR_LOG_ID                       0x1810a914
#define CMICD_S0_IDM_IDM_ERROR_LOG_FLAGS                    0x1810a91c
#define CMICD_S0_IDM_IDM_INTERRUPT_STATUS                   0x1810aa00

#define USB2_IDM_IDM_IO_CONTROL_DIRECT                      0x18115408
#define USB2_IDM_IDM_IO_STATUS                              0x18115500
#define USB2_IDM_IDM_RESET_CONTROL                          0x18115800
#define USB2_IDM_IDM_RESET_STATUS                           0x18115804
#define USB2_IDM_IDM_INTERRUPT_STATUS                       0x18115a00

#define USB2D_IDM_IDM_IO_CONTROL_DIRECT                     0x18111408
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable         0
#define USB2D_IDM_IDM_RESET_CONTROL                         0x18111800
#define USB2D_IDM_IDM_RESET_CONTROL__RESET                  0

#define A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL                 0x18119900
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_COMPLETE                0x18119904
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS                  0x18119908
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ADDR_LSB                0x1811990c
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ID                      0x18119914
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_FLAGS                   0x1811991c
#define A9JTAG_S0_IDM_IDM_INTERRUPT_STATUS                  0x18119a00

#define SRAM_S0_IDM_ERROR_LOG_CONTROL                       0x1811b900
#define SRAM_S0_IDM_ERROR_LOG_COMPLETE                      0x1811b904
#define SRAM_S0_IDM_ERROR_LOG_STATUS                        0x1811b908
#define SRAM_S0_IDM_ERROR_LOG_ADDR_LSB                      0x1811b90c
#define SRAM_S0_IDM_ERROR_LOG_ID                            0x1811b914
#define SRAM_S0_IDM_ERROR_LOG_FLAGS                         0x1811b91c
#define SRAM_S0_IDM_INTERRUPT_STATUS                        0x1811ba00

#define APBX_IDM_IDM_ERROR_LOG_CONTROL                      0x18130900
#define APBX_IDM_IDM_ERROR_LOG_COMPLETE                     0x18130904
#define APBX_IDM_IDM_ERROR_LOG_STATUS                       0x18130908
#define APBX_IDM_IDM_ERROR_LOG_ADDR_LSB                     0x1813090c
#define APBX_IDM_IDM_ERROR_LOG_ID                           0x18130914
#define APBX_IDM_IDM_ERROR_LOG_FLAGS                        0x1813091c
#define APBX_IDM_IDM_INTERRUPT_STATUS                       0x18130a00

#define APBY_IDM_IDM_ERROR_LOG_CONTROL                      0x18131900
#define APBY_IDM_IDM_ERROR_LOG_COMPLETE                     0x18131904
#define APBY_IDM_IDM_ERROR_LOG_STATUS                       0x18131908
#define APBY_IDM_IDM_ERROR_LOG_ADDR_LSB                     0x1813190c
#define APBY_IDM_IDM_ERROR_LOG_ID                           0x18131914
#define APBY_IDM_IDM_ERROR_LOG_FLAGS                        0x1813191c
#define APBY_IDM_IDM_INTERRUPT_STATUS                       0x18131a00

#define APBZ_IDM_IDM_ERROR_LOG_CONTROL                      0x18132900
#define APBZ_IDM_IDM_ERROR_LOG_COMPLETE                     0x18132904
#define APBZ_IDM_IDM_ERROR_LOG_STATUS                       0x18132908
#define APBZ_IDM_IDM_ERROR_LOG_ADDR_LSB                     0x1813290c
#define APBZ_IDM_IDM_ERROR_LOG_ID                           0x18132914
#define APBZ_IDM_IDM_ERROR_LOG_FLAGS                        0x1813291c
#define APBZ_IDM_IDM_INTERRUPT_STATUS                       0x18132a00

#define DDR_S1_IDM_ERROR_LOG_CONTROL                        0xf8102900
#define DDR_S1_IDM_ERROR_LOG_COMPLETE                       0xf8102904
#define DDR_S1_IDM_ERROR_LOG_STATUS                         0xf8102908
#define DDR_S1_IDM_ERROR_LOG_ADDR_LSB                       0xf810290c
#define DDR_S1_IDM_ERROR_LOG_ID                             0xf8102914
#define DDR_S1_IDM_ERROR_LOG_FLAGS                          0xf810291c
#define DDR_S1_IDM_INTERRUPT_STATUS                         0xf8102a00

#define DDR_S2_IDM_ERROR_LOG_CONTROL                        0xf8103900
#define DDR_S2_IDM_ERROR_LOG_COMPLETE                       0xf8103904
#define DDR_S2_IDM_ERROR_LOG_STATUS                         0xf8103908
#define DDR_S2_IDM_ERROR_LOG_ADDR_LSB                       0xf810390c
#define DDR_S2_IDM_ERROR_LOG_ID                             0xf8103914
#define DDR_S2_IDM_ERROR_LOG_FLAGS                          0xf810391c
#define DDR_S2_IDM_INTERRUPT_STATUS                         0xf8103a00

#define ROM_S0_IDM_ERROR_LOG_CONTROL                        0xf8104900
#define ROM_S0_IDM_ERROR_LOG_COMPLETE                       0xf8104904
#define ROM_S0_IDM_ERROR_LOG_STATUS                         0xf8104908
#define ROM_S0_IDM_ERROR_LOG_ADDR_LSB                       0xf810490c
#define ROM_S0_IDM_ERROR_LOG_ID                             0xf8104914
#define ROM_S0_IDM_ERROR_LOG_FLAGS                          0xf810491c
#define ROM_S0_IDM_INTERRUPT_STATUS                         0xf8104a00

#define NAND_IDM_IDM_ERROR_LOG_CONTROL                      0xf8105900
#define NAND_IDM_IDM_ERROR_LOG_COMPLETE                     0xf8105904
#define NAND_IDM_IDM_ERROR_LOG_STATUS                       0xf8105908
#define NAND_IDM_IDM_ERROR_LOG_ADDR_LSB                     0xf810590c
#define NAND_IDM_IDM_ERROR_LOG_ID                           0xf8105914
#define NAND_IDM_IDM_ERROR_LOG_FLAGS                        0xf810591c
#define NAND_IDM_IDM_INTERRUPT_STATUS                       0xf8105a00

#define QSPI_IDM_IDM_ERROR_LOG_CONTROL                      0xf8106900
#define QSPI_IDM_IDM_ERROR_LOG_COMPLETE                     0xf8106904
#define QSPI_IDM_IDM_ERROR_LOG_STATUS                       0xf8106908
#define QSPI_IDM_IDM_ERROR_LOG_ADDR_LSB                     0xf810690c
#define QSPI_IDM_IDM_ERROR_LOG_ID                           0xf8106914
#define QSPI_IDM_IDM_ERROR_LOG_FLAGS                        0xf810691c
#define QSPI_IDM_IDM_INTERRUPT_STATUS                       0xf8106a00

#define AXIIC_DS_0_IDM_ERROR_LOG_CONTROL                    0x18120900
#define AXIIC_DS_0_IDM_ERROR_LOG_COMPLETE                   0x18120904
#define AXIIC_DS_0_IDM_ERROR_LOG_STATUS                     0x18120908
#define AXIIC_DS_0_IDM_ERROR_LOG_ADDR_LSB                   0x1812090c
#define AXIIC_DS_0_IDM_ERROR_LOG_ID                         0x18120914
#define AXIIC_DS_0_IDM_ERROR_LOG_FLAGS                      0x1812091c
#define AXIIC_DS_0_IDM_INTERRUPT_STATUS                     0x18120a00

#define AXIIC_DS_1_IDM_ERROR_LOG_CONTROL                    0x18121900
#define AXIIC_DS_1_IDM_ERROR_LOG_COMPLETE                   0x18121904
#define AXIIC_DS_1_IDM_ERROR_LOG_STATUS                     0x18121908
#define AXIIC_DS_1_IDM_ERROR_LOG_ADDR_LSB                   0x1812190c
#define AXIIC_DS_1_IDM_ERROR_LOG_ID                         0x18121914
#define AXIIC_DS_1_IDM_ERROR_LOG_FLAGS                      0x1812191c
#define AXIIC_DS_1_IDM_INTERRUPT_STATUS                     0x18121a00

#if defined(CONFIG_MACH_GH2)
#define AXIIC_DS_2_IDM_ERROR_LOG_CONTROL                    0x18122900
#define AXIIC_DS_2_IDM_ERROR_LOG_COMPLETE                   0x18122904
#define AXIIC_DS_2_IDM_ERROR_LOG_STATUS                     0x18122908
#define AXIIC_DS_2_IDM_ERROR_LOG_ADDR_LSB                   0x1812290c
#define AXIIC_DS_2_IDM_ERROR_LOG_ID                         0x18122914
#define AXIIC_DS_2_IDM_ERROR_LOG_FLAGS                      0x1812291c
#define AXIIC_DS_2_IDM_INTERRUPT_STATUS                     0x18122a00
#else
#define AXIIC_DS_2_IDM_ERROR_LOG_CONTROL                    0x1811d900
#define AXIIC_DS_2_IDM_ERROR_LOG_COMPLETE                   0x1811d904
#define AXIIC_DS_2_IDM_ERROR_LOG_STATUS                     0x1811d908
#define AXIIC_DS_2_IDM_ERROR_LOG_ADDR_LSB                   0x1811d90c
#define AXIIC_DS_2_IDM_ERROR_LOG_ID                         0x1811d914
#define AXIIC_DS_2_IDM_ERROR_LOG_FLAGS                      0x1811d91c
#define AXIIC_DS_2_IDM_INTERRUPT_STATUS                     0x1811da00
#endif

#define AXIIC_DS_3_IDM_ERROR_LOG_CONTROL                    0x1811e900
#define AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE                   0x1811e904
#define AXIIC_DS_3_IDM_ERROR_LOG_STATUS                     0x1811e908
#define AXIIC_DS_3_IDM_ERROR_LOG_ADDR_LSB                   0x1811e90c
#define AXIIC_DS_3_IDM_ERROR_LOG_ID                         0x1811e914
#define AXIIC_DS_3_IDM_ERROR_LOG_FLAGS                      0x1811e91c
#define AXIIC_DS_3_IDM_INTERRUPT_STATUS                     0x1811ea00

/* AMAC */
#define GMAC0_DEVCONTROL                                    0x18042000
#define AMAC_IDM0_IO_CONTROL_DIRECT                         0x18110408
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL            6
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE       5
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN      3

/* Use define if socregs_ing_open.h which may need modification */
#if defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_GH2)
#define GMAC1_DEVCONTROL                                    0x1804a000
#endif

/* GPIO */
#define ChipcommonG_GP_DATA_IN                              0x1800a000
#define ChipcommonG_GP_DATA_IN_BASE                         0x000
#define ChipcommonG_GP_DATA_OUT_BASE                        0x004
#define ChipcommonG_GP_OUT_EN_BASE                          0x008
#define ChipcommonG_GP_INT_TYPE_BASE                        0x00c
#define ChipcommonG_GP_INT_DE_BASE                          0x010
#define ChipcommonG_GP_INT_EDGE_BASE                        0x014
#define ChipcommonG_GP_INT_MSK_BASE                         0x018
#define ChipcommonG_GP_INT_STAT_BASE                        0x01c
#define ChipcommonG_GP_INT_MSTAT_BASE                       0x020
#define ChipcommonG_GP_INT_CLR_BASE                         0x024
#define ChipcommonG_GP_AUX_SEL_BASE                         0x028
#define ChipcommonG_GP_INIT_VAL_BASE                        0x030
#define ChipcommonG_GP_PAD_RES_BASE                         0x034
#define ChipcommonG_GP_RES_EN_BASE                          0x038
#define ChipcommonG_GP_TEST_INPUT_BASE                      0x03c
#define ChipcommonG_GP_TEST_OUTPUT_BASE                     0x040
#define ChipcommonG_GP_TEST_ENABLE_BASE                     0x044
#define ChipcommonG_GP_PRB_ENABLE_BASE                      0x048
#define ChipcommonG_GP_PRB_OE_BASE                          0x04c

/* Watchdog */
#define ChipcommonG_WDT_WDOGLOAD                            0x18009000
#define DMU_PCU_CRU_RESET_REASON                            0x1800f014
#define DMU_PCU_CRU_RESET_REASON__watchdog_reset            0

/* USBD */
#define USB2D_ENDPNT_IN_CTRL_0                              0x1804c000

#endif /* __SOCREGS_P7_OPEN_H */
