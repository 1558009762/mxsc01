/*
 * $Copyright Open Broadcom Corporation$
 */


#ifndef __SOCREGS_P10_OPEN_H
#define __SOCREGS_P10_OPEN_H


#if defined(CONFIG_MACH_HR3)
/*
 * Hurricane3 only registers 
 */
#define DMU_PCU_IPROC_CONTROL 0x1800f000
#define DMU_CRU_RESET_BASE 0x200

#define DMU_PCU_IPROC_STRAPS_CAPTURED_BASE 0x028
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_boot_dev_R 10
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_type_R 6
#define DMU_PCU_IPROC_STRAPS_CAPTURED__strap_nand_page_R 4

#define IPROC_WRAP_GEN_PLL_STATUS__GEN_PLL_LOCK 0
#define IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_R 0
#define IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_WIDTH 10
#define IPROC_WRAP_GEN_PLL_CTRL1__PDIV_R 10
#define IPROC_WRAP_GEN_PLL_CTRL1__PDIV_WIDTH 4
#define IPROC_WRAP_GEN_PLL_CTRL1__CH0_MDIV_R 14
#define IPROC_WRAP_GEN_PLL_CTRL1__CH1_MDIV_R 22
#define IPROC_WRAP_GEN_PLL_CTRL2__CH2_MDIV_R 0
#define IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R 8
#define IPROC_WRAP_GEN_PLL_CTRL2__CH4_MDIV_R 16

#define IPROC_WRAP_USBPHY_CTRL_0 0x1800fc44
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ 26
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB 25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB 24
#define IPROC_WRAP_USBPHY_CTRL_2 0x1800fc4c
#define IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO 17
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0 0
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11 11
#define IPROC_WRAP_MISC_STATUS 0x1800fc58
#define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG 2
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK 1

#define IPROC_WRAP_TOP_STRAP_STATUS 0x1800fca4
#define IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL 17
#define IPROC_WRAP_TOP_STRAP_STATUS_1 0x1800fca8
#endif /* End of Hurricane3 only registers */

#define ICFG_CHIP_ID_REG 0x18000000
#define ChipcommonG_MII_Management_Control 0x18002000
#define ChipcommonG_SMBus0_SMBus_Config 0x18008000
#define ChipcommonG_UART0_UART_RBR_THR_DLL 0x18020000
#define ChipcommonG_UART1_UART_RBR_THR_DLL 0x18021000

#define QSPI_mspi_SPCR0_LSB 0x18047200
#define QSPI_mspi_DISABLE_FLUSH_GEN 0x18047384
#define QSPI_bspi_registers_REVISION_ID 0x18047000
#define QSPI_bspi_registers_BSPI_PIO_DATA 0x1804704c
#define QSPI_raf_START_ADDR 0x18047100
#define QSPI_raf_CURR_ADDR 0x18047120
#define QSPI_raf_interrupt_LR_fullness_reached 0x180473a0
#define QSPI_mspi_interrupt_MSPI_halt_set_transaction_done 0x180473b8
#define QSPI_IDM_IDM_IO_CONTROL_DIRECT 0x1811f408
#define CRU_control 0x1800e000

#define NAND_nand_flash_REVISION 0x18046000
#define NAND_direct_read_rd_miss 0x18046f00
#define NAND_IDM_IDM_IO_CONTROL_DIRECT 0x1811d408

#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL 0x18000c8c
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel 1
#define ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel_sw_ovwr 0
#define ICFG_PNOR_STRAPS 0x18000a5c
#define ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R 0
#define PNOR_set_opmode 0x18045018
#define PNOR_set_opmode__set_mw_R 0
#define PNOR_direct_cmd 0x18045010
#define PNOR_direct_cmd__cmd_type_R 21

#define ChipcommonG_tim0_TIM_TIMER1Load 0x18003000
#define ChipcommonG_tim1_TIM_TIMER1Load 0x18004000
#define ChipcommonG_tim2_TIM_TIMER1Load 0x18005000
#define ChipcommonG_tim3_TIM_TIMER1Load 0x18006000
#define ChipcommonG_SMBus0_SMBus_Config 0x18008000
#define ChipcommonG_SMBus1_SMBus_Config 0x1800b000

#define ChipcommonS_RNG_CTRL 0x18032000

#define USBH_Phy_Ctrl_P0 0x18049200
#define USBH_Phy_Ctrl_P1 0x18049204


#define IHOST_M0_IO_CONTROL_DIRECT 0x18100408
#define IHOST_M0_IO_CONTROL_DIRECT_BASE 0x408
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_1_L 31
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_1_R 12
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_1_WIDTH 20
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_1_RESETVALUE 0x00000
#define IHOST_M0_IO_CONTROL_DIRECT__SW_CONTROLLED_EVENTI 11
#define IHOST_M0_IO_CONTROL_DIRECT__SW_CONTROLLED_EVENTI_WIDTH 1
#define IHOST_M0_IO_CONTROL_DIRECT__SW_CONTROLLED_EVENTI_RESETVALUE 0x0
#define IHOST_M0_IO_CONTROL_DIRECT__OVERRIDE_EVENTI 10
#define IHOST_M0_IO_CONTROL_DIRECT__OVERRIDE_EVENTI_WIDTH 1
#define IHOST_M0_IO_CONTROL_DIRECT__OVERRIDE_EVENTI_RESETVALUE 0x0
#define IHOST_M0_IO_CONTROL_DIRECT__CDBGRSTREQ 9
#define IHOST_M0_IO_CONTROL_DIRECT__CDBGRSTREQ_WIDTH 1
#define IHOST_M0_IO_CONTROL_DIRECT__CDBGRSTREQ_RESETVALUE 0x0
#define IHOST_M0_IO_CONTROL_DIRECT__GLBLREQ_L 8
#define IHOST_M0_IO_CONTROL_DIRECT__GLBLREQ_R 5
#define IHOST_M0_IO_CONTROL_DIRECT__GLBLREQ_WIDTH 4
#define IHOST_M0_IO_CONTROL_DIRECT__GLBLREQ_RESETVALUE 0x0
#define IHOST_M0_IO_CONTROL_DIRECT__GLBCMD_L 4
#define IHOST_M0_IO_CONTROL_DIRECT__GLBCMD_R 2
#define IHOST_M0_IO_CONTROL_DIRECT__GLBCMD_WIDTH 3
#define IHOST_M0_IO_CONTROL_DIRECT__GLBCMD_RESETVALUE 0x0
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_0 1
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_0_WIDTH 1
#define IHOST_M0_IO_CONTROL_DIRECT__RESERVED_0_RESETVALUE 0x1
#define IHOST_M0_IO_CONTROL_DIRECT__CLK_EN 0
#define IHOST_M0_IO_CONTROL_DIRECT__CLK_EN_WIDTH 1
#define IHOST_M0_IO_CONTROL_DIRECT__CLK_EN_RESETVALUE 0x1
#define IHOST_M0_IO_CONTROL_DIRECT_WIDTH 32
#define IHOST_M0_IO_CONTROL_DIRECT__WIDTH 32
#define IHOST_M0_IO_CONTROL_DIRECT_ALL_L 31
#define IHOST_M0_IO_CONTROL_DIRECT_ALL_R 0
#define IHOST_M0_IO_CONTROL_DIRECT__ALL_L 31
#define IHOST_M0_IO_CONTROL_DIRECT__ALL_R 0
#define IHOST_M0_IO_CONTROL_DIRECT_DATAMASK 0xffffffff
#define IHOST_M0_IO_CONTROL_DIRECT_RDWRMASK 0x00000000
#define IHOST_M0_IO_CONTROL_DIRECT_RESETVALUE 0x3
#define IHOST_M0_IO_STATUS 0x18100500
#define IHOST_M0_IO_STATUS_BASE 0x500
#define IHOST_M0_IO_STATUS__DEBUG_STATUS_L 31
#define IHOST_M0_IO_STATUS__DEBUG_STATUS_R 11
#define IHOST_M0_IO_STATUS__DEBUG_STATUS_WIDTH 21
#define IHOST_M0_IO_STATUS__DEBUG_STATUS_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__WDRESET 10
#define IHOST_M0_IO_STATUS__WDRESET_WIDTH 1
#define IHOST_M0_IO_STATUS__WDRESET_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__DBGNOPWRDWN 9
#define IHOST_M0_IO_STATUS__DBGNOPWRDWN_WIDTH 1
#define IHOST_M0_IO_STATUS__DBGNOPWRDWN_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__axi_power_on_L 8
#define IHOST_M0_IO_STATUS__axi_power_on_R 7
#define IHOST_M0_IO_STATUS__axi_power_on_WIDTH 2
#define IHOST_M0_IO_STATUS__axi_power_on_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__ARM_SYS_IDLE 6
#define IHOST_M0_IO_STATUS__ARM_SYS_IDLE_WIDTH 1
#define IHOST_M0_IO_STATUS__ARM_SYS_IDLE_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFE 5
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFE_WIDTH 1
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFE_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFI 4
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFI_WIDTH 1
#define IHOST_M0_IO_STATUS__a9mp_STANDBYWFI_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS__GLBACK_L 3
#define IHOST_M0_IO_STATUS__GLBACK_R 0
#define IHOST_M0_IO_STATUS__GLBACK_WIDTH 4
#define IHOST_M0_IO_STATUS__GLBACK_RESETVALUE 0x0
#define IHOST_M0_IO_STATUS_WIDTH 32
#define IHOST_M0_IO_STATUS__WIDTH 32
#define IHOST_M0_IO_STATUS_ALL_L 31
#define IHOST_M0_IO_STATUS_ALL_R 0
#define IHOST_M0_IO_STATUS__ALL_L 31
#define IHOST_M0_IO_STATUS__ALL_R 0
#define IHOST_M0_IO_STATUS_DATAMASK 0xffffffff
#define IHOST_M0_IO_STATUS_RDWRMASK 0x00000000
#define IHOST_M0_IO_STATUS_RESETVALUE 0x0
#define IHOST_M0_RESET_CONTROL 0x18100800
#define IHOST_M0_RESET_CONTROL_BASE 0x800
#define IHOST_M0_RESET_CONTROL__RESERVED_L 31
#define IHOST_M0_RESET_CONTROL__RESERVED_R 1
#define IHOST_M0_RESET_CONTROL__RESERVED_WIDTH 31
#define IHOST_M0_RESET_CONTROL__RESERVED_RESETVALUE 0x0
#define IHOST_M0_RESET_CONTROL__RESET 0
#define IHOST_M0_RESET_CONTROL__RESET_WIDTH 1
#define IHOST_M0_RESET_CONTROL__RESET_RESETVALUE 0x0
#define IHOST_M0_RESET_CONTROL_WIDTH 32
#define IHOST_M0_RESET_CONTROL__WIDTH 32
#define IHOST_M0_RESET_CONTROL_ALL_L 31
#define IHOST_M0_RESET_CONTROL_ALL_R 0
#define IHOST_M0_RESET_CONTROL__ALL_L 31
#define IHOST_M0_RESET_CONTROL__ALL_R 0
#define IHOST_M0_RESET_CONTROL_DATAMASK 0xffffffff
#define IHOST_M0_RESET_CONTROL_RDWRMASK 0x00000000
#define IHOST_M0_RESET_CONTROL_RESETVALUE 0x0
#define IHOST_M0_IDM_RESET_STATUS 0x18100804
#define IHOST_M0_IDM_RESET_STATUS_BASE 0x804
#define IHOST_M0_IDM_RESET_STATUS__RESERVED_L 31
#define IHOST_M0_IDM_RESET_STATUS__RESERVED_R 4
#define IHOST_M0_IDM_RESET_STATUS__RESERVED_WIDTH 28
#define IHOST_M0_IDM_RESET_STATUS__RESERVED_RESETVALUE 0x0000000
#define IHOST_M0_IDM_RESET_STATUS__Write_received 3
#define IHOST_M0_IDM_RESET_STATUS__Write_received_WIDTH 1
#define IHOST_M0_IDM_RESET_STATUS__Write_received_RESETVALUE 0x0
#define IHOST_M0_IDM_RESET_STATUS__Read_received 2
#define IHOST_M0_IDM_RESET_STATUS__Read_received_WIDTH 1
#define IHOST_M0_IDM_RESET_STATUS__Read_received_RESETVALUE 0x0
#define IHOST_M0_IDM_RESET_STATUS__Active_write 1
#define IHOST_M0_IDM_RESET_STATUS__Active_write_WIDTH 1
#define IHOST_M0_IDM_RESET_STATUS__Active_write_RESETVALUE 0x0
#define IHOST_M0_IDM_RESET_STATUS__Active_read 0
#define IHOST_M0_IDM_RESET_STATUS__Active_read_WIDTH 1
#define IHOST_M0_IDM_RESET_STATUS__Active_read_RESETVALUE 0x0
#define IHOST_M0_IDM_RESET_STATUS_WIDTH 32
#define IHOST_M0_IDM_RESET_STATUS__WIDTH 32
#define IHOST_M0_IDM_RESET_STATUS_ALL_L 31
#define IHOST_M0_IDM_RESET_STATUS_ALL_R 0
#define IHOST_M0_IDM_RESET_STATUS__ALL_L 31
#define IHOST_M0_IDM_RESET_STATUS__ALL_R 0
#define IHOST_M0_IDM_RESET_STATUS_DATAMASK 0xffffffff
#define IHOST_M0_IDM_RESET_STATUS_RDWRMASK 0x00000000
#define IHOST_M0_IDM_RESET_STATUS_RESETVALUE 0x0
#define IHOST_M0_IDM_INTERRUPT_STATUS 0x18100a00
#define IHOST_M0_IDM_INTERRUPT_STATUS_BASE 0xa00
#define IHOST_M0_IDM_INTERRUPT_STATUS__RESERVED_L 31
#define IHOST_M0_IDM_INTERRUPT_STATUS__RESERVED_R 2
#define IHOST_M0_IDM_INTERRUPT_STATUS__RESERVED_WIDTH 30
#define IHOST_M0_IDM_INTERRUPT_STATUS__RESERVED_RESETVALUE 0x0
#define IHOST_M0_IDM_INTERRUPT_STATUS__timeout_interrupt 1
#define IHOST_M0_IDM_INTERRUPT_STATUS__timeout_interrupt_WIDTH 1
#define IHOST_M0_IDM_INTERRUPT_STATUS__timeout_interrupt_RESETVALUE 0x0
#define IHOST_M0_IDM_INTERRUPT_STATUS__Error_log_interrupt 0
#define IHOST_M0_IDM_INTERRUPT_STATUS__Error_log_interrupt_WIDTH 1
#define IHOST_M0_IDM_INTERRUPT_STATUS__Error_log_interrupt_RESETVALUE 0x0
#define IHOST_M0_IDM_INTERRUPT_STATUS_WIDTH 32
#define IHOST_M0_IDM_INTERRUPT_STATUS__WIDTH 32
#define IHOST_M0_IDM_INTERRUPT_STATUS_ALL_L 31
#define IHOST_M0_IDM_INTERRUPT_STATUS_ALL_R 0
#define IHOST_M0_IDM_INTERRUPT_STATUS__ALL_L 31
#define IHOST_M0_IDM_INTERRUPT_STATUS__ALL_R 0
#define IHOST_M0_IDM_INTERRUPT_STATUS_DATAMASK 0xffffffff
#define IHOST_M0_IDM_INTERRUPT_STATUS_RDWRMASK 0x00000000
#define IHOST_M0_IDM_INTERRUPT_STATUS_RESETVALUE 0x0


#define IHOST_S0_IDM_ERROR_LOG_CONTROL 0x18107900
#define IHOST_S0_IDM_ERROR_LOG_COMPLETE 0x18107904
#define IHOST_S0_IDM_ERROR_LOG_STATUS 0x18107908
#define IHOST_S0_IDM_ERROR_LOG_ADDR_LSB 0x1810790c
#define IHOST_S0_IDM_ERROR_LOG_ID 0x18107914
#define IHOST_S0_IDM_ERROR_LOG_FLAGS 0x1810791c
#define IHOST_S0_IDM_INTERRUPT_STATUS 0x18107a00

#define IHOST_S1_IDM_ERROR_LOG_CONTROL 0x18106900
#define IHOST_S1_IDM_ERROR_LOG_COMPLETE 0x18106904
#define IHOST_S1_IDM_ERROR_LOG_STATUS 0x18106908
#define IHOST_S1_IDM_ERROR_LOG_ADDR_LSB 0x1810690c
#define IHOST_S1_IDM_ERROR_LOG_ID 0x18106914
#define IHOST_S1_IDM_ERROR_LOG_FLAGS 0x1810691c
#define IHOST_S1_IDM_INTERRUPT_STATUS 0x18106a00

#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL 0x18108900
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_COMPLETE 0x18108904
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS 0x18108908
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1810890c
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ID 0x18108914
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_FLAGS 0x1810891c
#define AXI_PCIE_S0_IDM_IDM_INTERRUPT_STATUS 0x18108a00

#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_CONTROL 0x18109900
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_COMPLETE 0x18109904
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_STATUS 0x18109908
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1810990c
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_ID 0x18109914
#define AXI_PCIE_S1_IDM_IDM_ERROR_LOG_FLAGS 0x1810991c
#define AXI_PCIE_S1_IDM_IDM_INTERRUPT_STATUS 0x18109a00

#define CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL 0x1810a900
#define CMICD_S0_IDM_IDM_ERROR_LOG_COMPLETE 0x1810a904
#define CMICD_S0_IDM_IDM_ERROR_LOG_STATUS 0x1810a908
#define CMICD_S0_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1810a90c
#define CMICD_S0_IDM_IDM_ERROR_LOG_ID 0x1810a914
#define CMICD_S0_IDM_IDM_ERROR_LOG_FLAGS 0x1810a91c
#define CMICD_S0_IDM_IDM_INTERRUPT_STATUS 0x1810aa00

#define USB2_IDM_IDM_IO_CONTROL_DIRECT 0x18115408
#define USB2_IDM_IDM_IO_STATUS 0x18115500
#define USB2_IDM_IDM_RESET_CONTROL 0x18115800
#define USB2_IDM_IDM_RESET_STATUS 0x18115804
#define USB2_IDM_IDM_INTERRUPT_STATUS 0x18115a00

#define USB2D_IDM_IDM_IO_CONTROL_DIRECT 0x18111408
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable 0
#define USB2D_IDM_IDM_RESET_CONTROL 0x18111800
#define USB2D_IDM_IDM_RESET_CONTROL__RESET 0

#define SDIO_IDM0_IDM_IO_CONTROL_DIRECT 0x18116408
#define SDIO_IDM0_IDM_RESET_CONTROL 0x18116800
#define SDIO_IDM0_IDM_RESET_STATUS 0x18116804
#define SDIO_IDM0_IDM_INTERRUPT_STATUS 0x18116a00
#define SDIO_IDM1_IDM_IO_CONTROL_DIRECT 0x18117408
#define SDIO_IDM1_IDM_RESET_CONTROL 0x18117800
#define SDIO_IDM1_IDM_RESET_STATUS 0x18117804
#define SDIO_IDM1_IDM_INTERRUPT_STATUS 0x18117a00

#define A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL 0x18119900
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_COMPLETE 0x18119904
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS 0x18119908
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1811990c
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ID 0x18119914
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_FLAGS 0x1811991c
#define A9JTAG_S0_IDM_IDM_INTERRUPT_STATUS 0x18119a00

/*
#define SRAM_S0_IDM_ERROR_LOG_CONTROL 0x1811b900
#define SRAM_S0_IDM_ERROR_LOG_COMPLETE 0x1811b904
#define SRAM_S0_IDM_ERROR_LOG_STATUS 0x1811b908
#define SRAM_S0_IDM_ERROR_LOG_ADDR_LSB 0x1811b90c
#define SRAM_S0_IDM_ERROR_LOG_ID 0x1811b914
#define SRAM_S0_IDM_ERROR_LOG_FLAGS 0x1811b91c
#define SRAM_S0_IDM_INTERRUPT_STATUS 0x1811ba00
*/

#define APBX_IDM_IDM_ERROR_LOG_CONTROL 0x18130900
#define APBX_IDM_IDM_ERROR_LOG_COMPLETE 0x18130904
#define APBX_IDM_IDM_ERROR_LOG_STATUS 0x18130908
#define APBX_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1813090c
#define APBX_IDM_IDM_ERROR_LOG_ID 0x18130914
#define APBX_IDM_IDM_ERROR_LOG_FLAGS 0x1813091c
#define APBX_IDM_IDM_INTERRUPT_STATUS 0x18130a00

#define APBY_IDM_IDM_ERROR_LOG_CONTROL 0x18131900
#define APBY_IDM_IDM_ERROR_LOG_COMPLETE 0x18131904
#define APBY_IDM_IDM_ERROR_LOG_STATUS 0x18131908
#define APBY_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1813190c
#define APBY_IDM_IDM_ERROR_LOG_ID 0x18131914
#define APBY_IDM_IDM_ERROR_LOG_FLAGS 0x1813191c
#define APBY_IDM_IDM_INTERRUPT_STATUS 0x18131a00

#define APBZ_IDM_IDM_ERROR_LOG_CONTROL 0x18132900
#define APBZ_IDM_IDM_ERROR_LOG_COMPLETE 0x18132904
#define APBZ_IDM_IDM_ERROR_LOG_STATUS 0x18132908
#define APBZ_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1813290c
#define APBZ_IDM_IDM_ERROR_LOG_ID 0x18132914
#define APBZ_IDM_IDM_ERROR_LOG_FLAGS 0x1813291c
#define APBZ_IDM_IDM_INTERRUPT_STATUS 0x18132a00

#define DDR_S1_IDM_ERROR_LOG_CONTROL 0x18104900
#define DDR_S1_IDM_ERROR_LOG_COMPLETE 0x18104904
#define DDR_S1_IDM_ERROR_LOG_STATUS 0x18104908
#define DDR_S1_IDM_ERROR_LOG_ADDR_LSB 0x1810490c
#define DDR_S1_IDM_ERROR_LOG_ID 0x18104914
#define DDR_S1_IDM_ERROR_LOG_FLAGS 0x1810491c
#define DDR_S1_IDM_INTERRUPT_STATUS 0x18104a00

#define DDR_S2_IDM_ERROR_LOG_CONTROL 0x18105900
#define DDR_S2_IDM_ERROR_LOG_COMPLETE 0x18105904
#define DDR_S2_IDM_ERROR_LOG_STATUS 0x18105908
#define DDR_S2_IDM_ERROR_LOG_ADDR_LSB 0x1810590c
#define DDR_S2_IDM_ERROR_LOG_ID 0x18105914
#define DDR_S2_IDM_ERROR_LOG_FLAGS 0x1810591c
#define DDR_S2_IDM_INTERRUPT_STATUS 0x18105a00

#define ROM_S0_IDM_ERROR_LOG_CONTROL 0x1811a900
#define ROM_S0_IDM_ERROR_LOG_COMPLETE 0x1811a904
#define ROM_S0_IDM_ERROR_LOG_STATUS 0x1811a908
#define ROM_S0_IDM_ERROR_LOG_ADDR_LSB 0x1811a90c
#define ROM_S0_IDM_ERROR_LOG_ID 0x1811a914
#define ROM_S0_IDM_ERROR_LOG_FLAGS 0x1811a91c
#define ROM_S0_IDM_INTERRUPT_STATUS 0x1811aa00

#define NAND_IDM_IDM_ERROR_LOG_CONTROL 0x1811d900
#define NAND_IDM_IDM_ERROR_LOG_COMPLETE 0x1811d904
#define NAND_IDM_IDM_ERROR_LOG_STATUS 0x1811d908
#define NAND_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1811d90c
#define NAND_IDM_IDM_ERROR_LOG_ID 0x1811d914
#define NAND_IDM_IDM_ERROR_LOG_FLAGS 0x1811d91c
#define NAND_IDM_IDM_INTERRUPT_STATUS 0x1811da00

#define QSPI_IDM_IDM_ERROR_LOG_CONTROL 0x1811f900
#define QSPI_IDM_IDM_ERROR_LOG_COMPLETE 0x1811f904
#define QSPI_IDM_IDM_ERROR_LOG_STATUS 0x1811f908
#define QSPI_IDM_IDM_ERROR_LOG_ADDR_LSB 0x1811f90c
#define QSPI_IDM_IDM_ERROR_LOG_ID 0x1811f914
#define QSPI_IDM_IDM_ERROR_LOG_FLAGS 0x1811f91c
#define QSPI_IDM_IDM_INTERRUPT_STATUS 0x1811fa00

#define AXIIC_DS_0_IDM_ERROR_LOG_CONTROL 0x18120900
#define AXIIC_DS_0_IDM_ERROR_LOG_COMPLETE 0x18120904
#define AXIIC_DS_0_IDM_ERROR_LOG_STATUS 0x18120908
#define AXIIC_DS_0_IDM_ERROR_LOG_ADDR_LSB 0x1812090c
#define AXIIC_DS_0_IDM_ERROR_LOG_ID 0x18120914
#define AXIIC_DS_0_IDM_ERROR_LOG_FLAGS 0x1812091c
#define AXIIC_DS_0_IDM_INTERRUPT_STATUS 0x18120a00

#define AXIIC_DS_1_IDM_ERROR_LOG_CONTROL 0x18121900
#define AXIIC_DS_1_IDM_ERROR_LOG_COMPLETE 0x18121904
#define AXIIC_DS_1_IDM_ERROR_LOG_STATUS 0x18121908
#define AXIIC_DS_1_IDM_ERROR_LOG_ADDR_LSB 0x1812190c
#define AXIIC_DS_1_IDM_ERROR_LOG_ID 0x18121914
#define AXIIC_DS_1_IDM_ERROR_LOG_FLAGS 0x1812191c
#define AXIIC_DS_1_IDM_INTERRUPT_STATUS 0x18121a00

#define AXIIC_DS_2_IDM_ERROR_LOG_CONTROL 0x18122900
#define AXIIC_DS_2_IDM_ERROR_LOG_COMPLETE 0x18122904
#define AXIIC_DS_2_IDM_ERROR_LOG_STATUS 0x18122908
#define AXIIC_DS_2_IDM_ERROR_LOG_ADDR_LSB 0x1812290c
#define AXIIC_DS_2_IDM_ERROR_LOG_ID 0x18122914
#define AXIIC_DS_2_IDM_ERROR_LOG_FLAGS 0x1812291c
#define AXIIC_DS_2_IDM_INTERRUPT_STATUS 0x18122a00

/*
#define AXIIC_DS_3_IDM_ERROR_LOG_CONTROL 0x1811e900
#define AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE 0x1811e904
#define AXIIC_DS_3_IDM_ERROR_LOG_STATUS 0x1811e908
#define AXIIC_DS_3_IDM_ERROR_LOG_ADDR_LSB 0x1811e90c
#define AXIIC_DS_3_IDM_ERROR_LOG_ID 0x1811e914
#define AXIIC_DS_3_IDM_ERROR_LOG_FLAGS 0x1811e91c
#define AXIIC_DS_3_IDM_INTERRUPT_STATUS 0x1811ea00
*/

/* AMAC */
#define GMAC0_DEVCONTROL                                0x18042000
#define AMAC_IDM0_IO_CONTROL_DIRECT                     0x18110408
#define AMAC_IDM0_IO_CONTROL_DIRECT__CLK_250_SEL        6
#define AMAC_IDM0_IO_CONTROL_DIRECT__DIRECT_GMII_MODE   5
#define AMAC_IDM0_IO_CONTROL_DIRECT__DEST_SYNC_MODE_EN  3



/* GPIO */
#define ChipcommonG_GP_DATA_IN 0x1800a000
#define ChipcommonG_GP_DATA_IN_BASE 0x000
#define ChipcommonG_GP_DATA_OUT_BASE 0x004
#define ChipcommonG_GP_OUT_EN_BASE 0x008
#define ChipcommonG_GP_INT_TYPE_BASE 0x00c
#define ChipcommonG_GP_INT_DE_BASE 0x010
#define ChipcommonG_GP_INT_EDGE_BASE 0x014
#define ChipcommonG_GP_INT_MSK_BASE 0x018
#define ChipcommonG_GP_INT_STAT_BASE 0x01c
#define ChipcommonG_GP_INT_MSTAT_BASE 0x020
#define ChipcommonG_GP_INT_CLR_BASE 0x024
#define ChipcommonG_GP_AUX_SEL_BASE 0x028
#define ChipcommonG_GP_INIT_VAL_BASE 0x030
#define ChipcommonG_GP_PAD_RES_BASE 0x034
#define ChipcommonG_GP_RES_EN_BASE 0x038
#define ChipcommonG_GP_TEST_INPUT_BASE 0x03c
#define ChipcommonG_GP_TEST_OUTPUT_BASE 0x040
#define ChipcommonG_GP_TEST_ENABLE_BASE 0x044
#define ChipcommonG_GP_PRB_ENABLE_BASE 0x048
#define ChipcommonG_GP_PRB_OE_BASE 0x04c

/* Watchdog */
#define ChipcommonG_WDT_WDOGLOAD 0x18009000
#define DMU_PCU_CRU_RESET_REASON 0x1800f014
#define DMU_PCU_CRU_RESET_REASON__watchdog_reset 0

/* USBD */
#define USB2D_ENDPNT_IN_CTRL_0 0x1804c000

#endif /* __SOCREGS_P10_OPEN_H */
