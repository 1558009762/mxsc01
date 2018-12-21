/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/


#ifndef BCHP_SCA_H__
#define BCHP_SCA_H__

/***************************************************************************
 *SCA - Smart Card A
 ***************************************************************************/
#define BCHP_SCA_SC_UART_CMD_1                   0x00400c00 /* SMART CARD UART COMMAND REGISTER */
#define BCHP_SCA_SC_UART_CMD_2                   0x00400c40 /* SMART CARD UART COMMAND REGISTER */
#define BCHP_SCA_SC_PROTO_CMD                    0x00400c0c /* SMART CARD PROTOCOL COMMAND REGISTER */
#define BCHP_SCA_SC_FLOW_CMD                     0x00400c28 /* SMART CARD INTERFACE COMMAND REGISTER */
#define BCHP_SCA_SC_IF_CMD_1                     0x00400c04 /* SMART CARD INTERFACE COMMAND REGISTER */
#define BCHP_SCA_SC_IF_CMD_2                     0x00400c4c /* SMART CARD INTERFACE COMMAND REGISTER */
#define BCHP_SCA_SC_INTR_STAT_1                  0x00400c58 /* SMART CARD INTERRUPT STATUS REGISTER */
#define BCHP_SCA_SC_INTR_STAT_2                  0x00400c5c /* SMART CARD INTERRUPT STATUS REGISTER */
#define BCHP_SCA_SC_INTR_EN_1                    0x00400c50 /* SMART CARD INTERRUPT ENABLE REGISTER */
#define BCHP_SCA_SC_INTR_EN_2                    0x00400c54 /* SMART CARD INTERRUPT ENABLE REGISTER */
#define BCHP_SCA_SC_CLK_CMD                      0x00400c08 /* SMART CARD CLOCK COMMAND */
#define BCHP_SCA_SC_PRESCALE                     0x00400c10 /* SMART CARD CLOCK PRESCALE */
#define BCHP_SCA_SC_TIMER_CMD                    0x00400c48 /* SMART CARD TIMER COMMAND REGISTER */
#define BCHP_SCA_SC_BGT                          0x00400c44 /* SMART CARD BLOCK GUARD TIME REGISTER */
#define BCHP_SCA_SC_TIMER_CNT_1                  0x00400c68 /* SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
#define BCHP_SCA_SC_TIMER_CNT_2                  0x00400c6c /* SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
#define BCHP_SCA_SC_TIMER_CMP_1                  0x00400c60 /* SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
#define BCHP_SCA_SC_TIMER_CMP_2                  0x00400c64 /* SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
#define BCHP_SCA_SC_WAIT_1                       0x00400c70 /* SMART CARD WAITING TIMER REGISTER */
#define BCHP_SCA_SC_WAIT_2                       0x00400c74 /* SMART CARD WAITING TIMER REGISTER */
#define BCHP_SCA_SC_WAIT_3                       0x00400c78 /* SMART CARD WAITING TIMER REGISTER */
#define BCHP_SCA_SC_TGUARD                       0x00400c14 /* SMART CARD TRANSMIT GUARD TIME REGISTER */
#define BCHP_SCA_SC_TRANSMIT                     0x00400c18 /* SMART CARD TRANSMIT REGISTER */
#define BCHP_SCA_SC_RECEIVE                      0x00400c1c /* SMART CARD RECEIVE REGISTER */
#define BCHP_SCA_SC_STATUS_1                     0x00400c34 /* SMART CARD STATUS 1 REGISTER */
#define BCHP_SCA_SC_STATUS_2                     0x00400c38 /* SMART CARD STATUS 2 REGISTER */
#define BCHP_SCA_SC_TLEN_2                       0x00400c20 /* SMART CARD TRANSMIT LENGTH REGISTER */
#define BCHP_SCA_SC_TLEN_1                       0x00400c24 /* SMART CARD TRANSMIT LENGTH REGISTER */
#define BCHP_SCA_SC_RLEN_2                       0x00400c2c /* SMART CARD RECEIVE LENGTH REGISTER */
#define BCHP_SCA_SC_RLEN_1                       0x00400c30 /* SMART CARD RECEIVE LENGTH REGISTER */
#define BCHP_SCA_SC_EVENT1_CNT                   0x00400c80 /* SMART CARD EVENT 1 COUNT REGISTER */
#define BCHP_SCA_SC_EVENT1_CMP                   0x00400c88 /* SMART CARD EVENT 1 COMPARE REGISTER */
#define BCHP_SCA_SC_EVENT1_CMD_1                 0x00400c90 /* SMART CARD EVENT 1 COMMAND 1 REGISTER */
#define BCHP_SCA_SC_EVENT1_CMD_2                 0x00400c94 /* SMART CARD EVENT 1 COMMAND 2 REGISTER */
#define BCHP_SCA_SC_EVENT1_CMD_3                 0x00400c98 /* SMART CARD EVENT 1 COMMAND 3 REGISTER */
#define BCHP_SCA_SC_EVENT1_CMD_4                 0x00400c9c /* SMART CARD EVENT 1 COMMAND 4 REGISTER */
#define BCHP_SCA_SC_EVENT2_CNT                   0x00400ca8 /* SMART CARD EVENT 2 COUNT REGISTER */
#define BCHP_SCA_SC_EVENT2_CMP                   0x00400ca0 /* SMART CARD EVENT 2 COMPARE REGISTER */
#define BCHP_SCA_SC_EVENT2_CMD_1                 0x00400cb0 /* SMART CARD EVENT 2 COMMAND 1 REGISTER */
#define BCHP_SCA_SC_EVENT2_CMD_2                 0x00400cb4 /* SMART CARD EVENT 2 COMMAND 2 REGISTER */
#define BCHP_SCA_SC_EVENT2_CMD_3                 0x00400cb8 /* SMART CARD EVENT 2 COMMAND 3 REGISTER */
#define BCHP_SCA_SC_EVENT2_CMD_4                 0x00400cbc /* SMART CARD EVENT 2 COMMAND 4 REGISTER */

/***************************************************************************
 *SC_UART_CMD_1 - SMART CARD UART COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_UART_CMD_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_UART_CMD_1_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_UART_CMD_1_reserved0_SHIFT                     8

/* SCA :: SC_UART_CMD_1 :: inv_par [07:07] */
#define BCHP_SCA_SC_UART_CMD_1_inv_par_MASK                        0x00000080
#define BCHP_SCA_SC_UART_CMD_1_inv_par_SHIFT                       7

/* SCA :: SC_UART_CMD_1 :: get_atr [06:06] */
#define BCHP_SCA_SC_UART_CMD_1_get_atr_MASK                        0x00000040
#define BCHP_SCA_SC_UART_CMD_1_get_atr_SHIFT                       6

/* SCA :: SC_UART_CMD_1 :: io_en [05:05] */
#define BCHP_SCA_SC_UART_CMD_1_io_en_MASK                          0x00000020
#define BCHP_SCA_SC_UART_CMD_1_io_en_SHIFT                         5

/* SCA :: SC_UART_CMD_1 :: auto_rcv [04:04] */
#define BCHP_SCA_SC_UART_CMD_1_auto_rcv_MASK                       0x00000010
#define BCHP_SCA_SC_UART_CMD_1_auto_rcv_SHIFT                      4

/* SCA :: SC_UART_CMD_1 :: RESERVED [03:03] */
#define BCHP_SCA_SC_UART_CMD_1_RESERVED_MASK                       0x00000008
#define BCHP_SCA_SC_UART_CMD_1_RESERVED_SHIFT                      3

/* SCA :: SC_UART_CMD_1 :: t_r [02:02] */
#define BCHP_SCA_SC_UART_CMD_1_t_r_MASK                            0x00000004
#define BCHP_SCA_SC_UART_CMD_1_t_r_SHIFT                           2

/* SCA :: SC_UART_CMD_1 :: xmit_go [01:01] */
#define BCHP_SCA_SC_UART_CMD_1_xmit_go_MASK                        0x00000002
#define BCHP_SCA_SC_UART_CMD_1_xmit_go_SHIFT                       1

/* SCA :: SC_UART_CMD_1 :: uart_rst [00:00] */
#define BCHP_SCA_SC_UART_CMD_1_uart_rst_MASK                       0x00000001
#define BCHP_SCA_SC_UART_CMD_1_uart_rst_SHIFT                      0

/***************************************************************************
 *SC_UART_CMD_2 - SMART CARD UART COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_UART_CMD_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_UART_CMD_2_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_UART_CMD_2_reserved0_SHIFT                     8

/* SCA :: SC_UART_CMD_2 :: RESERVED [07:07] */
#define BCHP_SCA_SC_UART_CMD_2_RESERVED_MASK                       0x00000080
#define BCHP_SCA_SC_UART_CMD_2_RESERVED_SHIFT                      7

/* SCA :: SC_UART_CMD_2 :: convention [06:06] */
#define BCHP_SCA_SC_UART_CMD_2_convention_MASK                     0x00000040
#define BCHP_SCA_SC_UART_CMD_2_convention_SHIFT                    6

/* SCA :: SC_UART_CMD_2 :: rpar_retry [05:03] */
#define BCHP_SCA_SC_UART_CMD_2_rpar_retry_MASK                     0x00000038
#define BCHP_SCA_SC_UART_CMD_2_rpar_retry_SHIFT                    3

/* SCA :: SC_UART_CMD_2 :: tpar_retry [02:00] */
#define BCHP_SCA_SC_UART_CMD_2_tpar_retry_MASK                     0x00000007
#define BCHP_SCA_SC_UART_CMD_2_tpar_retry_SHIFT                    0

/***************************************************************************
 *SC_PROTO_CMD - SMART CARD PROTOCOL COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_PROTO_CMD :: reserved0 [31:08] */
#define BCHP_SCA_SC_PROTO_CMD_reserved0_MASK                       0xffffff00
#define BCHP_SCA_SC_PROTO_CMD_reserved0_SHIFT                      8

/* SCA :: SC_PROTO_CMD :: crc_lrc [07:07] */
#define BCHP_SCA_SC_PROTO_CMD_crc_lrc_MASK                         0x00000080
#define BCHP_SCA_SC_PROTO_CMD_crc_lrc_SHIFT                        7

/* SCA :: SC_PROTO_CMD :: edc_en [06:06] */
#define BCHP_SCA_SC_PROTO_CMD_edc_en_MASK                          0x00000040
#define BCHP_SCA_SC_PROTO_CMD_edc_en_SHIFT                         6

/* SCA :: SC_PROTO_CMD :: tbuf_rst [05:05] */
#define BCHP_SCA_SC_PROTO_CMD_tbuf_rst_MASK                        0x00000020
#define BCHP_SCA_SC_PROTO_CMD_tbuf_rst_SHIFT                       5

/* SCA :: SC_PROTO_CMD :: rbuf_rst [04:04] */
#define BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK                        0x00000010
#define BCHP_SCA_SC_PROTO_CMD_rbuf_rst_SHIFT                       4

/* SCA :: SC_PROTO_CMD :: cwi [03:00] */
#define BCHP_SCA_SC_PROTO_CMD_cwi_MASK                             0x0000000f
#define BCHP_SCA_SC_PROTO_CMD_cwi_SHIFT                            0

/***************************************************************************
 *SC_FLOW_CMD - SMART CARD INTERFACE COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_FLOW_CMD :: reserved0 [31:02] */
#define BCHP_SCA_SC_FLOW_CMD_reserved0_MASK                        0xfffffffc
#define BCHP_SCA_SC_FLOW_CMD_reserved0_SHIFT                       2

/* SCA :: SC_FLOW_CMD :: rflow [01:01] */
#define BCHP_SCA_SC_FLOW_CMD_rflow_MASK                            0x00000002
#define BCHP_SCA_SC_FLOW_CMD_rflow_SHIFT                           1

/* SCA :: SC_FLOW_CMD :: flow_en [00:00] */
#define BCHP_SCA_SC_FLOW_CMD_flow_en_MASK                          0x00000001
#define BCHP_SCA_SC_FLOW_CMD_flow_en_SHIFT                         0

/***************************************************************************
 *SC_IF_CMD_1 - SMART CARD INTERFACE COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_IF_CMD_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_IF_CMD_1_reserved0_MASK                        0xffffff00
#define BCHP_SCA_SC_IF_CMD_1_reserved0_SHIFT                       8

/* SCA :: SC_IF_CMD_1 :: auto_clk [07:07] */
#define BCHP_SCA_SC_IF_CMD_1_auto_clk_MASK                         0x00000080
#define BCHP_SCA_SC_IF_CMD_1_auto_clk_SHIFT                        7

/* SCA :: SC_IF_CMD_1 :: auto_io [06:06] */
#define BCHP_SCA_SC_IF_CMD_1_auto_io_MASK                          0x00000040
#define BCHP_SCA_SC_IF_CMD_1_auto_io_SHIFT                         6

/* SCA :: SC_IF_CMD_1 :: auto_rst [05:05] */
#define BCHP_SCA_SC_IF_CMD_1_auto_rst_MASK                         0x00000020
#define BCHP_SCA_SC_IF_CMD_1_auto_rst_SHIFT                        5

/* SCA :: SC_IF_CMD_1 :: auto_vcc [04:04] */
#define BCHP_SCA_SC_IF_CMD_1_auto_vcc_MASK                         0x00000010
#define BCHP_SCA_SC_IF_CMD_1_auto_vcc_SHIFT                        4

/* SCA :: SC_IF_CMD_1 :: io [03:03] */
#define BCHP_SCA_SC_IF_CMD_1_io_MASK                               0x00000008
#define BCHP_SCA_SC_IF_CMD_1_io_SHIFT                              3

/* SCA :: SC_IF_CMD_1 :: pres_pol [02:02] */
#define BCHP_SCA_SC_IF_CMD_1_pres_pol_MASK                         0x00000004
#define BCHP_SCA_SC_IF_CMD_1_pres_pol_SHIFT                        2

/* SCA :: SC_IF_CMD_1 :: rst [01:01] */
#define BCHP_SCA_SC_IF_CMD_1_rst_MASK                              0x00000002
#define BCHP_SCA_SC_IF_CMD_1_rst_SHIFT                             1

/* SCA :: SC_IF_CMD_1 :: vcc [00:00] */
#define BCHP_SCA_SC_IF_CMD_1_vcc_MASK                              0x00000001
#define BCHP_SCA_SC_IF_CMD_1_vcc_SHIFT                             0

/***************************************************************************
 *SC_IF_CMD_2 - SMART CARD INTERFACE COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_IF_CMD_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_IF_CMD_2_reserved0_MASK                        0xffffff00
#define BCHP_SCA_SC_IF_CMD_2_reserved0_SHIFT                       8

/* SCA :: SC_IF_CMD_2 :: db_en [07:07] */
#define BCHP_SCA_SC_IF_CMD_2_db_en_MASK                            0x00000080
#define BCHP_SCA_SC_IF_CMD_2_db_en_SHIFT                           7

/* SCA :: SC_IF_CMD_2 :: db_mask [06:06] */
#define BCHP_SCA_SC_IF_CMD_2_db_mask_MASK                          0x00000040
#define BCHP_SCA_SC_IF_CMD_2_db_mask_SHIFT                         6

/* SCA :: SC_IF_CMD_2 :: db_width [05:00] */
#define BCHP_SCA_SC_IF_CMD_2_db_width_MASK                         0x0000003f
#define BCHP_SCA_SC_IF_CMD_2_db_width_SHIFT                        0

/***************************************************************************
 *SC_INTR_STAT_1 - SMART CARD INTERRUPT STATUS REGISTER
 ***************************************************************************/
/* SCA :: SC_INTR_STAT_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_INTR_STAT_1_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_INTR_STAT_1_reserved0_SHIFT                    8

/* SCA :: SC_INTR_STAT_1 :: tpar_intr [07:07] */
#define BCHP_SCA_SC_INTR_STAT_1_tpar_intr_MASK                     0x00000080
#define BCHP_SCA_SC_INTR_STAT_1_tpar_intr_SHIFT                    7

/* SCA :: SC_INTR_STAT_1 :: timer_intr [06:06] */
#define BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK                    0x00000040
#define BCHP_SCA_SC_INTR_STAT_1_timer_intr_SHIFT                   6

/* SCA :: SC_INTR_STAT_1 :: pres_intr [05:05] */
#define BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK                     0x00000020
#define BCHP_SCA_SC_INTR_STAT_1_pres_intr_SHIFT                    5

/* SCA :: SC_INTR_STAT_1 :: bgt_intr [04:04] */
#define BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK                      0x00000010
#define BCHP_SCA_SC_INTR_STAT_1_bgt_intr_SHIFT                     4

/* SCA :: SC_INTR_STAT_1 :: tdone_intr [03:03] */
#define BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK                    0x00000008
#define BCHP_SCA_SC_INTR_STAT_1_tdone_intr_SHIFT                   3

/* SCA :: SC_INTR_STAT_1 :: retry_intr [02:02] */
#define BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK                    0x00000004
#define BCHP_SCA_SC_INTR_STAT_1_retry_intr_SHIFT                   2

/* SCA :: SC_INTR_STAT_1 :: tempty_intr [01:01] */
#define BCHP_SCA_SC_INTR_STAT_1_tempty_intr_MASK                   0x00000002
#define BCHP_SCA_SC_INTR_STAT_1_tempty_intr_SHIFT                  1

/* SCA :: SC_INTR_STAT_1 :: event1_intr [00:00] */
#define BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK                   0x00000001
#define BCHP_SCA_SC_INTR_STAT_1_event1_intr_SHIFT                  0

/***************************************************************************
 *SC_INTR_STAT_2 - SMART CARD INTERRUPT STATUS REGISTER
 ***************************************************************************/
/* SCA :: SC_INTR_STAT_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_INTR_STAT_2_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_INTR_STAT_2_reserved0_SHIFT                    8

/* SCA :: SC_INTR_STAT_2 :: rpar_intr [07:07] */
#define BCHP_SCA_SC_INTR_STAT_2_rpar_intr_MASK                     0x00000080
#define BCHP_SCA_SC_INTR_STAT_2_rpar_intr_SHIFT                    7

/* SCA :: SC_INTR_STAT_2 :: atrs_intr [06:06] */
#define BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK                     0x00000040
#define BCHP_SCA_SC_INTR_STAT_2_atrs_intr_SHIFT                    6

/* SCA :: SC_INTR_STAT_2 :: cwt_intr [05:05] */
#define BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK                      0x00000020
#define BCHP_SCA_SC_INTR_STAT_2_cwt_intr_SHIFT                     5

/* SCA :: SC_INTR_STAT_2 :: rlen_intr [04:04] */
#define BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK                     0x00000010
#define BCHP_SCA_SC_INTR_STAT_2_rlen_intr_SHIFT                    4

/* SCA :: SC_INTR_STAT_2 :: wait_intr [03:03] */
#define BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK                     0x00000008
#define BCHP_SCA_SC_INTR_STAT_2_wait_intr_SHIFT                    3

/* SCA :: SC_INTR_STAT_2 :: event2_intr [02:02] */
#define BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK                   0x00000004
#define BCHP_SCA_SC_INTR_STAT_2_event2_intr_SHIFT                  2

/* SCA :: SC_INTR_STAT_2 :: rcv_intr [01:01] */
#define BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK                      0x00000002
#define BCHP_SCA_SC_INTR_STAT_2_rcv_intr_SHIFT                     1

/* SCA :: SC_INTR_STAT_2 :: rready_intr [00:00] */
#define BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK                   0x00000001
#define BCHP_SCA_SC_INTR_STAT_2_rready_intr_SHIFT                  0

/***************************************************************************
 *SC_INTR_EN_1 - SMART CARD INTERRUPT ENABLE REGISTER
 ***************************************************************************/
/* SCA :: SC_INTR_EN_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_INTR_EN_1_reserved0_MASK                       0xffffff00
#define BCHP_SCA_SC_INTR_EN_1_reserved0_SHIFT                      8

/* SCA :: SC_INTR_EN_1 :: tpar_ien [07:07] */
#define BCHP_SCA_SC_INTR_EN_1_tpar_ien_MASK                        0x00000080
#define BCHP_SCA_SC_INTR_EN_1_tpar_ien_SHIFT                       7

/* SCA :: SC_INTR_EN_1 :: timer_ien [06:06] */
#define BCHP_SCA_SC_INTR_EN_1_timer_ien_MASK                       0x00000040
#define BCHP_SCA_SC_INTR_EN_1_timer_ien_SHIFT                      6

/* SCA :: SC_INTR_EN_1 :: pres_ien [05:05] */
#define BCHP_SCA_SC_INTR_EN_1_pres_ien_MASK                        0x00000020
#define BCHP_SCA_SC_INTR_EN_1_pres_ien_SHIFT                       5

/* SCA :: SC_INTR_EN_1 :: bgt_ien7 [04:04] */
#define BCHP_SCA_SC_INTR_EN_1_bgt_ien7_MASK                        0x00000010
#define BCHP_SCA_SC_INTR_EN_1_bgt_ien7_SHIFT                       4

/* SCA :: SC_INTR_EN_1 :: tdone_ien [03:03] */
#define BCHP_SCA_SC_INTR_EN_1_tdone_ien_MASK                       0x00000008
#define BCHP_SCA_SC_INTR_EN_1_tdone_ien_SHIFT                      3

/* SCA :: SC_INTR_EN_1 :: retry_ien [02:02] */
#define BCHP_SCA_SC_INTR_EN_1_retry_ien_MASK                       0x00000004
#define BCHP_SCA_SC_INTR_EN_1_retry_ien_SHIFT                      2

/* SCA :: SC_INTR_EN_1 :: tempty_ien [01:01] */
#define BCHP_SCA_SC_INTR_EN_1_tempty_ien_MASK                      0x00000002
#define BCHP_SCA_SC_INTR_EN_1_tempty_ien_SHIFT                     1

/* SCA :: SC_INTR_EN_1 :: event1_ien [00:00] */
#define BCHP_SCA_SC_INTR_EN_1_event1_ien_MASK                      0x00000001
#define BCHP_SCA_SC_INTR_EN_1_event1_ien_SHIFT                     0

/***************************************************************************
 *SC_INTR_EN_2 - SMART CARD INTERRUPT ENABLE REGISTER
 ***************************************************************************/
/* SCA :: SC_INTR_EN_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_INTR_EN_2_reserved0_MASK                       0xffffff00
#define BCHP_SCA_SC_INTR_EN_2_reserved0_SHIFT                      8

/* SCA :: SC_INTR_EN_2 :: rpar_ien [07:07] */
#define BCHP_SCA_SC_INTR_EN_2_rpar_ien_MASK                        0x00000080
#define BCHP_SCA_SC_INTR_EN_2_rpar_ien_SHIFT                       7

/* SCA :: SC_INTR_EN_2 :: atrs_ien [06:06] */
#define BCHP_SCA_SC_INTR_EN_2_atrs_ien_MASK                        0x00000040
#define BCHP_SCA_SC_INTR_EN_2_atrs_ien_SHIFT                       6

/* SCA :: SC_INTR_EN_2 :: cwt_ien [05:05] */
#define BCHP_SCA_SC_INTR_EN_2_cwt_ien_MASK                         0x00000020
#define BCHP_SCA_SC_INTR_EN_2_cwt_ien_SHIFT                        5

/* SCA :: SC_INTR_EN_2 :: rlen_ien [04:04] */
#define BCHP_SCA_SC_INTR_EN_2_rlen_ien_MASK                        0x00000010
#define BCHP_SCA_SC_INTR_EN_2_rlen_ien_SHIFT                       4

/* SCA :: SC_INTR_EN_2 :: wait_ien [03:03] */
#define BCHP_SCA_SC_INTR_EN_2_wait_ien_MASK                        0x00000008
#define BCHP_SCA_SC_INTR_EN_2_wait_ien_SHIFT                       3

/* SCA :: SC_INTR_EN_2 :: event2_ien [02:02] */
#define BCHP_SCA_SC_INTR_EN_2_event2_ien_MASK                      0x00000004
#define BCHP_SCA_SC_INTR_EN_2_event2_ien_SHIFT                     2

/* SCA :: SC_INTR_EN_2 :: rcv_ien [01:01] */
#define BCHP_SCA_SC_INTR_EN_2_rcv_ien_MASK                         0x00000002
#define BCHP_SCA_SC_INTR_EN_2_rcv_ien_SHIFT                        1

/* SCA :: SC_INTR_EN_2 :: rready_ien [00:00] */
#define BCHP_SCA_SC_INTR_EN_2_rready_ien_MASK                      0x00000001
#define BCHP_SCA_SC_INTR_EN_2_rready_ien_SHIFT                     0

/***************************************************************************
 *SC_CLK_CMD - SMART CARD CLOCK COMMAND
 ***************************************************************************/
/* SCA :: SC_CLK_CMD :: reserved0 [31:08] */
#define BCHP_SCA_SC_CLK_CMD_reserved0_MASK                         0xffffff00
#define BCHP_SCA_SC_CLK_CMD_reserved0_SHIFT                        8

/* SCA :: SC_CLK_CMD :: clk_en [07:07] */
#define BCHP_SCA_SC_CLK_CMD_clk_en_MASK                            0x00000080
#define BCHP_SCA_SC_CLK_CMD_clk_en_SHIFT                           7

/* SCA :: SC_CLK_CMD :: sc_clkdiv [06:04] */
#define BCHP_SCA_SC_CLK_CMD_sc_clkdiv_MASK                         0x00000070
#define BCHP_SCA_SC_CLK_CMD_sc_clkdiv_SHIFT                        4

/* SCA :: SC_CLK_CMD :: etu_clkdiv [03:01] */
#define BCHP_SCA_SC_CLK_CMD_etu_clkdiv_MASK                        0x0000000e
#define BCHP_SCA_SC_CLK_CMD_etu_clkdiv_SHIFT                       1

/* SCA :: SC_CLK_CMD :: bauddiv [00:00] */
#define BCHP_SCA_SC_CLK_CMD_bauddiv_MASK                           0x00000001
#define BCHP_SCA_SC_CLK_CMD_bauddiv_SHIFT                          0

/***************************************************************************
 *SC_PRESCALE - SMART CARD CLOCK PRESCALE
 ***************************************************************************/
/* SCA :: SC_PRESCALE :: reserved0 [31:08] */
#define BCHP_SCA_SC_PRESCALE_reserved0_MASK                        0xffffff00
#define BCHP_SCA_SC_PRESCALE_reserved0_SHIFT                       8

/* SCA :: SC_PRESCALE :: sc_prescale [07:00] */
#define BCHP_SCA_SC_PRESCALE_sc_prescale_MASK                      0x000000ff
#define BCHP_SCA_SC_PRESCALE_sc_prescale_SHIFT                     0

/***************************************************************************
 *SC_TIMER_CMD - SMART CARD TIMER COMMAND REGISTER
 ***************************************************************************/
/* SCA :: SC_TIMER_CMD :: reserved0 [31:08] */
#define BCHP_SCA_SC_TIMER_CMD_reserved0_MASK                       0xffffff00
#define BCHP_SCA_SC_TIMER_CMD_reserved0_SHIFT                      8

/* SCA :: SC_TIMER_CMD :: timer_src [07:07] */
#define BCHP_SCA_SC_TIMER_CMD_timer_src_MASK                       0x00000080
#define BCHP_SCA_SC_TIMER_CMD_timer_src_SHIFT                      7

/* SCA :: SC_TIMER_CMD :: timer_mode [06:06] */
#define BCHP_SCA_SC_TIMER_CMD_timer_mode_MASK                      0x00000040
#define BCHP_SCA_SC_TIMER_CMD_timer_mode_SHIFT                     6

/* SCA :: SC_TIMER_CMD :: timer_en [05:05] */
#define BCHP_SCA_SC_TIMER_CMD_timer_en_MASK                        0x00000020
#define BCHP_SCA_SC_TIMER_CMD_timer_en_SHIFT                       5

/* SCA :: SC_TIMER_CMD :: cwt_en [04:04] */
#define BCHP_SCA_SC_TIMER_CMD_cwt_en_MASK                          0x00000010
#define BCHP_SCA_SC_TIMER_CMD_cwt_en_SHIFT                         4

/* SCA :: SC_TIMER_CMD :: RESERVED [03:02] */
#define BCHP_SCA_SC_TIMER_CMD_RESERVED_MASK                        0x0000000c
#define BCHP_SCA_SC_TIMER_CMD_RESERVED_SHIFT                       2

/* SCA :: SC_TIMER_CMD :: wait_mode [01:01] */
#define BCHP_SCA_SC_TIMER_CMD_wait_mode_MASK                       0x00000002
#define BCHP_SCA_SC_TIMER_CMD_wait_mode_SHIFT                      1

/* SCA :: SC_TIMER_CMD :: wait_en [00:00] */
#define BCHP_SCA_SC_TIMER_CMD_wait_en_MASK                         0x00000001
#define BCHP_SCA_SC_TIMER_CMD_wait_en_SHIFT                        0

/***************************************************************************
 *SC_BGT - SMART CARD BLOCK GUARD TIME REGISTER
 ***************************************************************************/
/* SCA :: SC_BGT :: reserved0 [31:08] */
#define BCHP_SCA_SC_BGT_reserved0_MASK                             0xffffff00
#define BCHP_SCA_SC_BGT_reserved0_SHIFT                            8

/* SCA :: SC_BGT :: r2t [07:07] */
#define BCHP_SCA_SC_BGT_r2t_MASK                                   0x00000080
#define BCHP_SCA_SC_BGT_r2t_SHIFT                                  7

/* SCA :: SC_BGT :: t2r [06:06] */
#define BCHP_SCA_SC_BGT_t2r_MASK                                   0x00000040
#define BCHP_SCA_SC_BGT_t2r_SHIFT                                  6

/* SCA :: SC_BGT :: bgt [05:00] */
#define BCHP_SCA_SC_BGT_bgt_MASK                                   0x0000003f
#define BCHP_SCA_SC_BGT_bgt_SHIFT                                  0

/***************************************************************************
 *SC_TIMER_CNT_1 - SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER
 ***************************************************************************/
/* SCA :: SC_TIMER_CNT_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_TIMER_CNT_1_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_TIMER_CNT_1_reserved0_SHIFT                    8

/* SCA :: SC_TIMER_CNT_1 :: sc_timer_cnt [07:00] */
#define BCHP_SCA_SC_TIMER_CNT_1_sc_timer_cnt_MASK                  0x000000ff
#define BCHP_SCA_SC_TIMER_CNT_1_sc_timer_cnt_SHIFT                 0

/***************************************************************************
 *SC_TIMER_CNT_2 - SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER
 ***************************************************************************/
/* SCA :: SC_TIMER_CNT_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_TIMER_CNT_2_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_TIMER_CNT_2_reserved0_SHIFT                    8

/* SCA :: SC_TIMER_CNT_2 :: sc_timer_cnt [07:00] */
#define BCHP_SCA_SC_TIMER_CNT_2_sc_timer_cnt_MASK                  0x000000ff
#define BCHP_SCA_SC_TIMER_CNT_2_sc_timer_cnt_SHIFT                 0

/***************************************************************************
 *SC_TIMER_CMP_1 - SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER
 ***************************************************************************/
/* SCA :: SC_TIMER_CMP_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_TIMER_CMP_1_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_TIMER_CMP_1_reserved0_SHIFT                    8

/* SCA :: SC_TIMER_CMP_1 :: sc_timer_cmp [07:00] */
#define BCHP_SCA_SC_TIMER_CMP_1_sc_timer_cmp_MASK                  0x000000ff
#define BCHP_SCA_SC_TIMER_CMP_1_sc_timer_cmp_SHIFT                 0

/***************************************************************************
 *SC_TIMER_CMP_2 - SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER
 ***************************************************************************/
/* SCA :: SC_TIMER_CMP_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_TIMER_CMP_2_reserved0_MASK                     0xffffff00
#define BCHP_SCA_SC_TIMER_CMP_2_reserved0_SHIFT                    8

/* SCA :: SC_TIMER_CMP_2 :: sc_timer_cmp [07:00] */
#define BCHP_SCA_SC_TIMER_CMP_2_sc_timer_cmp_MASK                  0x000000ff
#define BCHP_SCA_SC_TIMER_CMP_2_sc_timer_cmp_SHIFT                 0

/***************************************************************************
 *SC_WAIT_1 - SMART CARD WAITING TIMER REGISTER
 ***************************************************************************/
/* SCA :: SC_WAIT_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_WAIT_1_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_WAIT_1_reserved0_SHIFT                         8

/* SCA :: SC_WAIT_1 :: sc_wait [07:00] */
#define BCHP_SCA_SC_WAIT_1_sc_wait_MASK                            0x000000ff
#define BCHP_SCA_SC_WAIT_1_sc_wait_SHIFT                           0

/***************************************************************************
 *SC_WAIT_2 - SMART CARD WAITING TIMER REGISTER
 ***************************************************************************/
/* SCA :: SC_WAIT_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_WAIT_2_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_WAIT_2_reserved0_SHIFT                         8

/* SCA :: SC_WAIT_2 :: sc_wait [07:00] */
#define BCHP_SCA_SC_WAIT_2_sc_wait_MASK                            0x000000ff
#define BCHP_SCA_SC_WAIT_2_sc_wait_SHIFT                           0

/***************************************************************************
 *SC_WAIT_3 - SMART CARD WAITING TIMER REGISTER
 ***************************************************************************/
/* SCA :: SC_WAIT_3 :: reserved0 [31:08] */
#define BCHP_SCA_SC_WAIT_3_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_WAIT_3_reserved0_SHIFT                         8

/* SCA :: SC_WAIT_3 :: sc_wait [07:00] */
#define BCHP_SCA_SC_WAIT_3_sc_wait_MASK                            0x000000ff
#define BCHP_SCA_SC_WAIT_3_sc_wait_SHIFT                           0

/***************************************************************************
 *SC_TGUARD - SMART CARD TRANSMIT GUARD TIME REGISTER
 ***************************************************************************/
/* SCA :: SC_TGUARD :: reserved0 [31:08] */
#define BCHP_SCA_SC_TGUARD_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_TGUARD_reserved0_SHIFT                         8

/* SCA :: SC_TGUARD :: sc_tguard [07:00] */
#define BCHP_SCA_SC_TGUARD_sc_tguard_MASK                          0x000000ff
#define BCHP_SCA_SC_TGUARD_sc_tguard_SHIFT                         0

/***************************************************************************
 *SC_TRANSMIT - SMART CARD TRANSMIT REGISTER
 ***************************************************************************/
/* SCA :: SC_TRANSMIT :: reserved0 [31:08] */
#define BCHP_SCA_SC_TRANSMIT_reserved0_MASK                        0xffffff00
#define BCHP_SCA_SC_TRANSMIT_reserved0_SHIFT                       8

/* SCA :: SC_TRANSMIT :: sc_transmit [07:00] */
#define BCHP_SCA_SC_TRANSMIT_sc_transmit_MASK                      0x000000ff
#define BCHP_SCA_SC_TRANSMIT_sc_transmit_SHIFT                     0

/***************************************************************************
 *SC_RECEIVE - SMART CARD RECEIVE REGISTER
 ***************************************************************************/
/* SCA :: SC_RECEIVE :: reserved0 [31:08] */
#define BCHP_SCA_SC_RECEIVE_reserved0_MASK                         0xffffff00
#define BCHP_SCA_SC_RECEIVE_reserved0_SHIFT                        8

/* SCA :: SC_RECEIVE :: sc_receive [07:00] */
#define BCHP_SCA_SC_RECEIVE_sc_receive_MASK                        0x000000ff
#define BCHP_SCA_SC_RECEIVE_sc_receive_SHIFT                       0

/***************************************************************************
 *SC_STATUS_1 - SMART CARD STATUS 1 REGISTER
 ***************************************************************************/
/* SCA :: SC_STATUS_1 :: reserved0 [31:07] */
#define BCHP_SCA_SC_STATUS_1_reserved0_MASK                        0xffffff80
#define BCHP_SCA_SC_STATUS_1_reserved0_SHIFT                       7

/* SCA :: SC_STATUS_1 :: card_pres [06:06] */
#define BCHP_SCA_SC_STATUS_1_card_pres_MASK                        0x00000040
#define BCHP_SCA_SC_STATUS_1_card_pres_SHIFT                       6

/* SCA :: SC_STATUS_1 :: reserved1 [05:03] */
#define BCHP_SCA_SC_STATUS_1_reserved1_MASK                        0x00000038
#define BCHP_SCA_SC_STATUS_1_reserved1_SHIFT                       3

/* SCA :: SC_STATUS_1 :: sc_io [02:02] */
#define BCHP_SCA_SC_STATUS_1_sc_io_MASK                            0x00000004
#define BCHP_SCA_SC_STATUS_1_sc_io_SHIFT                           2

/* SCA :: SC_STATUS_1 :: tempty [01:01] */
#define BCHP_SCA_SC_STATUS_1_tempty_MASK                           0x00000002
#define BCHP_SCA_SC_STATUS_1_tempty_SHIFT                          1

/* SCA :: SC_STATUS_1 :: tdone [00:00] */
#define BCHP_SCA_SC_STATUS_1_tdone_MASK                            0x00000001
#define BCHP_SCA_SC_STATUS_1_tdone_SHIFT                           0

/***************************************************************************
 *SC_STATUS_2 - SMART CARD STATUS 2 REGISTER
 ***************************************************************************/
/* SCA :: SC_STATUS_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_STATUS_2_reserved0_MASK                        0xffffff00
#define BCHP_SCA_SC_STATUS_2_reserved0_SHIFT                       8

/* SCA :: SC_STATUS_2 :: rpar_err [07:07] */
#define BCHP_SCA_SC_STATUS_2_rpar_err_MASK                         0x00000080
#define BCHP_SCA_SC_STATUS_2_rpar_err_SHIFT                        7

/* SCA :: SC_STATUS_2 :: reserved1 [06:04] */
#define BCHP_SCA_SC_STATUS_2_reserved1_MASK                        0x00000070
#define BCHP_SCA_SC_STATUS_2_reserved1_SHIFT                       4

/* SCA :: SC_STATUS_2 :: roverflow [03:03] */
#define BCHP_SCA_SC_STATUS_2_roverflow_MASK                        0x00000008
#define BCHP_SCA_SC_STATUS_2_roverflow_SHIFT                       3

/* SCA :: SC_STATUS_2 :: edc_err [02:02] */
#define BCHP_SCA_SC_STATUS_2_edc_err_MASK                          0x00000004
#define BCHP_SCA_SC_STATUS_2_edc_err_SHIFT                         2

/* SCA :: SC_STATUS_2 :: rempty [01:01] */
#define BCHP_SCA_SC_STATUS_2_rempty_MASK                           0x00000002
#define BCHP_SCA_SC_STATUS_2_rempty_SHIFT                          1

/* SCA :: SC_STATUS_2 :: rready [00:00] */
#define BCHP_SCA_SC_STATUS_2_rready_MASK                           0x00000001
#define BCHP_SCA_SC_STATUS_2_rready_SHIFT                          0

/***************************************************************************
 *SC_TLEN_2 - SMART CARD TRANSMIT LENGTH REGISTER
 ***************************************************************************/
/* SCA :: SC_TLEN_2 :: reserved0 [31:01] */
#define BCHP_SCA_SC_TLEN_2_reserved0_MASK                          0xfffffffe
#define BCHP_SCA_SC_TLEN_2_reserved0_SHIFT                         1

/* SCA :: SC_TLEN_2 :: sc_tlen [00:00] */
#define BCHP_SCA_SC_TLEN_2_sc_tlen_MASK                            0x00000001
#define BCHP_SCA_SC_TLEN_2_sc_tlen_SHIFT                           0

/***************************************************************************
 *SC_TLEN_1 - SMART CARD TRANSMIT LENGTH REGISTER
 ***************************************************************************/
/* SCA :: SC_TLEN_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_TLEN_1_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_TLEN_1_reserved0_SHIFT                         8

/* SCA :: SC_TLEN_1 :: sc_tlen [07:00] */
#define BCHP_SCA_SC_TLEN_1_sc_tlen_MASK                            0x000000ff
#define BCHP_SCA_SC_TLEN_1_sc_tlen_SHIFT                           0

/***************************************************************************
 *SC_RLEN_2 - SMART CARD RECEIVE LENGTH REGISTER
 ***************************************************************************/
/* SCA :: SC_RLEN_2 :: reserved0 [31:01] */
#define BCHP_SCA_SC_RLEN_2_reserved0_MASK                          0xfffffffe
#define BCHP_SCA_SC_RLEN_2_reserved0_SHIFT                         1

/* SCA :: SC_RLEN_2 :: sc_rlen [00:00] */
#define BCHP_SCA_SC_RLEN_2_sc_rlen_MASK                            0x00000001
#define BCHP_SCA_SC_RLEN_2_sc_rlen_SHIFT                           0

/***************************************************************************
 *SC_RLEN_1 - SMART CARD RECEIVE LENGTH REGISTER
 ***************************************************************************/
/* SCA :: SC_RLEN_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_RLEN_1_reserved0_MASK                          0xffffff00
#define BCHP_SCA_SC_RLEN_1_reserved0_SHIFT                         8

/* SCA :: SC_RLEN_1 :: sc_rlen [07:00] */
#define BCHP_SCA_SC_RLEN_1_sc_rlen_MASK                            0x000000ff
#define BCHP_SCA_SC_RLEN_1_sc_rlen_SHIFT                           0

/***************************************************************************
 *SC_EVENT1_CNT - SMART CARD EVENT 1 COUNT REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CNT :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CNT_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_EVENT1_CNT_reserved0_SHIFT                     8

/* SCA :: SC_EVENT1_CNT :: sc_event1_cnt [07:00] */
#define BCHP_SCA_SC_EVENT1_CNT_sc_event1_cnt_MASK                  0x000000ff
#define BCHP_SCA_SC_EVENT1_CNT_sc_event1_cnt_SHIFT                 0

/***************************************************************************
 *SC_EVENT1_CMP - SMART CARD EVENT 1 COMPARE REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CMP :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CMP_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_EVENT1_CMP_reserved0_SHIFT                     8

/* SCA :: SC_EVENT1_CMP :: sc_event1_cmp [07:00] */
#define BCHP_SCA_SC_EVENT1_CMP_sc_event1_cmp_MASK                  0x000000ff
#define BCHP_SCA_SC_EVENT1_CMP_sc_event1_cmp_SHIFT                 0

/***************************************************************************
 *SC_EVENT1_CMD_1 - SMART CARD EVENT 1 COMMAND 1 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CMD_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CMD_1_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT1_CMD_1_reserved0_SHIFT                   8

/* SCA :: SC_EVENT1_CMD_1 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT1_CMD_1_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT1_CMD_1_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_1 :: increment_event_src [04:00] */
#define BCHP_SCA_SC_EVENT1_CMD_1_increment_event_src_MASK          0x0000001f
#define BCHP_SCA_SC_EVENT1_CMD_1_increment_event_src_SHIFT         0

/***************************************************************************
 *SC_EVENT1_CMD_2 - SMART CARD EVENT 1 COMMAND 2 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CMD_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CMD_2_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT1_CMD_2_reserved0_SHIFT                   8

/* SCA :: SC_EVENT1_CMD_2 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT1_CMD_2_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT1_CMD_2_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_2 :: increment_event_src [04:00] */
#define BCHP_SCA_SC_EVENT1_CMD_2_increment_event_src_MASK          0x0000001f
#define BCHP_SCA_SC_EVENT1_CMD_2_increment_event_src_SHIFT         0

/***************************************************************************
 *SC_EVENT1_CMD_3 - SMART CARD EVENT 1 COMMAND 3 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CMD_3 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CMD_3_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT1_CMD_3_reserved0_SHIFT                   8

/* SCA :: SC_EVENT1_CMD_3 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT1_CMD_3_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT1_CMD_3_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_3 :: start_event_src [04:00] */
#define BCHP_SCA_SC_EVENT1_CMD_3_start_event_src_MASK              0x0000001f
#define BCHP_SCA_SC_EVENT1_CMD_3_start_event_src_SHIFT             0

/***************************************************************************
 *SC_EVENT1_CMD_4 - SMART CARD EVENT 1 COMMAND 4 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT1_CMD_4 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT1_CMD_4_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT1_CMD_4_reserved0_SHIFT                   8

/* SCA :: SC_EVENT1_CMD_4 :: event_en [07:07] */
#define BCHP_SCA_SC_EVENT1_CMD_4_event_en_MASK                     0x00000080
#define BCHP_SCA_SC_EVENT1_CMD_4_event_en_SHIFT                    7

/* SCA :: SC_EVENT1_CMD_4 :: RESERVED [06:04] */
#define BCHP_SCA_SC_EVENT1_CMD_4_RESERVED_MASK                     0x00000070
#define BCHP_SCA_SC_EVENT1_CMD_4_RESERVED_SHIFT                    4

/* SCA :: SC_EVENT1_CMD_4 :: intr_after_reset [03:03] */
#define BCHP_SCA_SC_EVENT1_CMD_4_intr_after_reset_MASK             0x00000008
#define BCHP_SCA_SC_EVENT1_CMD_4_intr_after_reset_SHIFT            3

/* SCA :: SC_EVENT1_CMD_4 :: intr_after_compare [02:02] */
#define BCHP_SCA_SC_EVENT1_CMD_4_intr_after_compare_MASK           0x00000004
#define BCHP_SCA_SC_EVENT1_CMD_4_intr_after_compare_SHIFT          2

/* SCA :: SC_EVENT1_CMD_4 :: run_after_reset [01:01] */
#define BCHP_SCA_SC_EVENT1_CMD_4_run_after_reset_MASK              0x00000002
#define BCHP_SCA_SC_EVENT1_CMD_4_run_after_reset_SHIFT             1

/* SCA :: SC_EVENT1_CMD_4 :: run_after_compare [00:00] */
#define BCHP_SCA_SC_EVENT1_CMD_4_run_after_compare_MASK            0x00000001
#define BCHP_SCA_SC_EVENT1_CMD_4_run_after_compare_SHIFT           0

/***************************************************************************
 *SC_EVENT2_CNT - SMART CARD EVENT 2 COUNT REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CNT :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CNT_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_EVENT2_CNT_reserved0_SHIFT                     8

/* SCA :: SC_EVENT2_CNT :: sc_event2_cnt [07:00] */
#define BCHP_SCA_SC_EVENT2_CNT_sc_event2_cnt_MASK                  0x000000ff
#define BCHP_SCA_SC_EVENT2_CNT_sc_event2_cnt_SHIFT                 0

/***************************************************************************
 *SC_EVENT2_CMP - SMART CARD EVENT 2 COMPARE REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CMP :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CMP_reserved0_MASK                      0xffffff00
#define BCHP_SCA_SC_EVENT2_CMP_reserved0_SHIFT                     8

/* SCA :: SC_EVENT2_CMP :: sc_event2_cmp [07:00] */
#define BCHP_SCA_SC_EVENT2_CMP_sc_event2_cmp_MASK                  0x000000ff
#define BCHP_SCA_SC_EVENT2_CMP_sc_event2_cmp_SHIFT                 0

/***************************************************************************
 *SC_EVENT2_CMD_1 - SMART CARD EVENT 2 COMMAND 1 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CMD_1 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CMD_1_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT2_CMD_1_reserved0_SHIFT                   8

/* SCA :: SC_EVENT2_CMD_1 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT2_CMD_1_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT2_CMD_1_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT2_CMD_1 :: increment_event_src [04:00] */
#define BCHP_SCA_SC_EVENT2_CMD_1_increment_event_src_MASK          0x0000001f
#define BCHP_SCA_SC_EVENT2_CMD_1_increment_event_src_SHIFT         0

/***************************************************************************
 *SC_EVENT2_CMD_2 - SMART CARD EVENT 2 COMMAND 2 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CMD_2 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CMD_2_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT2_CMD_2_reserved0_SHIFT                   8

/* SCA :: SC_EVENT2_CMD_2 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT2_CMD_2_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT2_CMD_2_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT2_CMD_2 :: increment_event_src [04:00] */
#define BCHP_SCA_SC_EVENT2_CMD_2_increment_event_src_MASK          0x0000001f
#define BCHP_SCA_SC_EVENT2_CMD_2_increment_event_src_SHIFT         0

/***************************************************************************
 *SC_EVENT2_CMD_3 - SMART CARD EVENT 2 COMMAND 3 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CMD_3 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CMD_3_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT2_CMD_3_reserved0_SHIFT                   8

/* SCA :: SC_EVENT2_CMD_3 :: RESERVED [07:05] */
#define BCHP_SCA_SC_EVENT2_CMD_3_RESERVED_MASK                     0x000000e0
#define BCHP_SCA_SC_EVENT2_CMD_3_RESERVED_SHIFT                    5

/* SCA :: SC_EVENT2_CMD_3 :: start_event_src [04:00] */
#define BCHP_SCA_SC_EVENT2_CMD_3_start_event_src_MASK              0x0000001f
#define BCHP_SCA_SC_EVENT2_CMD_3_start_event_src_SHIFT             0

/***************************************************************************
 *SC_EVENT2_CMD_4 - SMART CARD EVENT 2 COMMAND 4 REGISTER
 ***************************************************************************/
/* SCA :: SC_EVENT2_CMD_4 :: reserved0 [31:08] */
#define BCHP_SCA_SC_EVENT2_CMD_4_reserved0_MASK                    0xffffff00
#define BCHP_SCA_SC_EVENT2_CMD_4_reserved0_SHIFT                   8

/* SCA :: SC_EVENT2_CMD_4 :: event_en [07:07] */
#define BCHP_SCA_SC_EVENT2_CMD_4_event_en_MASK                     0x00000080
#define BCHP_SCA_SC_EVENT2_CMD_4_event_en_SHIFT                    7

/* SCA :: SC_EVENT2_CMD_4 :: RESERVED [06:04] */
#define BCHP_SCA_SC_EVENT2_CMD_4_RESERVED_MASK                     0x00000070
#define BCHP_SCA_SC_EVENT2_CMD_4_RESERVED_SHIFT                    4

/* SCA :: SC_EVENT2_CMD_4 :: intr_after_reset [03:03] */
#define BCHP_SCA_SC_EVENT2_CMD_4_intr_after_reset_MASK             0x00000008
#define BCHP_SCA_SC_EVENT2_CMD_4_intr_after_reset_SHIFT            3

/* SCA :: SC_EVENT2_CMD_4 :: intr_after_compare [02:02] */
#define BCHP_SCA_SC_EVENT2_CMD_4_intr_after_compare_MASK           0x00000004
#define BCHP_SCA_SC_EVENT2_CMD_4_intr_after_compare_SHIFT          2

/* SCA :: SC_EVENT2_CMD_4 :: run_after_reset [01:01] */
#define BCHP_SCA_SC_EVENT2_CMD_4_run_after_reset_MASK              0x00000002
#define BCHP_SCA_SC_EVENT2_CMD_4_run_after_reset_SHIFT             1

/* SCA :: SC_EVENT2_CMD_4 :: run_after_compare [00:00] */
#define BCHP_SCA_SC_EVENT2_CMD_4_run_after_compare_MASK            0x00000001
#define BCHP_SCA_SC_EVENT2_CMD_4_run_after_compare_SHIFT           0

#endif /* #ifndef BCHP_SCA_H__ */

/* End of File */
