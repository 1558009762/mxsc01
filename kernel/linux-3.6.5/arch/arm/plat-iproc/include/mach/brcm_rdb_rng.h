/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef __BRCM_RDB_RNG_H__
#define __BRCM_RDB_RNG_H__

#define RNG_CTRL_OFFSET                                                   0x00000000
#define RNG_CTRL_TYPE                                                     UInt32
#define RNG_CTRL_RESERVED_MASK                                            0xF00000CC
#define    RNG_CTRL_RNG_COMBLK2_OSC_DIS_SHIFT                             22
#define    RNG_CTRL_RNG_COMBLK2_OSC_DIS_MASK                              0x0FC00000
#define    RNG_CTRL_RNG_COMBLK1_OSC_DIS_SHIFT                             16
#define    RNG_CTRL_RNG_COMBLK1_OSC_DIS_MASK                              0x003F0000
#define    RNG_CTRL_RNG_JCLK_BYP_DIV_CNT_SHIFT                            8
#define    RNG_CTRL_RNG_JCLK_BYP_DIV_CNT_MASK                             0x0000FF00
#define    RNG_CTRL_RNG_JCLK_BYP_SRC_SHIFT                                5
#define    RNG_CTRL_RNG_JCLK_BYP_SRC_MASK                                 0x00000020
#define    RNG_CTRL_RNG_JCLK_BYP_SEL_SHIFT                                4
#define    RNG_CTRL_RNG_JCLK_BYP_SEL_MASK                                 0x00000010
#define    RNG_CTRL_RNG_RBG2X_SHIFT                                       1
#define    RNG_CTRL_RNG_RBG2X_MASK                                        0x00000002
#define    RNG_CTRL_RNG_RBGEN_SHIFT                                       0
#define    RNG_CTRL_RNG_RBGEN_MASK                                        0x00000001

#define RNG_STATUS_OFFSET                                                 0x00000004
#define RNG_STATUS_TYPE                                                   UInt32
#define RNG_STATUS_RESERVED_MASK                                          0x00F00000
#define    RNG_STATUS_RND_VAL_SHIFT                                       24
#define    RNG_STATUS_RND_VAL_MASK                                        0xFF000000
#define    RNG_STATUS_RNG_WARM_CNT_SHIFT                                  0
#define    RNG_STATUS_RNG_WARM_CNT_MASK                                   0x000FFFFF

#define RNG_DATA_OFFSET                                                   0x00000008
#define RNG_DATA_TYPE                                                     UInt32
#define RNG_DATA_RESERVED_MASK                                            0x00000000
#define    RNG_DATA_RNG_NUM_SHIFT                                         0
#define    RNG_DATA_RNG_NUM_MASK                                          0xFFFFFFFF

#define RNG_FF_THRES_OFFSET                                               0x0000000C
#define RNG_FF_THRES_TYPE                                                 UInt32
#define RNG_FF_THRES_RESERVED_MASK                                        0xFFFFFFE0
#define    RNG_FF_THRES_RNG_FF_THRESH_SHIFT                               0
#define    RNG_FF_THRES_RNG_FF_THRESH_MASK                                0x0000001F

#define RNG_INT_MASK_OFFSET                                               0x00000010
#define RNG_INT_MASK_TYPE                                                 UInt32
#define RNG_INT_MASK_RESERVED_MASK                                        0xFFFFFFFE
#define    RNG_INT_MASK_RNG_INT_OFF_SHIFT                                 0
#define    RNG_INT_MASK_RNG_INT_OFF_MASK                                  0x00000001

#endif /* __BRCM_RDB_RNG_H__ */
