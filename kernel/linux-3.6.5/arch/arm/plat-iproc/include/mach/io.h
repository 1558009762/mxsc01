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
#ifndef __PLAT_IPROC_IO_H
#define __PLAT_IPROC_IO_H

//#define IO_SPACE_LIMIT		(0xffffffff)

//#define __io(a)			__typesafe_io(a)
#define __mem_pci(a)		(a)

//#ifdef __ASSEMBLER__
//#define IOMEM(x)		(x)
//#else
//#define IOMEM(x)		((void __force __iomem *)(x))
//#endif

#define VC_DIRECT_ACCESS_BASE		0xC0000000UL
#define ARM_VC_PHYS_ADDR_BASE		0x40000000UL
#define __VC_BUS_TO_ARM_PHYS_ADDR(x)	((x) - (VC_DIRECT_ACCESS_BASE) + \
                                        (ARM_VC_PHYS_ADDR_BASE))

#endif /*__PLAT_IPROC_IO_H */
