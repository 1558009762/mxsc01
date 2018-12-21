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
#ifndef __MACH_DMUX_H
#define __MACH_DMUX_H

enum dmac_mux {
	KONA_DMUX_UARTB_A = 8,
	KONA_DMUX_UARTB_B = 9,
	KONA_DMUX_UARTB2_A = 10,
	KONA_DMUX_UARTB2_B = 11,
	KONA_DMUX_UARTB3_A = 12,
	KONA_DMUX_UARTB3_B = 13,
	KONA_DMUX_SSP_0A_RX0 = 16,
	KONA_DMUX_SSP_0B_TX0 = 17,
	KONA_DMUX_SSP_0C_RX1 = 18,
	KONA_DMUX_SSP_0D_TX1 = 19,
	KONA_DMUX_SSP_1A_RX0 = 20,
	KONA_DMUX_SSP_1B_TX0 = 21,
	KONA_DMUX_SSP_1C_RX1 = 22,
	KONA_DMUX_SSP_1D_TX1 = 23,
	KONA_DMUX_HSIA = 32,
	KONA_DMUX_HSIB = 33,
	KONA_DMUX_HSIC = 34,
	KONA_DMUX_HSID = 35,
	KONA_DMUX_EANC = 40,
	KONA_DMUX_STEREO = 41,
	KONA_DMUX_NVIN = 42,
	KONA_DMUX_VIN = 43,
	KONA_DMUX_VIBRA = 44,
	KONA_DMUX_IHF_0 = 45,
	KONA_DMUX_VOUT = 46,
	KONA_DMUX_SLIMA = 47,
	KONA_DMUX_SLIMB = 48,
	KONA_DMUX_SLIMC = 49,
	KONA_DMUX_SLIMD = 50,
	KONA_DMUX_SIM_A = 51,
	KONA_DMUX_SIM_B = 52,
	KONA_DMUX_SIM2_A = 53,
	KONA_DMUX_SIM2_B = 54,
	KONA_DMUX_IHF_1 = 55,
	KONA_DMUX_SSP_2A_RX0 = 56,
	KONA_DMUX_SSP_2B_TX0 = 57,
	KONA_DMUX_SSP_2C_RX1 = 58,
	KONA_DMUX_SSP_2D_TX1 = 59,
	KONA_DMUX_SPUM_SecureA = 65,
	KONA_DMUX_SPUM_SecureB = 66,
	KONA_DMUX_SPUM_OpenA = 67,
	KONA_DMUX_SPUM_OpenB = 68,
	KONA_DMUX_INVALID = 0x7f,
};

enum dma_peri dmux_name_to_id(const char *name);
int dmux_id_to_name(enum dma_peri peri, char *pname);
int dmux_sema_protect(void);
int dmux_sema_unprotect(void);
int dmux_alloc_channel(u32 * pchan);
int dmux_release_channel(u32 channel);
int dmux_alloc_peripheral(u32 channel, enum dma_peri peri, u8 * peri_req_id);
int dmux_alloc_multi_peripheral(u32 channel, enum dma_peri a, enum dma_peri b,
				u8 * src_id, u8 * dst_id);
int dmux_dealloc_peripheral(u32 channel);

#endif /* __MACH_DMUX_H */
