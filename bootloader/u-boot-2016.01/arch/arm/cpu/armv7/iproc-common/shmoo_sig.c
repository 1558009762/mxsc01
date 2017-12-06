/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <stdint.h>

#include "shmoo_sig.h"

uint8_t shmoo_mem[] = {0x53,0x48,0x4D,0x4F,0x14,0xDC,0x00,0x00,0xB9,0x0b,0x00,0x00};

#if 0
this is not mixed endian friendly
struct shmoo_sig_t {
    char            magic[4];

    uint32_t        dev_id:16;
    uint32_t        sku_id:16;

    uint32_t        ddr_type:2;
    uint32_t        ddr_clock:12;
    uint32_t        reserved1:18;
}
#endif

void shmoo_mem2sig(uint8_t *mem, struct shmoo_signature *sig)
{
    memcpy(sig->magic,mem,4);
    sig->dev_id = mem[4] | ((uint16_t)mem[5] << 8);
    sig->sku_id = mem[6] | ((uint16_t)mem[7] << 8);
    sig->ddr_type = mem[8] & 0x03;
    sig->ddr_clock = (mem[8] & 0xFC | ((uint16_t)mem[9] << 8)) >> 2;
}

void shmoo_sig2mem(struct shmoo_signature *sig, uint8_t *mem)
{
    memset(mem,0,10);
    memcpy(mem,sig->magic,4);
    mem[4] = sig->dev_id & 0xff;
    mem[5] = sig->dev_id >> 8;
    mem[6] = sig->sku_id & 0xff;
    mem[7] = sig->sku_id >> 8;
    mem[8] = sig->ddr_type;
    mem[8] |= (sig->ddr_clock << 2) & 0xFF;
    mem[9] |= sig->ddr_clock >> 6;
}

int shmoo_sigmemcmp(struct shmoo_signature *sig, uint8_t *mem)
{
    uint8_t tmp[10];
    shmoo_sig2mem(sig,tmp);
#if 0
    printf("%s tmp: %02X %02X %02X %02X %02x %02x %02x %02x %02x %02x\n",
        __func__,tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8], tmp[9]);
    printf("%s mem: %02X %02X %02X %02X %02x %02x %02x %02x %02x %02x\n",
        __func__,mem[0], mem[1], mem[2], mem[3], mem[4], mem[5], mem[6], mem[7], mem[8], mem[9]);
#endif
    return(memcmp(tmp,mem,sizeof(tmp)));
}


void shmoo_dump_sig(struct shmoo_signature *sig)
{
    printf("shmoo sig: magic:%c%c%c%c dev_id:%x sku_id:%x ddr_type:%x ddr_clock:%x\n",
        sig->magic[0], sig->magic[1], sig->magic[2], sig->magic[3],
        sig->dev_id,sig->sku_id,sig->ddr_type,sig->ddr_clock);
}

#if 0
int main()
{
    struct shmoo_signature sig;
    uint8_t mem[10];

    shmoo_mem2sig(shmoo_mem,&sig);
    shmoo_dump_sig(&sig);
    shmoo_sig2mem(&sig,mem);
    printf("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
        mem[0],mem[1],mem[2],mem[3],mem[4],mem[5],mem[6],mem[7],mem[8],mem[9]);
    printf("shmoo_sig_memcmp returned %d\n",shmoo_sigmemcmp(&sig,mem));
    return;
}
#endif
