/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
// endian agnostic wrapper of the shmoo signature which used to
// be a bitfield
struct shmoo_signature {
    char            magic[4];
    uint16_t        dev_id;
    uint16_t        sku_id;
    uint8_t         ddr_type;
    uint16_t        ddr_clock;
};

void shmoo_mem2sig(uint8_t *mem, struct shmoo_signature *sig);
void shmoo_sig2mem(struct shmoo_signature *sig, uint8_t *mem);
int shmoo_sigmemcmp(struct shmoo_signature *sig, uint8_t *mem);
void shmoo_dump_sig(struct shmoo_signature *sig);


