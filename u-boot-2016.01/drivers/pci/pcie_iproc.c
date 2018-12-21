/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <pci.h>
#include <asm/io.h>
#include <post.h>
#include "linux/sizes.h"
#include "asm/types.h"
#include "asm/iproc-common/pcie_iproc.h"
#include "asm/arch/socregs.h"

extern void iproc_pcie_iomux(int op);
static unsigned long  PORT_IN_USE = 0;
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
static uint32_t swap_u32(uint32_t i) {
    uint8_t c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((uint32_t)c1 << 24) + ((uint32_t)c2 << 16) + ((uint32_t)c3 << 8) + c4;
}

static void pcie_iol_w(uint32_t val, volatile uint32_t *addr)
{
    *addr = swap_u32(val);
}

static uint32_t pcie_iol_r(volatile uint32_t *addr)
{
    return(swap_u32(*addr));
}

#else

static void pcie_iol_w(uint32_t val, volatile uint32_t *addr)
{
    *addr = val;
}

static uint32_t pcie_iol_r(volatile uint32_t *addr)
{
    return(*addr);
}
#endif

static 
struct soc_pcie_port soc_pcie_ports[3] = {
    {
    0x18012000,
    0x08000000,
    0
    },
    {
    0x18013000, 
    0x40000000,
    0
    },
    {
    0x18014000,
    0x48000000,
    0
    }
};


struct pci_controller pci_hoses[3];

static int conf_trace = 0;
static void pci_dump_standard_conf(struct soc_pcie_port * port);
static void pci_dump_extended_conf(struct soc_pcie_port * port);

void pci_bus0_read_config_dword (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned long *val)
{
    if(conf_trace) printf("pci_bus0_read_config_word: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    * val = pcie_iol_r((volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA));

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %04lx\n\n", dev, where, *val);
}

void pci_bus0_read_config_word (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned short *val)
{
    unsigned int tmp;
    if(conf_trace) printf("pci_bus0_read_config_word: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    tmp = pcie_iol_r((volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA));
    *val = (tmp >> (8 * (where & 3))) & 0xffff;

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %04x\n\n", dev, where, *val);
}

void pci_bus0_read_config_byte (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned char *val)
{
    unsigned int tmp;
    if(conf_trace) printf("pci_bus0_read_config_byte: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    tmp = pcie_iol_r((volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA));
    *val = (tmp >> (8 * (where & 3))) & 0xff;

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %02x\n\n", dev, where, *val);
}

void pci_bus0_write_config_dword (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned long val)
{
    if(conf_trace) printf("pci_bus0_write_config_dword: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %04lx\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where, val);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    pcie_iol_w( val, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA) );

    if(conf_trace) printf("pci_bus0_write_config_dword write done\n");
}

void pci_bus0_write_config_word (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned short val)
{
    unsigned int tmp;

    if(conf_trace) printf("pci_bus0_write_config_word: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %04x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where, val);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    tmp = pcie_iol_r((volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA));

    if(conf_trace) printf("pci_bus0_write_config_word read first: dev: %08x <B%d, D%d, F%d>, where: %08x, tmp_val: %04x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where, tmp);

    tmp &= ~(0xffff << (8 * (where & 3)) );
    tmp |= (val << (8 * (where & 3)) );

    if(conf_trace) printf("pci_bus0_write_config_word write back: dev: %08x <B%d, D%d, F%d>, where: %08x, tmp_val: %04x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where, tmp);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    pcie_iol_w( tmp, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA) );

    if(conf_trace) printf("pci_bus0_write_config_word write done\n");
}

void pci_bus0_write_config_byte (struct soc_pcie_port *pcie_port, pci_dev_t dev, int where, unsigned char val)
{
    unsigned int tmp;

    if(conf_trace) printf("pci_bus0_write_config_byte: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %02x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), where, val);

    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    tmp = pcie_iol_r((volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA));
    tmp &= ~(0xff << (8 * (where & 3)) );
    tmp |= (val << (8 * (where & 3)) );
    pcie_iol_w( where & 0xffc, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_ADDR) );
    pcie_iol_w( tmp, (volatile uint32_t *)((pcie_port->reg_base) + SOC_PCIE_EXT_CFG_DATA) );
}

static int __pci_bus0_find_next_cap_ttl(struct soc_pcie_port *pcie_port, unsigned int devfn,
        u8 pos, int cap, int *ttl)
{
    u8 id;

    while ((*ttl)--) {
        pci_bus0_read_config_byte(pcie_port, devfn, pos, &pos);
        if (pos < 0x40)
            break;
        pos &= ~3;
        pci_bus0_read_config_byte(pcie_port,devfn, pos + PCI_CAP_LIST_ID,
                &id);
        if (id == 0xff)
            break;
        if (id == cap)
            return pos;
        pos += PCI_CAP_LIST_NEXT;
    }
    return 0;
}

static int __pci_bus0_find_next_cap(struct soc_pcie_port *pcie_port, unsigned int devfn,
                                u8 pos, int cap)
 {
         int ttl = PCI_FIND_CAP_TTL;

         return __pci_bus0_find_next_cap_ttl(pcie_port, devfn, pos, cap, &ttl);
 }


static int __pci_bus0_find_cap_start(struct soc_pcie_port *pcie_port,unsigned int devfn, u8 hdr_type)
 {
         u16 status;

         pci_bus0_read_config_word(pcie_port, devfn, PCI_STATUS, &status);
         if (!(status & PCI_STATUS_CAP_LIST))
                 return 0;

         switch (hdr_type) {
         case PCI_HEADER_TYPE_NORMAL:
         case PCI_HEADER_TYPE_BRIDGE:
                 return PCI_CAPABILITY_LIST;
         case PCI_HEADER_TYPE_CARDBUS:
                 return PCI_CB_CAPABILITY_LIST;
         default:
                 return 0;
         }

         return 0;
}

static int pci_bus0_find_capability(struct soc_pcie_port *pcie_port,unsigned int devfn, int cap)
{
         int pos;
         u8 hdr_type;

         pci_bus0_read_config_byte(pcie_port,devfn, PCI_HEADER_TYPE, &hdr_type);

         pos = __pci_bus0_find_cap_start(pcie_port, devfn, hdr_type & 0x7f);
         if (pos)
             pos = __pci_bus0_find_next_cap(pcie_port, devfn, pos, cap);

         return pos;
 }
//Link status register definitions



//3:0
//RO
//Link Speed. The negotiated Link speed.
//   0001b = 2.5 Gb/s
//All other encodings are reserved.

//9:4
//RO
//Negotiated Link Width. The negotiated Link width.
//   000001b = x1
//   000010b = x2
//   000100b = x4
//   001000b = x8
//   001100b = x12
//   010000b = x16
//   100000b = x32
//   All other encodings are reserved.

//10
//RO
//Training Error. 1 = indicates that a Link training error occurred. Reserved on Endpoint devices and Switch upstream ports.
//Cleared by hardware upon successful training of the Link to the L0 Link state.

//11
//RO
//Link Training. When set to one, indicates that Link training is in progress (Physical Layer LTSSM is
//in the Configuration or Recovery state) or that the Retrain Link bit was set to one but Link training has not yet begun.
//Hardware clears this bit once Link training is complete.
// This bit is not applicable and reserved on Endpoint devices and the Upstream Ports of Switches.

//12
//HWInit
//Slot Clock Configuration. This bit indicates that the component uses the same physical reference clock
//that the platform provides on the connector. If the device uses an independent clock irrespective of the
//presence of a reference on the connector, this bit must be clear.

int soc_pcie_check_link(int port)
{
    u32 devfn = 0;
    u16 pos, tmp16;
    u8 nlw, tmp8;
    struct soc_pcie_port * pcie_port = &soc_pcie_ports[port];
    //struct pci_controller *hose = &pci_hoses[port];
    pcie_port->linkError = 0;

    /* See if the port is in EP mode, indicated by header type 00 */
    pci_bus0_read_config_byte(pcie_port, devfn, PCI_HEADER_TYPE, &tmp8);
    if( tmp8 != PCI_HEADER_TYPE_BRIDGE ) {
        printf("PCIe port %d in End-Point mode - ignored\n", port);
        pcie_port->linkError = 1;
        return -1;
    }
    else
        printf("PCIe port %d in RC mode\n" , port);

    /* NS PAX only changes NLW field when card is present */
    pos = pci_bus0_find_capability(pcie_port,devfn, PCI_CAP_ID_EXP);
#ifdef DEBUG
    printf("\n pos is %d\n", pos);
#endif
    pci_bus0_read_config_word(pcie_port,devfn, pos + PCI_EXP_LNKSTA, &tmp16);

    printf("==>PCIE: LINKSTA reg %#x val %#x\n",
        pos+PCI_EXP_LNKSTA, tmp16 );



    nlw = (tmp16 & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT ;
    if ( nlw == 0 )
    {
        pcie_port->linkError = 1;
    }
    //port->link = tmp16 & PCI_EXP_LNKSTA_DLLLA ;

    //if( nlw != 0 ) port->link = 1;
#if 0
    for( ; pos < 0x100; pos += 2 )
        {
            pci_bus0_read_config_word(pcie_port, devfn, pos , &tmp16);
            if( tmp16 ) printf("reg[%#x]=%#x, ", pos , tmp16 );
        }
    //printf("PCIE link=%d\n", port->link );
#endif

    return( (nlw)? 0: -1);
}

/*
 * Initializte the PCIe controller
 */
static void  soc_pcie_hw_init(struct soc_pcie_port * port)
{
    /* Turn-on Root-Complex (RC) mode, from reset defailt of EP */

    /* The mode is set by straps, can be overwritten via DMU
       register <cru_straps_control> bit 5, "1" means RC
     */

    /* Send a downstream reset */
	pcie_iol_w( 0x0, (volatile uint32_t *)(port->reg_base + SOC_PCIE_CONTROL));
    udelay(250);
	pcie_iol_w( 0x1, (volatile uint32_t *)(port->reg_base + SOC_PCIE_CONTROL));
    mdelay(250);
#ifdef DEBUG
	printf("\n soc_pcie_hw_init : port->reg_base = 0x%x , its value = 0x%x \n", port->reg_base, pcie_iol_r((volatile uint32_t *)(port->reg_base + SOC_PCIE_CONTROL)));
#endif
    /* TBD: take care of PM, check we're on */
}


int iproc_pcie_rd_conf_dword(struct pci_controller *hose, pci_dev_t dev, int offset, u32 * value)
{
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_rd_conf_dword: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

	*value = pcie_iol_r((volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %08x\n\n", dev, offset, *value);

    return PCIBIOS_SUCCESSFUL;
}

int iproc_pcie_rd_conf_word(struct pci_controller *hose, pci_dev_t dev, int offset, u16 * value)
{
    volatile unsigned int tmp;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_rd_conf_word: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

	tmp = pcie_iol_r((volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));
    *value = (tmp >> (8 * (offset & 3))) & 0xffff;

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %04x\n\n", dev, offset, *value);

    return PCIBIOS_SUCCESSFUL;
}

int iproc_pcie_rd_conf_byte(struct pci_controller *hose, pci_dev_t dev, int offset, u8 * value)
{
    volatile unsigned int tmp;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_rd_conf_byte: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

	tmp = pcie_iol_r((volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));
    *value = (tmp >> (8 * (offset & 3))) & 0xff;

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %02x\n\n", dev, offset, *value);

    return PCIBIOS_SUCCESSFUL;
}

int iproc_pcie_wr_conf_dword(struct pci_controller *hose, pci_dev_t dev, int offset, u32 value)
{
    int ret = PCIBIOS_SUCCESSFUL;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_wr_conf_dword: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset, value); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

	pcie_iol_w(value, (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));
    return ret;
}

int iproc_pcie_wr_conf_word(struct pci_controller *hose, pci_dev_t dev, int offset, u16 value)
{
    int ret = PCIBIOS_SUCCESSFUL;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_wr_conf_word: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %04x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset, value); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

    __raw_writew(value, (u32)(pcie_port->reg_base) + PCIE_CONF_DATA_OFF + (offset & 3) );
    return ret;
}

int iproc_pcie_wr_conf_byte(struct pci_controller *hose, pci_dev_t dev, int offset, u8 value)
{
    int ret = PCIBIOS_SUCCESSFUL;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_wr_conf_byte: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %02x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset, value); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset),
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

    __raw_writeb(value, (u32)(pcie_port->reg_base) + PCIE_CONF_DATA_OFF + (offset & 3) );
    return ret;
}

#if 0
int pci_read_config_dword (pci_dev_t dev, int where, unsigned int *val)
{
    return iproc_pcie_rd_conf_dword(&pci_hoses[PORT_IN_USE], dev, where, val);
}

int pci_read_config_word (pci_dev_t dev, int where, unsigned short *val)
{
    return iproc_pcie_rd_conf_word(&pci_hoses[PORT_IN_USE], dev, where, val);
}

int pci_read_config_byte (pci_dev_t dev, int where, unsigned char *val)
{
    return iproc_pcie_rd_conf_byte(&pci_hoses[PORT_IN_USE], dev, where, val);
}

int pci_write_config_dword (pci_dev_t dev, int where, unsigned int val)
{
    return iproc_pcie_wr_conf_dword(&pci_hoses[PORT_IN_USE], dev, where, val);
}

int pci_write_config_word (pci_dev_t dev, int where, unsigned short val)
{
    return iproc_pcie_wr_conf_word(&pci_hoses[PORT_IN_USE], dev, where, val);
}

int pci_write_config_byte (pci_dev_t dev, int where, unsigned char val)
{
    return iproc_pcie_wr_conf_byte(&pci_hoses[PORT_IN_USE], dev, where, val);
}
#endif

/*
 * Setup the address translation
 */
static void soc_pcie_map_init(struct soc_pcie_port * port)
{

    /* pass thru' address translation */
#if 0
    unsigned size, i ;
    u32 addr;

    /*
     * NOTE:
     * All PCI-to-CPU address mapping are 1:1 for simplicity
     */

    /* Outbound address translation setup */
    size = SZ_128M;
    addr = OUT_PCI_ADDR;

    for(i=0 ; i < 3; i++)
        {
        const unsigned win_size = SZ_64M;
        /* 64-bit LE regs, write low word, high is 0 at reset */
		pcie_iol_w( addr, port->reg_base + SOC_PCIE_SYS_OMAP(i));
		pcie_iol_w( addr|0x1, port->reg_base + SOC_PCIE_SYS_OARR(i));
        addr += win_size;
        if( size >= win_size )
            size -= win_size;
        if( size == 0 )
            break;
        }

    /* 
     * Inbound address translation setup
     * Northstar only maps up to 128 MiB inbound, DRAM could be up to 1 GiB.
     *
     * For now allow access to entire DRAM, assuming it is less than 128MiB,
     * otherwise DMA bouncing mechanism may be required.
     * Also consider DMA mask to limit DMA physical address
     */

    size = SZ_64M;
    addr = IN_DDR_ADDR;

    size >>= 20;    /* In MB */
    size &= 0xff;   /* Size is an 8-bit field */

    /* 64-bit LE regs, write low word, high is 0 at reset */
	pcie_iol_w(addr | size | 0x1,
        port->reg_base + SOC_PCIE_SYS_IMAP1(0));
	pcie_iol_w(addr | 0x1,
        port->reg_base + SOC_PCIE_SYS_IARR(1));
#endif
}


/*
 * Setup PCIE Host bridge
 */
static void soc_pcie_bridge_init(struct soc_pcie_port * port)
{
        u32 devfn = 0;
        u8 tmp8;
        u16 tmp16;
        u32 mem_size = SZ_128M;
        
        /* Fake <bus> object */
        pci_bus0_read_config_byte(port, devfn, PCI_PRIMARY_BUS, &tmp8);
        pci_bus0_read_config_byte(port, devfn, PCI_SECONDARY_BUS, &tmp8);
        pci_bus0_read_config_byte(port, devfn, PCI_SUBORDINATE_BUS, &tmp8);

        pci_bus0_write_config_byte(port, devfn, PCI_PRIMARY_BUS, 0);
        pci_bus0_write_config_byte(port, devfn, PCI_SECONDARY_BUS, 1);
        pci_bus0_write_config_byte(port, devfn, PCI_SUBORDINATE_BUS, 4);

        pci_bus0_read_config_byte(port, devfn, PCI_PRIMARY_BUS, &tmp8);
        pci_bus0_read_config_byte(port, devfn, PCI_SECONDARY_BUS, &tmp8);
        pci_bus0_read_config_byte(port, devfn, PCI_SUBORDINATE_BUS, &tmp8);

        printf("membase %#x memlimit %#x\n",
               ((u32)port->out_pci_addr), ((u32)port->out_pci_addr) + mem_size);

        pci_bus0_read_config_word(port, devfn, PCI_CLASS_DEVICE, &tmp16);
        pci_bus0_read_config_word(port, devfn, PCI_MEMORY_BASE, &tmp16);
        pci_bus0_read_config_word(port, devfn, PCI_MEMORY_LIMIT, &tmp16);

        pci_bus0_write_config_word(port, devfn, PCI_MEMORY_BASE, 
               ((u32)port->out_pci_addr) >> 16 );
        pci_bus0_write_config_word(port, devfn, PCI_MEMORY_LIMIT, 
               (((u32)port->out_pci_addr) + mem_size) >> 16 );

    /* Force class to that of a Bridge */
        pci_bus0_write_config_word(port, devfn, PCI_CLASS_DEVICE,
        PCI_CLASS_BRIDGE_PCI);

        pci_bus0_read_config_word(port, devfn, PCI_CLASS_DEVICE, &tmp16);
        pci_bus0_read_config_word(port, devfn, PCI_MEMORY_BASE, &tmp16);
        pci_bus0_read_config_word(port, devfn, PCI_MEMORY_LIMIT, &tmp16);
    
}

/* ****************************************************
 * this function is only applicable to type 1 command
 ******************************************************/
int pcie_diag_wr_conf_dword(struct pci_controller *hose, pci_dev_t dev, int offset, u32 value)
{
    int ret = PCIBIOS_SUCCESSFUL;
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);

    if(conf_trace) printf("Pcie_wr_conf_dword: dev: %08x <B%d, D%d, F%d>, where: %08x, val: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset, value); 
		pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset) | 0x01,
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

		pcie_iol_w(value, (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));
    return ret;
}

/* ****************************************************
 * this function is only applicable to type 1 command
 ******************************************************/
int pcie_diag_rd_conf_dword(struct pci_controller *hose, pci_dev_t dev, int offset, u32 * value)
{
    struct soc_pcie_port *pcie_port = (struct soc_pcie_port *)(hose->priv_data);
    
    if(conf_trace) printf("Pcie_rd_conf_dword: dev: %08x <B%d, D%d, F%d>, where: %08x\n", 
                dev, PCI_BUS(dev), PCI_DEV(dev), PCI_FUNC(dev), offset); 
	pcie_iol_w(PCIE_CONF_BUS( PCI_BUS(dev) ) |
                    PCIE_CONF_DEV( PCI_DEV(dev) ) |
                    PCIE_CONF_FUNC( PCI_FUNC(dev) ) |
                    PCIE_CONF_REG(offset)|0x01,
                    (volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_ADDR_OFF));

	*value = pcie_iol_r((volatile uint32_t *)(pcie_port->reg_base + PCIE_CONF_DATA_OFF));

    if(conf_trace) printf("Return : dev: %08x, where: %08x, val: %08x\n\n", dev, offset, *value);

    return PCIBIOS_SUCCESSFUL;
}

extern void pci_iproc_set_port(int port);
void pci_iproc_init_board (int port)
{
    struct pci_controller * hose = &pci_hoses[port];
    struct soc_pcie_port *pcie_port = &soc_pcie_ports[port];

	pci_iproc_set_port(port);

    soc_pcie_hw_init(pcie_port);
    
    hose->priv_data =(void *)pcie_port;
    hose->first_busno = 0;
    hose->last_busno = 0;
    hose->current_busno = 0;
    
    pci_set_ops(hose,
            iproc_pcie_rd_conf_byte,
            iproc_pcie_rd_conf_word,
            iproc_pcie_rd_conf_dword,
            iproc_pcie_wr_conf_byte,
            iproc_pcie_wr_conf_word,
            iproc_pcie_wr_conf_dword );

    pci_register_hose(hose);

    udelay(1000);

    soc_pcie_map_init(pcie_port);

    if( soc_pcie_check_link(port) != 0 )
    {
        printf("\n**************\n port %d is not active!!\n**************\n",port);
        return;
    }
    printf("\n**************\n port %d is active!!\n**************\n",port);
	
	
    pci_iproc_set_port(port);
    
    soc_pcie_bridge_init(pcie_port);

    pci_hose_config_device(hose, PCI_BDF(1,0,0), (unsigned long)NULL, (u32)soc_pcie_ports[port].out_pci_addr, 0x146);
    pci_write_config_word(0, 0x3c, 0x1a9);  /* Set IRQ */
    pci_bus0_write_config_word(pcie_port, 0, 0x3c, 0x1a9);
    /* Set bridge */
    pci_bus0_write_config_byte(pcie_port, 0, 0x1b, 0x00);  // latency timer
    pci_bus0_write_config_word(pcie_port, 0, 0x28, 0x00);  // prefetch_base
    pci_bus0_write_config_word(pcie_port, 0, 0x2a, 0x00);
    pci_bus0_write_config_word(pcie_port, 0, 0x4,  0x146);  // cmd

#ifdef CONFIG_PCI_SCAN_SHOW
    printf("\nExtended Config\n");
    pci_dump_extended_conf(pcie_port);

    printf("\nStandard Config\n");
    pci_dump_standard_conf(pcie_port);

    hose->last_busno = pci_hose_scan(hose);
    //hose->last_busno = 1;
    printf("\n pci_iproc_init_board : hose->last_busno = 0x%x \n", hose->last_busno);
#endif

    printf("Done PCI initialization\n");


}

static void pci_dump_standard_conf(struct soc_pcie_port * port)
{
    unsigned int i, val, cnt;



    /* Disable trace */
    conf_trace = 0;

    for(i=0; i<64; i++)
    {      
        iproc_pcie_rd_conf_dword(&pci_hoses[PORT_IN_USE], 0, i*4, &val);

        cnt = i % 4;
        if(cnt==0) printf("i=%d <%x> \t 0x%08x \t", i, i, val);
        else if(cnt==3) printf("0x%08x \n", val);
        else printf("0x%08x \t", val);
    }
    printf("\n");

    for(i=0; i<6; i++)
    {      
        iproc_pcie_rd_conf_dword(&pci_hoses[PORT_IN_USE], 0, 0x10+i*4, &val);
        printf(" BAR-%d: 0x%08x\n\n", i, val);
    }
}

static void pci_dump_extended_conf(struct soc_pcie_port * port)
{
    unsigned int i,cnt;
    unsigned short val16;

    conf_trace = 0;

    for(i=0; i<128; i++)
    {
        pci_bus0_read_config_word(port, 0, i*2, &val16);

        cnt = i % 8;
        if(cnt==0) printf("i=%d <%x> \t 0x%04x  ", i, i, val16);
        else if(cnt==7) printf("0x%04x \n", val16);
        else printf("0x%04x  ", val16);
    }
    printf("\n");
}


int pci_skip_dev(struct pci_controller *hose, pci_dev_t dev)
{
    return 0;
}

#ifdef CONFIG_PCI_SCAN_SHOW
int pci_print_dev(struct pci_controller *hose, pci_dev_t dev)
{
    return 1;
}
#endif /* CONFIG_PCI_SCAN_SHOW */

extern void pci_unregister_hoses(void);

/* set the current working slot*/
void pci_iproc_set_port(int port)
{
    PORT_IN_USE = port;
}

int iproc_pcie_get_link_status(int port) 
{
    return  soc_pcie_ports[port].linkError;
}

u32 pci_iproc_get_pci_addr(int port)
{
    return soc_pcie_ports[port].out_pci_addr;
}

u32 pci_iproc_get_reg_base(int port)
{
    return soc_pcie_ports[port].reg_base;
}

/* Probe function. */
void pci_init_board(void)
{
#if defined(CONFIG_PCI_BOOTDELAY)
    char *s;
    int i;

    /* wait "pcidelay" ms (if defined)... */
    s = getenv ("pcidelay");
    if (s) {
        int val = simple_strtoul (s, NULL, 10);
        for (i=0; i<val; i++)
            udelay (1000);
    }
#endif /* CONFIG_PCI_BOOTDELAY */
   
    pci_iproc_init_board(PORT_IN_USE);
}
