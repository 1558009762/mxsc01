#ifndef _MAP_ADDR_H_
#define _MAP_ADDR_H_

#include <stdint.h>
#include "mmu_vir_addr.h"

#define BASE_MAP_ADDR(n)	(g_map_info[n].map_opt_addr)

typedef enum
{
	ADDR_HDLC = 0,
	MAP_ADDR_MAX
}	e_map_addr_type;

typedef struct
{
	unsigned int		addr;
	unsigned int		span;	// size = span + 1
}	t_map_addr_para;

extern t_map_info		g_map_info[MAP_ADDR_MAX];

// -------------------------------------
extern void map_addr_init( void );
extern void map_addr_free( void );

#if 0
#define IORD(address, offset, value)	do{ \
	(value) = (*(volatile unsigned *)((address)+4*(offset))); \
} while(0);

#define IOWR(address, offset, value)	do{ \
	(*(volatile unsigned *)((address)+4*(offset))) = (value); \
} while(0);
#endif
uint32_t map_read(uint32_t phy_add, uint32_t addr);
void map_write(uint32_t phy_add, uint32_t addr, uint32_t value);

#endif

