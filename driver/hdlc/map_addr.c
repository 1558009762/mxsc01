#include <stdio.h>
#include "map_addr.h"

static const t_map_addr_para	_map_addr_table[]	= {
	{0x08000000, 0x10000},
};

t_map_info		g_map_info[MAP_ADDR_MAX];

void map_addr_init( void )
{
	int		i;

	for( i = 0; i < MAP_ADDR_MAX; ++i )
	{
		get_vaddr( _map_addr_table[i].addr, _map_addr_table[i].span, &g_map_info[i] );
	}
}

void map_addr_free( void )
{
	int		i;

	for( i = 0; i < MAP_ADDR_MAX; ++i )
	{
		clr_vaddr( &g_map_info[i] );
	}
}

uint32_t map_read(uint32_t phy_add, uint32_t addr)
{
	uint32_t value;
	//IORD(BASE_MAP_ADDR(phy_add), addr, value);
	//printf("mread : addr = 0x%X, value = 0x%X\n", addr, value);
	return value;
}

void map_write(uint32_t phy_add, uint32_t addr, uint32_t value)
{
	//IOWR(BASE_MAP_ADDR(phy_add), addr, value);
	//printf("mwrite : addr = 0x%X, value = 0x%X\n", addr, value);
}

