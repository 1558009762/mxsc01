#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <errno.h>
#include "mmu_vir_addr.h"

int get_vaddr( unsigned int phy_addr, unsigned int reglen, t_map_info * pmap )
{
	unsigned int	vir_addr	= 0;
	unsigned int	pghead;
	unsigned int	offset;
	unsigned int	maplen;
	int				memfd;

	memfd		= open("/dev/mem", O_RDWR | O_SYNC);
	if( memfd < 0 )
	{
		return 0;
	}

	pghead		= (unsigned int)(phy_addr & ~(sysconf(_SC_PAGE_SIZE) - 1)); // page head addr
	offset		= phy_addr - pghead;
	maplen		= reglen + offset;	// need get length
	// get virtual addr of the page
	vir_addr	= (unsigned int)mmap(NULL, maplen, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, pghead );

	if( 0 == vir_addr )
		return 0;

	pmap->map_vir_addr	= vir_addr;
	pmap->map_vir_size	= maplen;
	vir_addr			+= offset;
	pmap->map_opt_addr	= vir_addr;
	pmap->map_opt_size	= reglen;

	close( memfd );

	printf("map_opt_addr = 0x%X\n", vir_addr);
	
	return 1;
}

void clr_vaddr( t_map_info * pmap )
{
	if( pmap != NULL )
	{
		munmap((void *)pmap->map_vir_addr, pmap->map_vir_size);
	}
}

