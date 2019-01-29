#ifndef _MMU_VIT_ADDR_H_
#define _MMU_VIT_ADDR_H_

typedef struct
{
	unsigned int		map_vir_addr;
	unsigned int		map_vir_size;

	unsigned int		map_opt_addr;
	unsigned int		map_opt_size;
}	t_map_info;

extern int	get_vaddr		( unsigned int phy_addr, unsigned int reglen, t_map_info * );
extern void	clr_vaddr		( t_map_info * );

#endif

