#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "map_addr.h"
#include "hdlc.h"

int main(int argc, char **argv)
{
	int i;
	uint32_t add,slot,id_slot;
	fpga_id_ver				id_ver_s;	// start
	fpga_id_ver				id_ver_e;	// end
	ushort			ddm_base[0x02],data;
	
	map_addr_init();

	
	if(argc < 6)
	{
		if(argv[1] != NULL && strcmp(argv[1], "r") == 0)
		{
			if(argv[2] == NULL)
			{
				printf("Invalid args.\n");
				exit(0);
			}
			else
			{
				slot = (uint32_t)strtoul(argv[2], NULL, 0);
				add = (uint32_t)strtoul(argv[3], NULL, 0);
				//fpga_rm_rd_set( ic_rd_rt,  0, 1, slot, slot * FPGA_CARD_SIZE );
				rt_read_fpga( ddm_base, slot, add, 0x02 );
				
				//lp_rd_get_id( slot, &id_ver_s, &id_ver_e );

				printf("add %0x = 0x%8x\n", add+0,ddm_base[0]);//id_ver_s.bits.id);
				printf("add %0x = 0x%8x\n", add+1,ddm_base[1]);//id_ver_s.bits.id);
				
			}
		}
		else if(argv[1] != NULL && strcmp(argv[1], "w") == 0)
		{
			
			if(argv[2] == NULL)
			{
				printf("Invalid args.\n");
				exit(0);
			}
			else
			{
				slot = (uint32_t)strtoul(argv[2], NULL, 0);
				add = (uint32_t)strtoul(argv[3], NULL, 0);
				data = (uint32_t)strtoul(argv[4], NULL, 0);
				printf("hdlc write slot == %d,add ==%x,data==%x\n",slot,add,data);
				add = add+slot*FPGA_CARD_SIZE;
				fpga_rm_wr(add,data);
			}
		}
		else
		{
			printf("para err !!!\n");
			exit(0);
		}
	}
	
	
	return 0;
}
