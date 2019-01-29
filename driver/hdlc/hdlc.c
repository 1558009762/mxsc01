
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <unistd.h>
#include <sys/mman.h>
#include <assert.h>
#include <sys/time.h>
#include <time.h>
#include "map_addr.h"
#include "hdlc.h"


/* notes : ------------------------------------------------------------------------------ */
/*
	lc		:	local
	rm		:	remote
	ic		:	fpga
	rt		:	realtime
	sp		:	space
	sgl		:	single
	reg		:	register
*/


/* For debugging and testing ---------------------------------------------------------- */
/*#define SIMU_FPGA*/
/*#define FPGA_DEBUG*/

#ifdef FPGA_DEBUG
	static struct 
	{
		int				len;
		uint		    buf[100];
	}	ic_debug;
#endif

/* global variables */

/*
	NOTE: 
		In fact, the IC(fpga) is accessed ONLY with 32-bits width !!!
		So, addresses, data and lengths used here are all based on 32 bits !!!
*/
/*
#define CPUIF_BASE			0x100000				// ctrl local fpga 
#define CPUIF_SPAN			524288
#define NIOS_CUPIF_BASE	0x80000

const uint 			    ic_size		= 0x00080000;			// 0x00010000 bytes
static uint		   		ic_base ;
static uint		   		ic_base_local ;

static int				fpga_dev		= -1;
*/

/* loop && nrt read parameters */
#define IC_RD_BUF_WIDTH					6	// 数据缓冲区地址的宽度(bit)
#define IC_RD_BUF_NUM					(1 << IC_RD_BUF_WIDTH)		//缓存指令数
#define IC_RD3_BUF_NUM					(1 << (IC_RD_BUF_WIDTH - 1))//缓存指令数
#define IC_RT_RD_BUF_NUM				(1 << (IC_RD_BUF_WIDTH - 2))//缓存指令数
#define IC_RD_BLK_WIDTH					5	// 数据缓冲区数据长度的宽度(bit)

typedef union
{
#ifdef CROSS_COMPILE//songll
	struct
	{
		uint		type	: 3;//读取的类型
		uint		off		: 11;//数据缓冲区地址
		uint		addr	: 18;//读取数据的基地址
	}	sgl;//随机读取数据
#else
	struct
	{
		uint		addr	: 18;
		uint		off		: 11;
		uint		type	:  3;
	}	sgl;//随机读取数据
#endif
#ifdef CROSS_COMPILE//songll
	struct
	{
		uint		type	: 3;//读取的类型
		uint		buf		: IC_RD_BUF_WIDTH;//数据缓冲区地址
		uint		len		: IC_RD_BLK_WIDTH;//连续读取数据的长度
		uint		addr	: 18;//读取数据的基地址
	}	blk;//连续读取数据
#else
	struct
	{
		uint		addr	: 18;
		uint		len		: IC_RD_BLK_WIDTH;
		uint		buf		: IC_RD_BUF_WIDTH;
		uint		type	: 3;
	}	blk;//连续读取数据
#endif

	uint		whole;
}	ic_rd_op;				//非实时读指令的定义

/* fpga read instruction cache */
/* The order MUST be same with the enum ic_op_type !!! */		//保存的指令
ic_rd_op				ic_rd_icache[ic_op_end][IC_RD_BUF_NUM];

/* The order MUST be same with the enum ic_op_type !!! */		//操作空间
const ic_seg_sp ic_op_sp[ic_op_end]	= 	{
	/* non-realtime read operation space */
	{
		{ 0x00000400, IC_RD_BUF_NUM  },
		{ 0x00000400, 0x800  },					// size bases on 16-bits
	},

	/* loop read 1 operation space */
	{
		{ 0x00000800, IC_RD_BUF_NUM  },
		{ 0x00000800, 0x800  },					// size bases on 16-bits
	},
	
	/* loop read 2 operation space */
	{
		{ 0x00000C00, IC_RD_BUF_NUM  },
		{ 0x00000C00, 0x800  },					// size bases on 16-bits
	},

	/* realtime read operation space */	
	{
		{ 0x00001000, IC_RT_RD_BUF_NUM  },
		{ 0x00001000, 0x100  },					// size bases on 16-bits
	},

	/* realtime write operation space */
	{
		{ 0x00001400, 0x40 },					// size bases on 16-bits
		{ 0x00001400, 0  },
	},

	/* loop read 3 operation space */
	{
		{ 0x00001800, IC_RD3_BUF_NUM },			// size bases on 16-bits
		{ 0x00001800, 0x400  },
	}
};

static int rd_info[500][2];

uint IORD(uint address, uint offset)
{
	return (*(volatile unsigned *)((address)+4*(offset)));
}

void IOWR(uint address, uint offset, uint value)
{

	(*(volatile unsigned *)((address)+4*(offset))) = (value);
}


uint _ic_read( uint addr )
{
	return IORD(BASE_MAP_ADDR(ADDR_HDLC), addr);
}

void _ic_write( uint addr, uint value )
{
	IOWR(BASE_MAP_ADDR(ADDR_HDLC), addr, value);
}



/* ---------------------------------------------------------------------------------- */
/*
	read any registers of the fpga, directly
	return:
		>=0,	success
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
int fpga_rd( uint * pdata, uint addr, uint len )
{
    uint             i;

	for( i = 0; i < len; i++ )
	{
	    pdata[i]    = _ic_read(addr + i);
	}

	return len;
}


/* ---------------------------------------------------------------------------------- */
/*
	write any registers of the fpga, directly
	return:
		0,		success
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
int fpga_wr( uint addr, uint * pdata, uint len )
{
    uint                 i;

	for( i = 0; i < len; i++ )
	{
	    _ic_write(addr + i, pdata[i]);
	}
	
	return 0;
}
#if 1
/* ---------------------------------------------------------------------------------- */
/*
	realtime write, for remote registers
	input:
		addr	: base on 16-bits
		data	: bases on 16-bits
	return:
		0,		sucess
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
int fpga_rm_wr( uint addr, ushort data )
{
	uint			cardslot;
	uint			rmdata;
		
	/* the address is 18-bits, and the highest 4-bits is sent on the MPC address bus A5~A2
		the lowest 14-bits is sent with the data : 
		the MPC data bus = | RS-2bits | ADDR - 14bits | DATA - 16bits | */

	cardslot	= (addr >> 14) & 0x0F;
	rmdata		= ((addr & ((1L << 14) - 1)) << 16) | data;

	_ic_write(ic_op_sp[ic_rt_wr].opr.base + cardslot, rmdata);
	
	// must delay 5 usec for the FPGA dealing with the instruction
	delay_usec( 5 );

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= ic_op_sp[ic_rt_wr].opr.base + cardslot;
ic_debug.buf[ic_debug.len++]	= cardslot;
ic_debug.buf[ic_debug.len++]	= rmdata;
#endif

	return 0;
}
#endif

/* ---------------------------------------------------------------------------------- */
/*
	return the maxisize of a data block in the data buffer.	//每个指令对应的缓冲区大小
*/
uint fpga_blk_size( ic_op_type opt )
{
	return (ic_op_sp[opt].buf.size / ic_op_sp[opt].opr.size);	
}

/* ---------------------------------------------------------------------------------- */
/*
	read setting, for remote registers
		op		: ic_op_type 		< ic_rt_wr //read type over !
		cache	: the instruction register's position, and it is which read buffer segment( 0 ~ IC_RD_BUF_NUM-1 )
		len		: the length of the block (based on 16bits !!!) ( maxisize = fpga_blk_size( opt ) )
					0,		disable
					1,		single
					>1,		block
		off		: NO use
		addr	: the base of the registers
	return:
		>0,		success
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
//设置数组ic_base中cache地址处的值
int fpga_rm_rd_set( ic_op_type opt, uint cache, uint len, uint off, uint addr )
{
	ic_rd_op			* oper;
	const ic_seg_sp		* sp;
	uint		        seg_size;

	if( (opt >= ic_rt_wr && opt != ic_rd_lp3) || cache >= ic_op_sp[opt].opr.size )
		return -2;

	/* get pointers */
	oper		= &ic_rd_icache[opt][cache];
	sp			= &ic_op_sp[opt];
	seg_size	= fpga_blk_size( opt );		/* base on 16-bits */

	/* organize the operator */
	if( len == 0 )
	{
		oper->sgl.type	= 0;
		oper->sgl.off	= off;
		oper->sgl.addr	= addr;

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->opr.base + cache;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
#endif
	}
	else if( len == 1 )
	{
		oper->sgl.type	= 1;
		oper->sgl.off	= seg_size*cache;
		oper->sgl.addr	= addr;

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->opr.base + cache;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
ic_debug.buf[ic_debug.len++]	= oper->sgl.off;
ic_debug.buf[ic_debug.len++]	= oper->sgl.addr;
#endif
	}
	else
	{
		if( len > seg_size )
			return -2;
		oper->blk.type	= 2;
		oper->blk.buf	= cache;
		oper->blk.len	= len - 1;
		oper->blk.addr	= addr;

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->opr.base + cache;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->blk.type;
ic_debug.buf[ic_debug.len++]	= oper->blk.buf;
ic_debug.buf[ic_debug.len++]	= oper->blk.len;
ic_debug.buf[ic_debug.len++]	= oper->blk.addr;
#endif
	}
	/* set */
	_ic_write(sp->opr.base + cache, oper->whole);

	if((opt == 1) || (opt == 2) || (opt ==5))
	{
		for(int i = 0; i < 500; ++i)
		{
			if(rd_info[i][0] == 0)
			{
				rd_info[i][0] = sp->opr.base + cache;
				rd_info[i][1] = oper->whole;

				break;
			}
			else 
			{
				if(rd_info[i][0] == (int)(sp->opr.base + cache))
				{
					rd_info[i][0] = sp->opr.base + cache;
					rd_info[i][1] = oper->whole;

					break;
				}
				else
				{
					continue;
				}
			}
		}
	}
	
	return len;
}

void fpga_rm_rd_reset( )
{
	for(int i = 0; i < 500; ++i)
	{
		if(rd_info[i][1] != 0)
		{
			_ic_write(rd_info[i][0], rd_info[i][1]);
		}
		else
			continue;	
	}
}

void fpga_rm_rd_cl( )
{
	for(int i = 0; i < 500; ++i)
	{
		if(rd_info[i][1] != 0)
		{
			_ic_write(rd_info[i][0], 0x00);
		}
		else
			continue;	
	}
}
/* ---------------------------------------------------------------------------------- */
/*
	read instruction state
		op		: ic_op_type 		< ic_rt_wr //read type over !
		cache	: the instruction register's position, and it is which read buffer segment( 0 ~ IC_RD_BUF_NUM-1 )
		* plen	: return the length of the block (based on 16bits !!!) ( maxisize = fpga_blk_size( opt ) )
					0,		disable
					1,		single
					>1,		block
		* poff	: NO use
		* paddr	: return the base of the registers
		* pwhole: return the whole uint
	return:
		>0,		read len, base on 16-bits
		== 0, 	disable
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
//读取指令的类型，plen返回要读取的数据长度，paddr返回要读取的寄存器地址
int fpga_rm_rd_state( ic_op_type opt, uint cache, uint * plen, uint * poff, uint * paddr, uint * pwhole )
{
	ic_rd_op			* oper;
	const ic_seg_sp		* sp;


	if( (opt >= ic_rt_wr && opt != ic_rd_lp3) || cache >= ic_op_sp[opt].opr.size )
		return -2;
		
	/* get pointers */
	oper		= &ic_rd_icache[opt][cache];
	sp			= &ic_op_sp[opt];
	if( pwhole != NULL )
	{
		*pwhole	= oper->whole;
	}
	
	/* organize the operator && get */
	if( poff != NULL )
	{
		*poff	= oper->sgl.off;
	}
	if( paddr != NULL )
	{
		*paddr	= oper->sgl.addr;
	}
		
	if( oper->sgl.type == 1 )
	{
		if( plen != NULL )
		{
			*plen	= 1;
		}
		
		return 1;
	}
	else if( oper->blk.type == 2 )
	{
		if( plen != NULL )
		{
			*plen	= oper->blk.len + 1;
			return *plen;
		}
	}
	else
	{
		if( plen != NULL )
		{
			*plen	= 0;
		}
	}

	return 0;
}

/* ---------------------------------------------------------------------------------- */
/*
	read a block of bytes, for remote registers
		op		: ic_op_type 		< ic_rt_wr //read type over !
		cache	: the instruction register's position, and it is which read buffer segment( 0 ~ IC_RD_BUF_NUM-1 )
		pdata	: data
	return:
		>0,		read len, base on 16-bits
		== 0, 	disable
		-1,		fail to open the driver
		-2,		some parameters are out of the valid range
*/
//读取数据到pdata中
int fpga_rm_rd( ic_op_type opt, uint cache, ushort * pdata )
{
	ic_rd_op			* oper;
	const ic_seg_sp		* sp;
	int					seg_size;
	uint		        i, tmp, len, reg_num;
	uint				offset;


	if( (opt >= ic_rt_wr && opt != ic_rd_lp3) || cache >= ic_op_sp[opt].opr.size || pdata == NULL )
		return -2;

	/* get pointers */
	oper		= &ic_rd_icache[opt][cache];
	sp			= &ic_op_sp[opt];
	seg_size	= fpga_blk_size( opt );		/* base on 16-bits *///指令缓冲区的大小
	
	/* organize the operator && get 随机读数据*/
	if( oper->sgl.type == 1 )
	{
		/* the off bases on 16-bits */
		reg_num		= oper->sgl.off >> 1;
		tmp			= _ic_read(sp->buf.base + reg_num);
		pdata[0]	= (oper->sgl.off & 1) ? ((ushort)(tmp >> 16)) : ((ushort)tmp);

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->buf.base + oper->sgl.off;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
ic_debug.buf[ic_debug.len++]	= oper->sgl.off;
ic_debug.buf[ic_debug.len++]	= oper->sgl.addr;
#endif
	
		return 1;
	}
	//连续读取数据
	else if( oper->blk.type == 2 )
	{
	    /* the off and the len base on 16-bits */
	    len			= oper->blk.len + 1;
	    reg_num		= len >> 1;
		offset		= sp->buf.base + ((cache*seg_size) >> 1);
    	for( i = 0; i < reg_num; i++ )
    	{
    		tmp			=	_ic_read(offset + i);
	    		
    		*pdata++	= (ushort)tmp;
    		*pdata++	= (ushort)(tmp >> 16);
    	}
	    if( len & 1 )
	    {
	    	tmp			=	_ic_read( offset + i );
	    		
    		*pdata++	= (ushort)tmp;
	    }

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->buf.base + ((cache*seg_size) >> 1);
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->blk.type;
ic_debug.buf[ic_debug.len++]	= oper->blk.buf;
ic_debug.buf[ic_debug.len++]	= oper->blk.len;
ic_debug.buf[ic_debug.len++]	= oper->blk.addr;
#endif

		return len;
	}

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
#endif
	return 0;
}


int fpga_rm_rd_byte( ic_op_type opt, uint cache, uchar * pdata )
{
	ic_rd_op			* oper;
	const ic_seg_sp		* sp;
	int					seg_size;
	uint		        i, tmp, len, reg_num;
	uint				offset;

	if( (opt >= ic_rt_wr && opt != ic_rd_lp3) || cache >= ic_op_sp[opt].opr.size || pdata == NULL )
		return -2;

	/* get pointers */
	oper		= &ic_rd_icache[opt][cache];
	sp			= &ic_op_sp[opt];
	seg_size	= fpga_blk_size( opt );		/* base on 16-bits */
	
	/* organize the operator && get */
	if( oper->sgl.type == 1 )
	{
		/* the off bases on 16-bits */
		reg_num		= oper->sgl.off >> 1;
		tmp			= _ic_read( sp->buf.base + reg_num );
		if( oper->sgl.off & 1 )
		{
			*pdata++	= (uchar)(tmp >> 16 );
			*pdata++	= (uchar)(tmp >> 24 );
		}
		else
		{
			*pdata++	= (uchar)(tmp );
			*pdata++	= (uchar)(tmp >> 8 );
		}

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->buf.base + oper->sgl.off;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
ic_debug.buf[ic_debug.len++]	= oper->sgl.off;
ic_debug.buf[ic_debug.len++]	= oper->sgl.addr;
#endif
	
		return 2;
	}
	else if( oper->blk.type == 2 )
	{
	    /* the off and the len base on 16-bits */
	    len			= oper->blk.len + 1;
	    reg_num		= len >> 1;
		offset		= sp->buf.base + ((cache*seg_size) >> 1);
    	for( i = 0; i < reg_num; i++ )
    	{
    		tmp			=	_ic_read( offset + i );
	    		
    		*pdata++	= (uchar)(tmp );
    		*pdata++	= (uchar)(tmp >> 8);
    		*pdata++	= (uchar)(tmp >> 16);
    		*pdata++	= (uchar)(tmp >> 24);
    	}
	    if( len & 2 )
	    {
	    	tmp			=	_ic_read( offset + i );

    		*pdata++	= (uchar)(tmp );
    		*pdata++	= (uchar)(tmp >> 8);
	    }

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= sp->buf.base + ((cache*seg_size) >> 1);
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->blk.type;
ic_debug.buf[ic_debug.len++]	= oper->blk.buf;
ic_debug.buf[ic_debug.len++]	= oper->blk.len;
ic_debug.buf[ic_debug.len++]	= oper->blk.addr;
#endif

		return (len << 1);
	}

#ifdef FPGA_DEBUG
ic_debug.len					= 0;
ic_debug.buf[ic_debug.len++]	= oper->whole;
ic_debug.buf[ic_debug.len++]	= oper->sgl.type;
#endif
	return 0;
}
int time_cmp_usec( struct timeval * start, struct timeval * stop, uint usec_val )
{  
	uint  	 usec;

	if( stop->tv_sec < start->tv_sec )
		return 1;
		
	if( stop->tv_sec == start->tv_sec )
	{
		if( stop->tv_usec <= start->tv_usec )
		{
			return 0;
		}
	}

	usec	= (stop->tv_sec - start->tv_sec) * 1000 * 1000 + (stop->tv_usec - start->tv_usec);

	return (usec > usec_val);
}
void delay_usec( uint usec )
{
	struct timeval	start, stop;

	gettimeofday( &start, NULL );

	do
	{
		gettimeofday( &stop, NULL );
	}	while( !(time_cmp_usec(&start, &stop, usec)) );
}
int rt_read_fpga( ushort * pdata, int card_slot, uint reg_off, uint len )
{
	unsigned int		addr;
	unsigned int		offset;
	uint				i, num;

	assert( pdata != NULL && card_slot < 16 && reg_off < FPGA_CARD_SIZE && len < 256 );

#ifdef EXM01
	if( (0 == card_slot) || (15 == card_slot) ||  (16 == card_slot) || (7 == card_slot) || (8 == card_slot))
	{
		if( (0 == card_slot) || (15 == card_slot) )
		{
			addr = 0x8000 + reg_off;
		}
		else if( 16 == card_slot )
		{
			addr = 0x4000 + reg_off;
		}
		else if( 7 == card_slot )
		{
			addr = reg_off;
		}
		else if( 8 == card_slot )
		{
			addr = reg_off;
		}
			
		for( i = 0; i < len; ++i, pdata++, addr++)
		{
			*pdata = local_read( addr );								
		}			
	}
	else
#endif
	{
		addr		= FPGA_CARD_SIZE * card_slot + reg_off;
		num			= len >> 4;
		if( len & 0x0F )
		{
			offset	= len & 0x0F;
		}
		else
		{
			num--;
			offset	= 0x10;
		}

		for( i = 0; i < num; ++i )
		{
			fpga_rm_rd_set( ic_rd_rt, i, 0x10, 0, addr + i * 0x10 );
		}
		fpga_rm_rd_set( ic_rd_rt, i, offset, 0, addr + i * 0x10 );

		// dealy
		delay_usec( 40 * len );

		// read
		for( i = 0; i < num; ++i )
		{
			fpga_rm_rd( ic_rd_rt, i, pdata + i * 0x10 );
		}
		fpga_rm_rd( ic_rd_rt, i, pdata + i * 0x10 );
	}

	return 1;
}
//读取id和version
int lp_rd_get_id( int no, fpga_id_ver * id_ver_s, fpga_id_ver * id_ver_e )
{
	int		rc1, rc2;

	id_ver_s->whole	= 0;
	id_ver_e->whole	= 0;

	if( no == 0 )
	{
		rc1		= fpga_rm_rd( ic_rd_nrt, no, (ushort *)id_ver_s );
		rc2		= fpga_rm_rd( ic_rd_nrt, 63, (ushort *)id_ver_e );
		
		if( 0x0000 == id_ver_s->whole )
			id_ver_s->whole	= 0xFFFF;
		if( 0x0000 == id_ver_e->whole )
			id_ver_e->whole	= 0xFFFF;
	}
	else
	{
		rc1		= fpga_rm_rd( ic_rd_nrt, no, (ushort *)id_ver_s );
		rc2		= rc1;
		if( 0x0000 == id_ver_s->whole )
			id_ver_s->whole	= 0xFFFF;
		id_ver_e->whole	= id_ver_s->whole;
	}

	if( rc1 > 0 && rc2 > 0 && id_ver_s->whole == id_ver_e->whole )
	{
		return 1;
	}
printf("num %d\n", no);
printf("id = 0x%x\n", id_ver_s->bits.id);
printf("ver = 0x%x\n\n", id_ver_s->bits.ver);
	return 0;
}
/*
int fpga_wr_local( uint addr, uint *data, uint len )
{
    uint                 i;

	for( i = 0; i < len; i++ )
	{
	    local_write(addr + i, data[i]);
	}
	
	return 0;
}
*/


