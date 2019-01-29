#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h>	/* for NAME_MAX */
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <strings.h>	/* for strcasecmp() */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


#define I2C_DEV              "/dev/i2c-1"
#define DEV_ADDR             0x50

#define BLOCK_MAX            8
#define I2C_DEV_FIFO_LEN     8
#define DEV_ADDR             0x50

enum i2c_opt
{
	I2C_RD_DATA = 0,
	I2C_WR_DATA,
	I2C_WR_FILE,
};

int i2c_dev_open(int bus)
{
	char dev_name[20];
	int i2c_fd;
	
	snprintf(dev_name, 19, "/dev/i2c-%d", bus);
	i2c_fd = open(dev_name, O_RDWR);
	if(i2c_fd < 0)
	{
		perror("can't open dev\n");
		return -1;
	}
	
	return i2c_fd;
}

void i2c_dev_close(int i2c_fd)
{
	close(i2c_fd);
}

int i2c_bytes_write(int fd, unsigned char addr, unsigned short reg, unsigned char *val, int len)
{
	struct i2c_rdwr_ioctl_data rdwr;
	struct i2c_msg msg;
	int    ret = 0;
	int    i;

	rdwr.nmsgs = 1;
	rdwr.msgs = &msg;

	//发送要读取的设备地址
	msg.addr = addr;
	msg.flags = 0;         //write
	msg.len = len + 2;     //数据长度
	
	//发送数据
	msg.buf = (unsigned char *)malloc(msg.len);
	if (NULL == msg.buf)
	{
		printf("get memory error \n");
		free(msg.buf);
		return -1;
	}

	msg.buf[0] = (unsigned char)(reg >> 8);
	msg.buf[1] = (unsigned char)(reg);
	for (i = 2; i < msg.len; i++)
	{
		msg.buf[i] = val[i - 2];
	}

	ret = ioctl(fd, I2C_RDWR, &rdwr);	//读出来
	usleep(1000);
	if (ret < 0)
	{
		printf("  write nbyte error!\n");
		return -1;
	}

	return 0;
}

int i2c_bytes_read(int fd, unsigned char addr, unsigned short reg, unsigned char *val, int len)
{
	unsigned char outbuf[2];
	struct i2c_rdwr_ioctl_data rdwr;
	struct i2c_msg msg[2];
	int ret = 0;
	
	/* 数据帧类型有2种
	* 写要发送起始信号，进行写寄存器操作，再发送起始信号,进行读操作,
	* 有2个起始信号，因此需要分开来操作。
	*/
	outbuf[0] = (unsigned char)(reg >> 8);
	outbuf[1] = (unsigned char)(reg);
	rdwr.nmsgs = 2;
	
	//发送要读取的寄存器地址
	msg[0].addr  = addr;
	msg[0].flags = 0;              //write
	msg[0].len   = 2;              //数据长度
	msg[0].buf   = outbuf;         //发送寄存器地址

	//读取数据
	msg[1].len = len;              //读取数据长度
	msg[1].addr = addr;            //设备地址
	msg[1].flags = I2C_M_RD;       //read
	msg[1].buf = val;

	rdwr.msgs = msg;
	ret = ioctl(fd, I2C_RDWR, &rdwr);		//发送i2c,进行读取操作 
	//usleep(10);
	if (ret < 0)
	{
		perror("  read nbyte error! \n");
		return -1;
	}

	return 0;
}



int main(int argc, char *argv[])
{
	char     filename[64];	
	char    *wr_ptr = NULL;
	int      file   = -1;
	int      device = -1;
	int      i2cbus = 0;
	int      rd_len = 0;
	int      len = 0;
	int      i   = 0;
	unsigned int file_len;
	struct stat buf;	
	enum i2c_opt  flags;
	unsigned char   data[BLOCK_MAX];	
	unsigned char   address = 0;
	unsigned int    reg = 0;

	printf("./i2c R/W/DPLL I2C_BUS DEV_ADDR REG LEN/DATA/FILE \n");
	
	if(argc < 6)
	{
		printf("param error \n");
		return -1;
	}
	
	wr_ptr  = argv[1];
	i2cbus  = strtoul(argv[2], NULL, 0);
	address = strtoul(argv[3], NULL, 0);
	reg     = strtoul(argv[4], NULL, 0);
	//snprintf(filename, 19, "/dev/i2c-%d", i2cbus);
	//filename[20] = '\0';
	printf("i2c bus number  = %d \n", i2cbus);
	printf("device address  = 0x%x \n", address);

	device = i2c_dev_open(i2cbus);
	if(device < 0)
	{
		printf("open device file error, %d \n", device);
		return -1;
	}

	if(strncmp(wr_ptr, "r", 1) == 0)
	{
		flags = I2C_RD_DATA;
		len = strtoul(argv[5], NULL, 0);
	}
	else if(strncmp(wr_ptr, "w", 1) == 0)
	{
		unsigned char *cp;
		unsigned char tmp;
		
		flags = I2C_WR_DATA;
		
		cp = argv[5];
		len = strlen(argv[5]);
		
		if (len % 2 != 0) 
		{
			printf("error: unaligned byte:hex\n");

			return 0;
		} 
		else 
		{
			for(i = 0; i < len; i++, cp++) 
			{
				tmp = *cp - '0';
				if(tmp > 9)
					tmp -= ('A' - '0') - 10;
				if(tmp > 15)
					tmp -= ('a' - 'A');
				if(tmp > 15) {
					printf("Hex conversion error on %c, mark as 0.\n", *cp);
					tmp = 0;
				}
				if((i % 2) == 0)
					data[i / 2] = (tmp << 4);
				else
					data[i / 2] |= tmp;
			}	
		}		
		len >>= 1;
		
		printf("write %d bytes at 0x%04x: \n", len, (unsigned short)reg);
		for (i = 0 ; i < len ; i++)
			printf(" %02x",data[i]);
		printf(" \n");	
	}
	else
	{	
		flags = I2C_WR_FILE;
		snprintf(filename, 19, "%s", argv[5]);
		filename[64] = '\0';
		file = open(filename, O_RDWR);
		if(file < 0)
		{
			printf("open file %s error \n", filename);
			return -1;
		}
	}

	switch(flags)
	{
		case I2C_RD_DATA:
			//reg = 0;
			while(len > 0)
			{
				if(len > I2C_DEV_FIFO_LEN)
				{
					rd_len = I2C_DEV_FIFO_LEN;
				}
				else
				{
					rd_len = len;
				}
				
				memset(data, 0, sizeof(data));
				i2c_bytes_read(device, address, reg, data, rd_len);
				
				for(i = 0; i < rd_len; ++i)
					printf("0x%-2x ", data[i]);		
				
				len -= rd_len;
				reg += rd_len;				
			}			
			printf("\n");
			
			break;
		case I2C_WR_DATA:
			//reg = 0;
			while(len > 0)
			{
				int wr_len = 0;
				
				if(len > I2C_DEV_FIFO_LEN)
				{
					wr_len = I2C_DEV_FIFO_LEN;
				}
				else
				{
					wr_len = len;
				}
				i2c_bytes_write(device, address, reg, data, wr_len);
				len -= wr_len;
				reg += wr_len;
			}
			
			break;
		case I2C_WR_FILE:

			
			fstat(file, &buf);
			file_len = (unsigned int)(buf.st_size);
			printf("file_len = 0x%x \r\n", file_len);
			reg = 0;
			printf("\nPercent:    %3d%% \r\n", (int)(reg / file_len * 100));
			while(rd_len = read(file, data, I2C_DEV_FIFO_LEN))
			{	
		#if 0
				for(i = 0; i < rd_len; ++i)
					printf("0x%-2x ", data[i]);		
				printf("\n");
		#endif
				address = address | (reg & 0x10000 ? 0x04 : 0);

				i2c_bytes_write(device, address, reg, data, rd_len);
				reg += rd_len;
				
				if(reg % 0x800 == 0)
					printf("\033[1A""\033[K""Percent:    %3d%% \r\n", (int)((float)reg / file_len * 100));
			}
		
			break;
		default:
			printf("error i2c option \n");
			break;
	}
	
	return 0;
}






