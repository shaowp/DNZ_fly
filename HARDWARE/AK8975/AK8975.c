#include "AK8975.h"

int16_t MagOffset[3];

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED 1
//***********AK8963 磁力计寄存器******
#define MAG_ID 0x00
#define MAG_ID_BUMBER 0x48
#define MAG_Status1 0x02
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08

#define MAG_Status2 0x09
#define MAG_Control 0x0A
#define MAG_Self_test 0x0C
#define MAG_Self_test 0x0C

#define MAG_ADDR 0x0C //IIC写入时的地址字节数据，+1为读取

int8_t AK8975_Init(void)
{
	uint8_t address = 0;

	address = MAG_Read_Byte(MAG_ID);
	if (address != 0x48) //检查ID
	{
		printf("AK8975 INIT ERROR\n");
		return 1; //返回 1
	}
	printf("AK8975 INIT SUCCESS\n");
	MAG_Write_Byte(0x0A, 0x11); //0x10 16位模式  0x 01 单次测量模式		//14位 - 0.6uT/LSB      16位 - 0.15uT/LSB
	delay_ms(10);				//延时等待磁力计可用
	return 0;					//成功为0，返回0
}

int8_t AK8975_Updata(void)
{
	short mx, my, mz;
	u8 buf[6], res;
	res = MAG_Read_Len(MAG_ADDR, 0x03, 6, buf);
	if (res == 0)
	{
		mx = (((u16)buf[0] << 8) | buf[1]);
		mz = (((u16)buf[4] << 8) | buf[5]);
		my = (((u16)buf[2] << 8) | buf[3]);
	}
	printf("%d\t%d\t%d\n", mx, my, mz);

	MAG_Write_Byte(0x0A, 0x11); //0x10 16位模式  0x 01 单次测量模式		//14位 - 0.6uT/LSB      16位 - 0.15uT/LSB

	return 0;
}

//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MAG_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
	if (SOFT_IIC_Wait_Ack())			 //等待应答
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg); //写寄存器地址
	SOFT_IIC_Wait_Ack();	 //等待应答
	for (i = 0; i < len; i++)
	{
		SOFT_IIC_Send_Byte(buf[i]); //发送数据
		if (SOFT_IIC_Wait_Ack())	//等待ACK
		{
			SOFT_IIC_Stop();
			return 1;
		}
	}
	SOFT_IIC_Stop();
	return 0;
}

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MAG_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 0); //发送器件地址+写命令
	if (SOFT_IIC_Wait_Ack())			 //等待应答
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg); //写寄存器地址
	SOFT_IIC_Wait_Ack();	 //等待应答
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 1); //发送器件地址+读命令
	SOFT_IIC_Wait_Ack();				 //等待应答
	while (len)
	{
		if (len == 1)
			*buf = SOFT_IIC_Read_Byte(0); //读数据,发送nACK
		else
			*buf = SOFT_IIC_Read_Byte(1); //读数据,发送ACK
		len--;
		buf++;
	}
	SOFT_IIC_Stop(); //产生一个停止条件
	return 0;
}
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MAG_Write_Byte(u8 reg, u8 data)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 0); //发送器件地址+写命令
	if (SOFT_IIC_Wait_Ack())				 //等待应答
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg);  //写寄存器地址
	SOFT_IIC_Wait_Ack();	  //等待应答
	SOFT_IIC_Send_Byte(data); //发送数据
	if (SOFT_IIC_Wait_Ack())  //等待ACK
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Stop();
	return 0;
}

//IIC读一个字节
//reg:寄存器地址
//返回值:读到的数据
u8 MAG_Read_Byte(u8 reg)
{
	u8 res;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 0); //发送器件地址+写命令
	SOFT_IIC_Wait_Ack();					 //等待应答
	SOFT_IIC_Send_Byte(reg);				 //写寄存器地址
	SOFT_IIC_Wait_Ack();					 //等待应答
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 1); //发送器件地址+读命令
	SOFT_IIC_Wait_Ack();					 //等待应答
	res = SOFT_IIC_Read_Byte(0);			 //读取数据,发送nACK
	SOFT_IIC_Stop();						 //产生一个停止条件
	return res;
}
