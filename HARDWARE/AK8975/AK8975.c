#include "AK8975.h"
#include "data_process.h"
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
	MAG_Write_Byte(0x0A, 0x11); //0x10 16位模式  0x01 单次测量模式		//14位 - 0.6uT/LSB      16位 - 0.15uT/LSB

	AK8975_MAG.mx_offset = 0.0;
	AK8975_MAG.my_offset = 0.0;
	AK8975_MAG.mz_offset = 0.0;
	AK8975_MAG.x_gain = 1.0;
	AK8975_MAG.y_gain = 1.0;
	AK8975_MAG.z_gain = 1.0;

	// B[0]	0.975454084181572
	// B[1]	0.00806000616669667
	// B[2]	-0.0119091414985016
	// B[3]	0.985901401190494
	// B[4]	0.00165710771775011
	// B[5]	1.04004240346894
	// MagOffset.x	0.275001639772076
	// MagOffset.y	-0.28155495239433
	// MagOffset.z	-0.233880795500522

	AK8975_MAG.G_mx_offset = 0.275001639772076;
	AK8975_MAG.G_my_offset = -0.28155495239433;
	AK8975_MAG.G_mz_offset = -0.233880795500522;

	AK8975_MAG.B0 = 0.975454084181572;
	AK8975_MAG.B1 = 0.00806000616669667;
	AK8975_MAG.B2 = -0.0119091414985016;
	AK8975_MAG.B3 = 0.985901401190494;
	AK8975_MAG.B4 = 0.00165710771775011;
	AK8975_MAG.B5 = 1.04004240346894;
	delay_ms(10); //延时等待磁力计可用
	return 0;	 //成功为0，返回0
}

int8_t AK8975_Updata(void)
{
	float G_mx, G_my, G_mz;
	short temp_mx, temp_my, temp_mz;
	u8 buf[6], res;
	res = MAG_Read_Len(MAG_ADDR, 0x03, 6, buf);
	if (res == 0)
	{
		temp_mx = (((u16)buf[1] << 8) | buf[0]);
		temp_my = (((u16)buf[3] << 8) | buf[2]);
		temp_mz = (((u16)buf[5] << 8) | buf[4]);
		// printf("%d\t %d\t %d\t", temp_mx, temp_my, temp_mz);
	}
	//高斯数据的校准
	//原来的数据的单位是16位的，量程为±1229 ?T即 2458*10^-6 T =24.58G
	//原始数据是13位，对应的量程为±1229，即原始数据需要 * 8192/2458 =xx ?T
	//8192/24.58=333

	//校准的时候需要
	// G_mx = temp_mx / 341.0;
	// G_my = temp_my / 341.0;
	// G_mz = temp_mz / 341.0;
	// printf("%f\t %f\t %f\n", G_mx, G_my, G_mz);

	// //校准之后需要使用
	G_mx = temp_mx / 333.0 - AK8975_MAG.G_mx_offset;
	G_my = temp_my / 333.0 - AK8975_MAG.G_my_offset;
	G_mz = temp_mz / 333.0 - AK8975_MAG.G_mz_offset;

	// printf("%.2f\t %.2f\t %.2f\n", G_mx, G_my, G_mz);
	AK8975_MAG.G_mx = AK8975_MAG.B0 * G_mx + AK8975_MAG.B1 * G_my + AK8975_MAG.B2 * G_mz;
	AK8975_MAG.G_my = AK8975_MAG.B1 * G_mx + AK8975_MAG.B3 * G_my + AK8975_MAG.B4 * G_mz;
	AK8975_MAG.G_mz = AK8975_MAG.B2 * G_mx + AK8975_MAG.B4 * G_my + AK8975_MAG.B5 * G_mz;

	//数据放到磁力计主数据
	AK8975_MAG.mx = -AK8975_MAG.G_mx * 341.0; //特别注意方向，方向非常重要
	AK8975_MAG.my = AK8975_MAG.G_my * 341.0;
	AK8975_MAG.mz = AK8975_MAG.G_mz * 341.0;

	MAG_Write_Byte(0x0A, 0x11); //0x10 16位模式  0x 01 单次测量模式		//14位 - 0.6uT/LSB      16位 - 0.15uT/LSB
	return 0;
}

//磁力计校准
void Mag_Calibartion(void)
{
	//	float G_mx, G_my, G_mz;
	//	short temp_mx, temp_my, temp_mz;
	//	u8 buf[6], res;
	//第一次校准。板子水平放置，然后旋转
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	while (1)
	{
		AK8975_Updata();
		delay_ms(100);
	}

	// short x_max = -32767;
	// short x_min = 32767;
	// short y_max = -32767;
	// short y_min = 32767;
	// short z_max = -32767;
	// short z_min = 32767;
	// // short x_max = 0;
	// // short x_min = 0;
	// // short y_max = 0;
	// // short y_min = 0;
	// // short z_max = 0;
	// // short z_min = 0;
	// short cal_cnt = 0; //需要校准的话改成2000或者更大

	// //第一次校准。板子水平放置，然后旋转
	// printf("first cal MAG\n");
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// for (cal_cnt = 0; cal_cnt <= 2000; cal_cnt++)
	// {
	// 	AK8975_Updata();
	// 	// MAG_IMU_Filter();
	// 	// printf("mx %.2f\tmy %.2f\tmz %.2f\n", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
	// 	if (AK8975_MAG.mx >= x_max)
	// 		x_max = AK8975_MAG.mx;
	// 	if (AK8975_MAG.my >= y_max)
	// 		y_max = AK8975_MAG.my;
	// 	if (AK8975_MAG.mz >= z_max)
	// 		z_max = AK8975_MAG.mz;
	// 	if (AK8975_MAG.mx <= x_min)
	// 		x_min = AK8975_MAG.mx;
	// 	if (AK8975_MAG.my <= y_min)
	// 		y_min = AK8975_MAG.my;
	// 	if (AK8975_MAG.mz <= z_min)
	// 		z_min = AK8975_MAG.mz;
	// 	printf(". ");
	// 	delay_ms(10);
	// }
	// //第二次校准，机头朝上
	// ///我猜的，我也不知道哪个朝上
	// printf("second cal MAG\n");
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// for (cal_cnt = 0; cal_cnt <= 2000; cal_cnt++)
	// {

	// 	AK8975_Updata();
	// 	// MAG_IMU_Filter();
	// 	// printf("mx %.2f\tmy %.2f\tmz %.2f\n", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
	// 	if (AK8975_MAG.mx >= x_max)
	// 		x_max = AK8975_MAG.mx;
	// 	if (AK8975_MAG.my >= y_max)
	// 		y_max = AK8975_MAG.my;
	// 	if (AK8975_MAG.mz >= z_max)
	// 		z_max = AK8975_MAG.mz;
	// 	if (AK8975_MAG.mx <= x_min)
	// 		x_min = AK8975_MAG.mx;
	// 	if (AK8975_MAG.my <= y_min)
	// 		y_min = AK8975_MAG.my;
	// 	if (AK8975_MAG.mz <= z_min)
	// 		z_min = AK8975_MAG.mz;
	// 	printf(". ");
	// 	delay_ms(10);
	// }
	// printf("xmax:%d\tymax:%d\tzmax:%d\t", x_max, y_max, z_max);
	// printf("xmin:%d\tymin:%d\tzmin:%d\n", x_min, y_min, z_min);
	// AK8975_MAG.mx_offset = (x_min + x_max) / 2.0;
	// AK8975_MAG.my_offset = (y_min + y_max) / 2.0;
	// AK8975_MAG.mz_offset = (z_min + z_max) / 2.0;
	// AK8975_MAG.y_gain = 1.0 * (x_max - x_min) / (y_max - y_min);
	// AK8975_MAG.z_gain = 1.0 * (x_max - x_min) / (z_max - z_min);
	// printf("xo%.2f\t yo%.2f\t zo%.2f\t %.2f\t %.2f\n", AK8975_MAG.mx_offset, AK8975_MAG.my_offset, AK8975_MAG.mz_offset, AK8975_MAG.y_gain, AK8975_MAG.z_gain);
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

////////////////////////////////////
//////////////变量区////////////////
Struct_AK8975_MAG AK8975_MAG;

////////////////////////////////////
