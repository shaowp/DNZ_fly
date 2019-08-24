#ifndef _MAGNETIC_H
#define _MAGNETIC_H

#include "usart.h"
#include "softiic.h"
#include "sys.h"
#include "delay.h"

u8 MAG_Write_Byte(u8 reg, u8 data);
u8 MAG_Read_Byte(u8 reg);
u8 MAG_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf);
u8 MAG_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf);
void MAG_Init(void);
u8 MAG_Get_data(short *gx, short *gy, short *gz);

extern int8_t AK8975_Init(void);
extern int8_t AK8975_Updata(void);
extern void Mag_Calibartion(void);

////////////////////////////////////
//////////////变量区////////////////

typedef struct ak8975_mag_data
{
	//高斯量
	float G_mx;
	float G_my;
	float G_mz;
	float mx;
	float my;
	float mz;
	float mx_offset;
	float my_offset;
	float mz_offset;
	float y_gain;
	float z_gain;
	float x_gain;
	//磁力计修正数据
	float B0;
	float B1;
	float B2;
	float B3;
	float B4;
	float B5;
	//磁力计高斯偏置
	float G_mx_offset;
	float G_my_offset;
	float G_mz_offset;
} Struct_AK8975_MAG;

extern Struct_AK8975_MAG AK8975_MAG;
////////////////////////////////////

#endif
