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

#endif
