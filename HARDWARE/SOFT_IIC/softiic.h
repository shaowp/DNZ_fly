#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������V3
//MPU6050 IIC���� ����
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/17
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////

//IO��������
#define SOFT_SDA_IN()             \
	{                             \
		GPIOB->CRH &= 0XFFFF0FFF; \
		GPIOB->CRH |= 8 << 12;    \
	}
#define SOFT_SDA_OUT()            \
	{                             \
		GPIOB->CRH &= 0XFFFF0FFF; \
		GPIOB->CRH |= 3 << 12;    \
	}

//IO��������
#define SOFT_IIC_SCL PBout(10) //SCL
#define SOFT_IIC_SDA PBout(11) //SDA
#define SOFT_READ_SDA PBin(11) //����SDA

//IIC���в�������
void SOFT_IIC_Delay(void);				  //MPU IIC��ʱ����
void SOFT_IIC_Init(void);				  //��ʼ��IIC��IO��
void SOFT_IIC_Start(void);				  //����IIC��ʼ�ź�
void SOFT_IIC_Stop(void);				  //����IICֹͣ�ź�
void SOFT_IIC_Send_Byte(u8 txd);		  //IIC����һ���ֽ�
u8 SOFT_IIC_Read_Byte(unsigned char ack); //IIC��ȡһ���ֽ�
u8 SOFT_IIC_Wait_Ack(void);				  //IIC�ȴ�ACK�ź�
void SOFT_IIC_Ack(void);				  //IIC����ACK�ź�
void SOFT_IIC_NAck(void);				  //IIC������ACK�ź�

#endif
