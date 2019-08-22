#include "AK8975.h"

int16_t MagOffset[3];

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED 1
//***********AK8963 �����ƼĴ���******
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

#define MAG_ADDR 0x0C //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

int8_t AK8975_Init(void)
{
	uint8_t address = 0;

	address = MAG_Read_Byte(MAG_ID);
	if (address != 0x48) //���ID
	{
		printf("AK8975 INIT ERROR\n");
		return 1; //���� 1
	}
	printf("AK8975 INIT SUCCESS\n");
	MAG_Write_Byte(0x0A, 0x11); //0x10 16λģʽ  0x 01 ���β���ģʽ		//14λ - 0.6uT/LSB      16λ - 0.15uT/LSB
	delay_ms(10);				//��ʱ�ȴ������ƿ���
	return 0;					//�ɹ�Ϊ0������0
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

	MAG_Write_Byte(0x0A, 0x11); //0x10 16λģʽ  0x 01 ���β���ģʽ		//14λ - 0.6uT/LSB      16λ - 0.15uT/LSB

	return 0;
}

//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MAG_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	u8 i;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (SOFT_IIC_Wait_Ack())			 //�ȴ�Ӧ��
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	SOFT_IIC_Wait_Ack();	 //�ȴ�Ӧ��
	for (i = 0; i < len; i++)
	{
		SOFT_IIC_Send_Byte(buf[i]); //��������
		if (SOFT_IIC_Wait_Ack())	//�ȴ�ACK
		{
			SOFT_IIC_Stop();
			return 1;
		}
	}
	SOFT_IIC_Stop();
	return 0;
}

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MAG_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 0); //����������ַ+д����
	if (SOFT_IIC_Wait_Ack())			 //�ȴ�Ӧ��
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg); //д�Ĵ�����ַ
	SOFT_IIC_Wait_Ack();	 //�ȴ�Ӧ��
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((addr << 1) | 1); //����������ַ+������
	SOFT_IIC_Wait_Ack();				 //�ȴ�Ӧ��
	while (len)
	{
		if (len == 1)
			*buf = SOFT_IIC_Read_Byte(0); //������,����nACK
		else
			*buf = SOFT_IIC_Read_Byte(1); //������,����ACK
		len--;
		buf++;
	}
	SOFT_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}
//IICдһ���ֽ�
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MAG_Write_Byte(u8 reg, u8 data)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 0); //����������ַ+д����
	if (SOFT_IIC_Wait_Ack())				 //�ȴ�Ӧ��
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Send_Byte(reg);  //д�Ĵ�����ַ
	SOFT_IIC_Wait_Ack();	  //�ȴ�Ӧ��
	SOFT_IIC_Send_Byte(data); //��������
	if (SOFT_IIC_Wait_Ack())  //�ȴ�ACK
	{
		SOFT_IIC_Stop();
		return 1;
	}
	SOFT_IIC_Stop();
	return 0;
}

//IIC��һ���ֽ�
//reg:�Ĵ�����ַ
//����ֵ:����������
u8 MAG_Read_Byte(u8 reg)
{
	u8 res;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 0); //����������ַ+д����
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	SOFT_IIC_Send_Byte(reg);				 //д�Ĵ�����ַ
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MAG_ADDR << 1) | 1); //����������ַ+������
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	res = SOFT_IIC_Read_Byte(0);			 //��ȡ����,����nACK
	SOFT_IIC_Stop();						 //����һ��ֹͣ����
	return res;
}
