#include "mpu6050.h"


#define NEED_CAL_GYRO
#define NEED_CAL_ACC

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //��λMPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //����MPU6050
	MPU_Set_Gyro_Fsr(3);					 //�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					 //���ٶȴ�����,��2g
	MPU_Set_Rate(50);						 //���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); //I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); //INT���ŵ͵�ƽ��Ч
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) //����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //���ٶ��������Ƕ�����
		MPU_Set_Rate(300);						 //���ò�����Ϊ600Hz
		printf("MPU INIT SUCCESS\n");
	}
	else
	{
		printf("MPU INIT ERROR\n");
		return 1;
	}

	//���ñ�����offset
	MPU_GYRO.gyrox_offset = -112;
	MPU_GYRO.gyroy_offset = 101;
	MPU_GYRO.gyroz_offset = 4;

	MPU_ACC.accx_offset = 0;
	MPU_ACC.accy_offset = 0;
	MPU_ACC.accz_offset = 0;

	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //���������������̷�Χ
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //���ü��ٶȴ����������̷�Χ
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)
		data = 1;
	else if (lpf >= 98)
		data = 2;
	else if (lpf >= 42)
		data = 3;
	else if (lpf >= 20)
		data = 4;
	else if (lpf >= 10)
		data = 5;
	else
		data = 6;
	return MPU_Write_Byte(MPU_CFG_REG, data); //�������ֵ�ͨ�˲���
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ��
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //�������ֵ�ͨ�˲���
	return MPU_Set_LPF(rate / 2);					  //�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 36.53 + ((double)raw) / 340;
	return temp * 100;
	;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	short temp_gx, temp_gy, temp_gz;
	res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		temp_gx = ((u16)buf[0] << 8) | buf[1];
		temp_gy = ((u16)buf[2] << 8) | buf[3];
		temp_gz = ((u16)buf[4] << 8) | buf[5];
	}
	MPU_GYRO.gyrox = temp_gx - MPU_GYRO.gyrox_offset;
	MPU_GYRO.gyroy = temp_gy - MPU_GYRO.gyroy_offset;
	MPU_GYRO.gyroz = temp_gz - MPU_GYRO.gyroz_offset;
	return res;
}
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	short temp_ax, temp_ay, temp_az;
	res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		temp_ax = ((u16)buf[0] << 8) | buf[1];
		temp_ay = ((u16)buf[2] << 8) | buf[3];
		temp_az = ((u16)buf[4] << 8) | buf[5];
	}
	MPU_ACC.accx = temp_ax - MPU_ACC.accx_offset;
	MPU_ACC.accy = temp_ay - MPU_ACC.accy_offset;
	MPU_ACC.accz = temp_az - MPU_ACC.accz_offset;
	return res;
}
//IIC����д
//addr:������ַ
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
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
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
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
u8 MPU_Write_Byte(u8 reg, u8 data)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����
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
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 0); //����������ַ+д����
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	SOFT_IIC_Send_Byte(reg);				 //д�Ĵ�����ַ
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 1); //����������ַ+������
	SOFT_IIC_Wait_Ack();					 //�ȴ�Ӧ��
	res = SOFT_IIC_Read_Byte(0);			 //��ȡ����,����nACK
	SOFT_IIC_Stop();						 //����һ��ֹͣ����
	return res;
}

void Gyro_Calibartion(void)
{
	uint8_t k = 30;
	int32_t g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;
	short temp_gx, temp_gy, temp_gz;
	u8 buf[6], res, i;
	//��Ҫ���жϷɿ��Ƿ��ֹ
	while (k--)
	{
		do
		{
			delay_ms(10);
			res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
			if (res == 0)
			{
				temp_gx = ((u16)buf[0] << 8) | buf[1];
				temp_gy = ((u16)buf[2] << 8) | buf[3];
				temp_gz = ((u16)buf[4] << 8) | buf[5];
			}
			ErrorGyro[0] = temp_gx - LastGyro[0];
			ErrorGyro[1] = temp_gy - LastGyro[1];
			ErrorGyro[2] = temp_gz - LastGyro[2];
			LastGyro[0] = temp_gx;
			LastGyro[1] = temp_gy;
			LastGyro[2] = temp_gz;
		} while ((ErrorGyro[0] > MAX_GYRO_QUIET) || (ErrorGyro[0] < MIN_GYRO_QUIET) || (ErrorGyro[1] > MAX_GYRO_QUIET) || (ErrorGyro[1] < MIN_GYRO_QUIET) || (ErrorGyro[2] > MAX_GYRO_QUIET) || (ErrorGyro[2] < MIN_GYRO_QUIET));
	}
	for (i = 0; i < 100; i++) //��������100�Σ�һ����ʱ100*3=300ms
	{
		delay_ms(5);
		res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
		if (res == 0)
		{
			temp_gx = ((u16)buf[0] << 8) | buf[1];
			temp_gy = ((u16)buf[2] << 8) | buf[3];
			temp_gz = ((u16)buf[4] << 8) | buf[5];
		}
		g_Gyro_xoffset += temp_gx;
		g_Gyro_yoffset += temp_gy;
		g_Gyro_zoffset += temp_gz;
	}
	MPU_GYRO.gyrox_offset = g_Gyro_xoffset / 100; //�õ���̬ƫ��ֵ//�ɼ���100����ƽ��
	MPU_GYRO.gyroy_offset = g_Gyro_yoffset / 100;
	MPU_GYRO.gyroz_offset = g_Gyro_zoffset / 100;
	printf("gxoff:%d\tgyoff:%d\tgzoff:%d\n", MPU_GYRO.gyrox_offset, MPU_GYRO.gyroy_offset, MPU_GYRO.gyroz_offset);
}
void Acc_Calibartion(void)
{
}

////////////////////////////////////
//////////////������////////////////

//������ٶȣ������ǵ�ȫ�ֱ���
Struct_MPU_ACC MPU_ACC;
Struct_MPU_GYRO MPU_GYRO;

////////////////////////////////////
