#include "mpu6050.h"
#include "data_process.h"
#define NEED_CAL_GYRO
#define NEED_CAL_ACC
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED 1

#define AcceMax_1G 16384
#define GRAVITY_MSS 9.80665f
#define ACCEL_TO_1G GRAVITY_MSS / AcceMax_1G
#define One_G_TO_Accel AcceMax_1G / GRAVITY_MSS
#define One_G_TO_Accel AcceMax_1G / GRAVITY_MSS

// b00.00	b10.00	b20.00	k01.00	k11.00	k21.00
// b00.00	b110.00	b220.00	k01.00	k111.00	k221.00
//b00.24	b1-0.30	b20.21	k00.99	k11.00	k20.97
// b00.26	b1-0.30	b20.21	k00.99	k11.00	k20.97
float K[3] = {0.99, 11.0, 20.97}; //Ĭ�ϱ�����
float B[3] = {0.26, -0.3, 20.21};   //Ĭ����λ���

//��ʼ��MPU6050
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Init(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //��λMPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //����MPU6050
	MPU_Set_Gyro_Fsr(3);					 //�����Ǵ�����,��1000dps///����Ϊ3�Ļ�Ϊ��2000
	MPU_Set_Accel_Fsr(0);					 //���ٶȴ�����,��2g
	MPU_Set_Rate(300);						 //���ò�����300Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); //I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); //INT���ŵ͵�ƽ��Ч
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) //����ID��ȷ
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //���ٶ��������Ƕ�����
		MPU_Set_Rate(300);						 //���ò�����Ϊ300Hz
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

	// MPU_ACC.accx = K[0] * temp_ax - B[0] * One_G_TO_Accel; //����У׼�ó���K��Bֵ�������õ�
	// MPU_ACC.accy = K[1] * temp_ay - B[1] * One_G_TO_Accel;
	// MPU_ACC.accz = K[2] * temp_az - B[2] * One_G_TO_Accel;
	MPU_ACC.accx = temp_ax;
	MPU_ACC.accy = temp_ay;
	MPU_ACC.accz = temp_az;
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

//���ٶȼ�����У׼
void Acc_Calibartion(void)
{
}

/**************************************************************************/
/***************************APMУ׼���ٶȼ�*******************************/
/*��һ��ɿ�ƽ�ţ�Z�����������Ϸ������洹ֱ���ϣ���Z axis is about 1g,X��Y is about 0g*/
/*�ڶ���ɿ�ƽ�ţ�X�����������Ϸ�����ͷ��ֱ���ϣ���X axis is about 1g,Y��Z is about 0g*/
/*������ɿ�ƽ�ţ�X�����������·�����ഹֱ���ϣ���X axis is about -1g,Y��Z is about 0g*/
/*������ɿ�ƽ�ţ�Y�����������·����Ҳഹֱ���ϣ���Y axis is about -1g,X��Z is about 0g*/
/*������ɿ�ƽ�ţ�Y�����������Ϸ�����β��ֱ���ϣ���Y axis is about 1g,X��Z is about 0g*/
/*������ɿ�ƽ�ţ�Z�����������·������洹ֱ���ϣ���Z axis is about -1g,X��Y is about 0g*/

/*
1. 	ƽ
2. 	��ͷ����
3.	��β����
4. 	usb�ڳ���
5. 	usb�ڳ���
6. 	��
*/
typedef struct
{
	float x;
	float y;
	float z;
} Acce_Unit;

void delay_x_ms(u8 time)
{
	u8 i = 0;
	for (i = 0; i < 8; i++)
	{
		delay_ms(1000);
	}
}

void get_yuanshi_acc(void) //��ȡԭʼACC,�����˲�
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
	MPU_ACC.accx = temp_ax;
	MPU_ACC.accy = temp_ay;
	MPU_ACC.accz = temp_az;
	// ACC_IMU_Filter();
	// printf("x:%d\ty:%d\tz:%d\n", MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz);
	delay_ms(10);
}

void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
	int16_t j, k;
	for (j = 0; j < 6; j++)
	{
		dS[j] = 0.0f;
		for (k = 0; k < 6; k++)
		{
			JS[j][k] = 0.0f;
		}
	}
}

void Calibrate_Find_Delta(float dS[6], float JS[6][6], float delta[6])
{
	//Solve 6-d matrix equation JS*x = dS
	//first put in upper triangular form
	int16_t i, j, k;
	float mu;
	//make upper triangular
	for (i = 0; i < 6; i++)
	{
		//eliminate all nonzero entries below JS[i][i]
		for (j = i + 1; j < 6; j++)
		{
			mu = JS[i][j] / JS[i][i];
			if (mu != 0.0f)
			{
				dS[j] -= mu * dS[i];
				for (k = j; k < 6; k++)
				{
					JS[k][j] -= mu * JS[k][i];
				}
			}
		}
	}
	//back-substitute
	for (i = 5; i >= 0; i--)
	{
		dS[i] /= JS[i][i];
		JS[i][i] = 1.0f;

		for (j = 0; j < i; j++)
		{
			mu = JS[i][j];
			dS[j] -= mu * dS[i];
			JS[i][j] = 0.0f;
		}
	}
	for (i = 0; i < 6; i++)
	{
		delta[i] = dS[i];
	}
}

void Calibrate_Update_Matrices(float dS[6],
							   float JS[6][6],
							   float beta[6],
							   float data[3])
{
	int16_t j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];
	for (j = 0; j < 3; j++)
	{
		b = beta[3 + j];
		dx = (float)data[j] - beta[j];
		residual -= b * b * dx * dx;
		jacobian[j] = 2.0f * b * b * dx;
		jacobian[3 + j] = -2.0f * b * dx * dx;
	}

	for (j = 0; j < 6; j++)
	{
		dS[j] += jacobian[j] * residual;
		for (k = 0; k < 6; k++)
		{
			JS[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

u8 Calibrate_accel(Acce_Unit accel_sample[6], Acce_Unit *accel_offsets, Acce_Unit *accel_scale)
{
	int16_t i;
	int16_t num_iterations = 0;
	float eps = 0.000000001;
	float change = 100.0;
	float data[3] = {0};
	float beta[6] = {0};
	float delta[6] = {0};
	float ds[6] = {0};
	float JS[6][6] = {0};
	unsigned char temp = SUCCESS;
	// reset
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1.0f / GRAVITY_MSS;
	while (num_iterations < 20 && change > eps)
	{
		num_iterations++;
		Calibrate_Reset_Matrices(ds, JS);

		for (i = 0; i < 6; i++)
		{
			data[0] = accel_sample[i].x;
			data[1] = accel_sample[i].y;
			data[2] = accel_sample[i].z;
			Calibrate_Update_Matrices(ds, JS, beta, data);
		}
		Calibrate_Find_Delta(ds, JS, delta);
		change = delta[0] * delta[0] +
				 delta[0] * delta[0] +
				 delta[1] * delta[1] +
				 delta[2] * delta[2] +
				 delta[3] * delta[3] / (beta[3] * beta[3]) +
				 delta[4] * delta[4] / (beta[4] * beta[4]) +
				 delta[5] * delta[5] / (beta[5] * beta[5]);
		for (i = 0; i < 6; i++)
		{
			beta[i] -= delta[i];
		}
	}
	// copy results out
	accel_scale->x = beta[3] * GRAVITY_MSS;
	accel_scale->y = beta[4] * GRAVITY_MSS;
	accel_scale->z = beta[5] * GRAVITY_MSS;
	accel_offsets->x = beta[0] * accel_scale->x;
	accel_offsets->y = beta[1] * accel_scale->y;
	accel_offsets->z = beta[2] * accel_scale->z;

	// sanity check scale
	if (fabsf(accel_scale->x - 1.0f) > 0.2f || fabsf(accel_scale->y - 1.0f) > 0.2f || fabsf(accel_scale->z - 1.0f) > 0.2f)
	{
		temp = FAILED;
	}
	// sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
	if (fabsf(accel_offsets->x) > 3.5f || fabsf(accel_offsets->y) > 3.5f || fabsf(accel_offsets->z) > 3.5f)
	{
		temp = FAILED;
	}
	// return success or failure
	return temp;
}

Acce_Unit acce_sample[6] = {0};		  //����6�У�����6�����������
float acce_sample_sum[3] = {0, 0, 0}; //���ٶȺ�����
uint16_t sample_num;				  // ��������

void Accel_six_Calibartion(void)
{
	uint8_t i;
	uint8_t Cal_Flag;
	Acce_Unit new_offset = {
		0,
		0,
		0,
	};
	Acce_Unit new_scales = {
		1.0,
		1.0,
		1.0,
	};

	//�ϵ��Ԥ�ȣ����ݲ���
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
	}
	/*******************��Ӧ�ı�־λ��0**************/
	for (i = 0; i < 6; i++)
	{
		//		Accel_Calibration_Finished[i]=0;//��Ӧ���־λ����
		acce_sample[i].x = 0; //��ն�Ӧ��ļ��ٶȼ���
		acce_sample[i].y = 0; //��ն�Ӧ��ļ��ٶȼ���
		acce_sample[i].z = 0; //��ն�Ӧ��ļ��ٶȼ���
	}
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0; //��Ӧ��������0
	/***********************************************/

	/*************��ʼУ׼*******************/
	//��һ�ε�У׼�����棨��Ԫ�����ģ���ֱ����
	printf("first cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[0].x = acce_sample_sum[0] / 200; //ȡƽ��ֵ
	acce_sample[0].y = acce_sample_sum[1] / 200;
	acce_sample[0].z = acce_sample_sum[2] / 200;
	//У׼֮�������0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("one ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[0].x, acce_sample[0].y, acce_sample[0].z);
	delay_ms(1000);

	//�ڶ��ε�У׼���������һ�ߴ�ֱ����
	printf("second cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g������
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[1].x = acce_sample_sum[0] / 200;
	acce_sample[1].y = acce_sample_sum[1] / 200;
	acce_sample[1].z = acce_sample_sum[2] / 200;
	//У׼֮�������0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("two ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[1].x, acce_sample[1].y, acce_sample[1].z);
	delay_ms(1000);

	//������У׼,��������������Ǳ�û����Ĵ�ֱ����
	printf("third cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g������
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[2].x = acce_sample_sum[0] / 200;
	acce_sample[2].y = acce_sample_sum[1] / 200;
	acce_sample[2].z = acce_sample_sum[2] / 200;
	//У׼֮�������0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("three ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[2].x, acce_sample[2].y, acce_sample[2].z);
	delay_ms(1000);

	//���Ĵε�У׼����β��ֱ����
	printf("four cal\n");
	delay_x_ms(5);
	//�����ٶ�����
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g������
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[3].x = acce_sample_sum[0] / 200;
	acce_sample[3].y = acce_sample_sum[1] / 200;
	acce_sample[3].z = acce_sample_sum[2] / 200;
	//У׼֮�������0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("four ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[3].x, acce_sample[3].y, acce_sample[3].z);
	delay_ms(1000);

	//����ε�У׼������ûԪ�����Ĵ�ֱ���ϣ�
	printf("five cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g������
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[4].x = acce_sample_sum[0] / 200;
	acce_sample[4].y = acce_sample_sum[1] / 200;
	acce_sample[4].z = acce_sample_sum[2] / 200;
	//У׼֮�������0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("five ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[4].x, acce_sample[4].y, acce_sample[4].z);
	delay_ms(1000);

	//�����ε�У׼�����������У׼,jitouchaoshang
	printf("six cal\n");
	delay_x_ms(5);
	ACC_IMU_Filter();
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g������
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //���ٶȼ�ת��Ϊ1g����
	}
	acce_sample[5].x = acce_sample_sum[0] / 200;
	acce_sample[5].y = acce_sample_sum[1] / 200;
	acce_sample[5].z = acce_sample_sum[2] / 200;
	printf("six ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[5].x, acce_sample[5].y, acce_sample[5].z);
	delay_ms(1000);
	//����У׼����
	Cal_Flag = Calibrate_accel(acce_sample,
							   &new_offset,
							   &new_scales); //������6������
	if (Cal_Flag == SUCCESS)
	{
		B[0] = new_offset.x; //*One_G_TO_Accel;
		B[1] = new_offset.y; //*One_G_TO_Accel;
		B[2] = new_offset.z; //*One_G_TO_Accel;
		K[0] = new_scales.x;
		K[1] = new_scales.y;
		K[2] = new_scales.z;
	}
	printf("b0%.2f\tb1%.2f\tb2%.2f\tk0%.2f\tk1%.2f\tk2%.2f\n", B[0], B[1], B[2], K[0], K[1], K[2]);
	// b00.00	b10.00	b20.00	k01.00	k11.00	k21.00
}

////////////////////////////////////
//////////////������////////////////

//������ٶȣ������ǵ�ȫ�ֱ���
Struct_MPU_ACC MPU_ACC;
Struct_MPU_GYRO MPU_GYRO;

////////////////////////////////////
