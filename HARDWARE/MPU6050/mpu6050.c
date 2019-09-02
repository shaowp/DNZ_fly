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
float K[3] = {0.99, 11.0, 20.97}; //默认标度误差
float B[3] = {0.26, -0.3, 20.21};   //默认零位误差

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); //复位MPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); //唤醒MPU6050
	MPU_Set_Gyro_Fsr(3);					 //陀螺仪传感器,±1000dps///参数为3的话为±2000
	MPU_Set_Accel_Fsr(0);					 //加速度传感器,±2g
	MPU_Set_Rate(300);						 //设置采样率300Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); //I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   //关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); //INT引脚低电平有效
	res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if (res == MPU_ADDR) //器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); //设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //加速度与陀螺仪都工作
		MPU_Set_Rate(300);						 //设置采样率为300Hz
		printf("MPU INIT SUCCESS\n");
	}
	else
	{
		printf("MPU INIT ERROR\n");
		return 1;
	}

	//设置变量的offset
	MPU_GYRO.gyrox_offset = -112;
	MPU_GYRO.gyroy_offset = 101;
	MPU_GYRO.gyroz_offset = 4;

	MPU_ACC.accx_offset = 0;
	MPU_ACC.accy_offset = 0;
	MPU_ACC.accz_offset = 0;

	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); //设置陀螺仪满量程范围
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); //设置加速度传感器满量程范围
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
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
	return MPU_Write_Byte(MPU_CFG_REG, data); //设置数字低通滤波器
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if (rate > 1000)
		rate = 1000;
	if (rate < 4)
		rate = 4;
	data = 1000 / rate - 1;
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); //设置数字低通滤波器
	return MPU_Set_LPF(rate / 2);					  //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
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
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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

	// MPU_ACC.accx = K[0] * temp_ax - B[0] * One_G_TO_Accel; //六面校准得出的K、B值在这里用到
	// MPU_ACC.accy = K[1] * temp_ay - B[1] * One_G_TO_Accel;
	// MPU_ACC.accz = K[2] * temp_az - B[2] * One_G_TO_Accel;
	MPU_ACC.accx = temp_ax;
	MPU_ACC.accy = temp_ay;
	MPU_ACC.accz = temp_az;
	return res;
}
//IIC连续写
//addr:器件地址
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
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
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
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
u8 MPU_Write_Byte(u8 reg, u8 data)
{
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
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
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 0); //发送器件地址+写命令
	SOFT_IIC_Wait_Ack();					 //等待应答
	SOFT_IIC_Send_Byte(reg);				 //写寄存器地址
	SOFT_IIC_Wait_Ack();					 //等待应答
	SOFT_IIC_Start();
	SOFT_IIC_Send_Byte((MPU_ADDR << 1) | 1); //发送器件地址+读命令
	SOFT_IIC_Wait_Ack();					 //等待应答
	res = SOFT_IIC_Read_Byte(0);			 //读取数据,发送nACK
	SOFT_IIC_Stop();						 //产生一个停止条件
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
	//需要先判断飞控是否禁止
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
	for (i = 0; i < 100; i++) //连续采样100次，一共耗时100*3=300ms
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
	MPU_GYRO.gyrox_offset = g_Gyro_xoffset / 100; //得到静态偏移值//采集了100组做平均
	MPU_GYRO.gyroy_offset = g_Gyro_yoffset / 100;
	MPU_GYRO.gyroz_offset = g_Gyro_zoffset / 100;
	printf("gxoff:%d\tgyoff:%d\tgzoff:%d\n", MPU_GYRO.gyrox_offset, MPU_GYRO.gyroy_offset, MPU_GYRO.gyroz_offset);
}

//加速度计六面校准
void Acc_Calibartion(void)
{
}

/**************************************************************************/
/***************************APM校准加速度计*******************************/
/*第一面飞控平放，Z轴正向朝着正上方（正面垂直朝上），Z axis is about 1g,X、Y is about 0g*/
/*第二面飞控平放，X轴正向朝着正上方（机头垂直朝上），X axis is about 1g,Y、Z is about 0g*/
/*第三面飞控平放，X轴正向朝着正下方（左侧垂直朝上），X axis is about -1g,Y、Z is about 0g*/
/*第四面飞控平放，Y轴正向朝着正下方（右侧垂直朝上），Y axis is about -1g,X、Z is about 0g*/
/*第五面飞控平放，Y轴正向朝着正上方（机尾垂直朝上），Y axis is about 1g,X、Z is about 0g*/
/*第六面飞控平放，Z轴正向朝着正下方（背面垂直朝上），Z axis is about -1g,X、Y is about 0g*/

/*
1. 	平
2. 	机头朝上
3.	机尾朝上
4. 	usb口朝下
5. 	usb口朝上
6. 	反
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

void get_yuanshi_acc(void) //读取原始ACC,并且滤波
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

Acce_Unit acce_sample[6] = {0};		  //三行6列，保存6面待矫正数据
float acce_sample_sum[3] = {0, 0, 0}; //加速度和数据
uint16_t sample_num;				  // 采样次数

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

	//上电后预热，数据不对
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
	}
	/*******************对应的标志位归0**************/
	for (i = 0; i < 6; i++)
	{
		//		Accel_Calibration_Finished[i]=0;//对应面标志位清零
		acce_sample[i].x = 0; //清空对应面的加速度计量
		acce_sample[i].y = 0; //清空对应面的加速度计量
		acce_sample[i].z = 0; //清空对应面的加速度计量
	}
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0; //对应面数据清0
	/***********************************************/

	/*************开始校准*******************/
	//第一次点校准，正面（有元器件的）垂直朝上
	printf("first cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[0].x = acce_sample_sum[0] / 200; //取平均值
	acce_sample[0].y = acce_sample_sum[1] / 200;
	acce_sample[0].z = acce_sample_sum[2] / 200;
	//校准之后变量归0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("one ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[0].x, acce_sample[0].y, acce_sample[0].z);
	delay_ms(1000);

	//第二次点校准，有排针的一边垂直朝上
	printf("second cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程下
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[1].x = acce_sample_sum[0] / 200;
	acce_sample[1].y = acce_sample_sum[1] / 200;
	acce_sample[1].z = acce_sample_sum[2] / 200;
	//校准之后变量归0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("two ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[1].x, acce_sample[1].y, acce_sample[1].z);
	delay_ms(1000);

	//第三次校准,与有排针对立的那边没排针的垂直朝上
	printf("third cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程下
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[2].x = acce_sample_sum[0] / 200;
	acce_sample[2].y = acce_sample_sum[1] / 200;
	acce_sample[2].z = acce_sample_sum[2] / 200;
	//校准之后变量归0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("three ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[2].x, acce_sample[2].y, acce_sample[2].z);
	delay_ms(1000);

	//第四次点校准，机尾垂直朝上
	printf("four cal\n");
	delay_x_ms(5);
	//读加速度数据
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程下
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[3].x = acce_sample_sum[0] / 200;
	acce_sample[3].y = acce_sample_sum[1] / 200;
	acce_sample[3].z = acce_sample_sum[2] / 200;
	//校准之后变量归0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("four ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[3].x, acce_sample[3].y, acce_sample[3].z);
	delay_ms(1000);

	//第五次点校准，背面没元器件的垂直朝上，
	printf("five cal\n");
	delay_x_ms(5);
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程下
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[4].x = acce_sample_sum[0] / 200;
	acce_sample[4].y = acce_sample_sum[1] / 200;
	acce_sample[4].z = acce_sample_sum[2] / 200;
	//校准之后变量归0
	sample_num = 0;
	acce_sample_sum[0] = acce_sample_sum[1] = acce_sample_sum[2] = 0;
	printf("five ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[4].x, acce_sample[4].y, acce_sample[4].z);
	delay_ms(1000);

	//第六次点校准，进入第六面校准,jitouchaoshang
	printf("six cal\n");
	delay_x_ms(5);
	ACC_IMU_Filter();
	for (sample_num = 0; sample_num < 200; sample_num++)
	{
		get_yuanshi_acc();
		acce_sample_sum[0] += MPU_ACC.accx * ACCEL_TO_1G; //加速度计转化为1g量程下
		acce_sample_sum[1] += MPU_ACC.accy * ACCEL_TO_1G; //加速度计转化为1g量程
		acce_sample_sum[2] += MPU_ACC.accz * ACCEL_TO_1G; //加速度计转化为1g量程
	}
	acce_sample[5].x = acce_sample_sum[0] / 200;
	acce_sample[5].y = acce_sample_sum[1] / 200;
	acce_sample[5].z = acce_sample_sum[2] / 200;
	printf("six ok\n");
	printf("x:%.2f\ty:%.2f\tz:%.2f\n\n", acce_sample[5].x, acce_sample[5].y, acce_sample[5].z);
	delay_ms(1000);
	//保存校准数据
	Cal_Flag = Calibrate_accel(acce_sample,
							   &new_offset,
							   &new_scales); //将所得6面数据
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
//////////////变量区////////////////

//定义加速度，陀螺仪的全局变量
Struct_MPU_ACC MPU_ACC;
Struct_MPU_GYRO MPU_GYRO;

////////////////////////////////////
