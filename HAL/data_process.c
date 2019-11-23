#include "data_process.h"
#include "filter.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "IMU.h"

#define QUEUE_SIZE 20







//匿名滤波
//---------------------------------------------------------------------//
#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN16G_TO_CMSS 0.4790f
#define GYR_ACC_FILTER 0.25f //陀螺仪加速度计滤波系数
enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};
typedef struct
{
	float center_pos_cm[VEC_XYZ];
	float gyro_rad[VEC_XYZ];
	float gyro_rad_old[VEC_XYZ];
	float gyro_rad_acc[VEC_XYZ];
	float linear_acc[VEC_XYZ];
} _center_pos_st;
_center_pos_st center_pos;

static float gyr_f[5][VEC_XYZ], acc_f[5][VEC_XYZ];

void ACC_IMU_Filter_ANO(void)
{

	float hz = 200;
	//MPU_ACC.accx
	//MPU_GYRO.gyrox

	gyr_f[4][X] = MPU_GYRO.gyrox;
	gyr_f[4][Y] = MPU_GYRO.gyroy;
	gyr_f[4][Z] = MPU_GYRO.gyroz;

	acc_f[4][X] = MPU_ACC.accx;
	acc_f[4][Y] = MPU_ACC.accy;
	acc_f[4][Z] = MPU_ACC.accz;

	for (u8 i = 0; i < 3; i++)
	{
		for (u8 j = 4; j > 0; j--)
		{
			//
			gyr_f[j - 1][X + i] += GYR_ACC_FILTER * (gyr_f[j][X + i] - gyr_f[j - 1][X + i]);
			acc_f[j - 1][X + i] += GYR_ACC_FILTER * (acc_f[j][X + i] - acc_f[j - 1][X + i]);
		}
	}

	//旋转加速度补偿
	for (u8 i = 0; i < 3; i++)
	{
		center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
		center_pos.gyro_rad[i] = gyr_f[0][X + i] * RANGE_PN2000_TO_RAD; //0.001065f;
		center_pos.gyro_rad_acc[i] = hz * (center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
	}
	center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] * center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] * center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] * center_pos.center_pos_cm[Y];

	//滤波之后赋值
	MPU_GYRO.gyrox = gyr_f[0][0];
	MPU_GYRO.gyroy = gyr_f[0][1];
	MPU_GYRO.gyroz = gyr_f[0][2];

	MPU_ACC.accx = acc_f[0][0] - center_pos.linear_acc[0] / RANGE_PN16G_TO_CMSS; //sensor_val_ref[A_X+i];//
	MPU_ACC.accy = acc_f[0][1] - center_pos.linear_acc[1] / RANGE_PN16G_TO_CMSS; //sensor_val_ref[A_X+i];//
	MPU_ACC.accz = acc_f[0][2] - center_pos.linear_acc[2] / RANGE_PN16G_TO_CMSS; //sensor_val_ref[A_X+i];//
}
//---------------------------------------------------------------------//




//-----------------------------------------------------------------------//
//软件低通滤波
#define ACCEL_LPF_CUTOFF_FREQ 8.0f
biquadFilter_t accFilterLPF[3]; //二阶低通滤波器
void ACC_IMU_Filter_SOFT_INIT(float accUpdateRate)
{
	for (int axis = 0; axis < 3; axis++)
	{
		biquadFilterInitLPF(&accFilterLPF[axis], accUpdateRate, ACCEL_LPF_CUTOFF_FREQ);
	}
}

//加速度计软件低通滤波
void ACC_IMU_Filter_SOFT(void)
{

	const float factor = 0.2f; //滤波因素
	static float tBuff[3];
	MPU_ACC.accx = tBuff[0] * (1 - factor) + MPU_ACC.accx * factor;
	MPU_ACC.accy = tBuff[1] * (1 - factor) + MPU_ACC.accy * factor;
	MPU_ACC.accz = tBuff[2] * (1 - factor) + MPU_ACC.accz * factor;

	float accfx = (float)MPU_ACC.accx / 4096.0;
	float accfy = (float)MPU_ACC.accy / 4096.0;
	float accfz = (float)MPU_ACC.accz / 4096.0;

	accfx = biquadFilterApply(&accFilterLPF[0], accfx);
	accfy = biquadFilterApply(&accFilterLPF[1], accfy);
	accfz = biquadFilterApply(&accFilterLPF[2], accfz);

	MPU_ACC.accx = accfx * 4096;
	MPU_ACC.accy = accfy * 4096;
	MPU_ACC.accz = accfz * 4096;
	tBuff[0] = MPU_ACC.accx;
	tBuff[1] = MPU_ACC.accy;
	tBuff[2] = MPU_ACC.accz;
}
//--------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------//
//加速度计巴特沃斯滤波
struct _ButterWorth2d_Acc_Tag
{
	int16_t input[3];
	int16_t output[3];
};
struct _ButterWorth2d_Acc_Tag accButter[3] =
	{
		/* input[3] output[3] */
		{0, 0, 0, 0, 0, 0}, // X-axis
		{0, 0, 0, 0, 0, 0}, // Y-axis
		{0, 0, 0, 0, 0, 0}, // Z-axis
};

void ACC_IMU_Filter_ButterWorth(void)
{
	const static float b_acc[3] = {0.1311064399166, 0.2622128798333, 0.1311064399166};
	const static float a_acc[3] = {1, -0.7477891782585, 0.272214937925};
	float accelFilter[3];
	uint8_t axis;
	accButter[0].input[2] = (int16_t)(MPU_ACC.accx);
	accButter[1].input[2] = (int16_t)(MPU_ACC.accy);
	accButter[2].input[2] = (int16_t)(MPU_ACC.accz);

	for (axis = 0; axis < 3; axis++)
	{
		accButter[axis].output[2] =
			(int16_t)(b_acc[0] * accButter[axis].input[2] + b_acc[1] * accButter[axis].input[1] + b_acc[2] * accButter[axis].input[0] - a_acc[1] * accButter[axis].output[1] - a_acc[2] * accButter[axis].output[0]);
		accelFilter[axis] = accButter[axis].output[2];
	}
	for (axis = 0; axis < 3; axis++)
	{
		/* x(n) 序列保存 */
		accButter[axis].input[0] = accButter[axis].input[1];
		accButter[axis].input[1] = accButter[axis].input[2];
		/* y(n) 序列保存 */
		accButter[axis].output[0] = accButter[axis].output[1];
		accButter[axis].output[1] = accButter[axis].output[2];
	}
	MPU_ACC.accx = accelFilter[0];
	MPU_ACC.accy = accelFilter[1];
	MPU_ACC.accz = accelFilter[2];
}
//---------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------//
//卡尔曼滤波
void ACC_IMU_Filter(void)
{
	//卡尔曼参数
	static struct _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.543},
										  {0.02, 0, 0, 0, 0.001, 0.543},
										  {0.02, 0, 0, 0, 0.001, 0.543}};
	kalman_1(&ekf[0], 1.0 * MPU_ACC.accx); //一维卡尔曼
	kalman_1(&ekf[1], 1.0 * MPU_ACC.accy); //一维卡尔曼
	kalman_1(&ekf[2], 1.0 * MPU_ACC.accz); //一维卡尔曼

	MPU_ACC.accx = ekf[0].out;
	MPU_ACC.accy = ekf[1].out;
	MPU_ACC.accz = ekf[2].out;
}
//---------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------//
//磁力计滤波
void GYRO_IMU_Filter(void)
{
	const float factor = 0.15f; //滤波因素
	static float tBuff[3];
	MPU_GYRO.gyrox = tBuff[0] = tBuff[0] * (1 - factor) + MPU_GYRO.gyrox * factor;
	MPU_GYRO.gyroy = tBuff[1] = tBuff[1] * (1 - factor) + MPU_GYRO.gyroy * factor;
	MPU_GYRO.gyroz = tBuff[2] = tBuff[2] * (1 - factor) + MPU_GYRO.gyroz * factor;
}
//---------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------//
//磁力计滤波
void MAG_IMU_Filter(void)
{
	static struct _1_ekf_filter ekf[3] = {{0.02, 0, 0, 0, 0.001, 0.1}, {0.02, 0, 0, 0, 0.001, 0.1}, {0.02, 0, 0, 0, 0.001, 0.1}};
	kalman_1(&ekf[0], 1.0 * AK8975_MAG.mx); //一维卡尔曼
	kalman_1(&ekf[1], 1.0 * AK8975_MAG.my); //一维卡尔曼
	kalman_1(&ekf[2], 1.0 * AK8975_MAG.mz); //一维卡尔曼
	AK8975_MAG.mx = ekf[0].out;
	AK8975_MAG.my = ekf[1].out;
	AK8975_MAG.mz = ekf[2].out;

	/************磁力计倾角补偿*****************/
	float TempX;
	float TempY;
	TempX = AK8975_MAG.mx * IMU.Cos_Roll + AK8975_MAG.mz * IMU.Sin_Roll;
	TempY = AK8975_MAG.mx * IMU.Sin_Pitch * IMU.Sin_Roll + AK8975_MAG.my * IMU.Cos_Pitch - AK8975_MAG.mz * IMU.Cos_Roll * IMU.Sin_Pitch;
	/***********反正切得到磁力计观测角度*********/
	IMU.yaw_mag = atan2(TempX, TempY) * 57.296; //得到罗盘偏航角
}
//---------------------------------------------------------------------------------//



//---------------------------------------------------------------------------------//
//滑动均值滤波
//队列部分,三个队列
Typedef_Queue *qACCx;
Typedef_Queue *qACCy;
Typedef_Queue *qACCz;

float qACCx1, qACCx2, qACCx3, qACCx4, qACCx5;

void ACC_IMU_Filter_Queue_init(void)
{
	qACCx = CreatQueue(QUEUE_SIZE);
	qACCy = CreatQueue(QUEUE_SIZE);
	qACCz = CreatQueue(QUEUE_SIZE);
}
void ACC_IMU_Filter_Queue(void)
{



	// float tempACCx = 0;
	// qACCx1 = qACCx2;
	// qACCx2 = qACCx3;
	// qACCx3 = qACCx4;
	// qACCx4 = qACCx5;
	// qACCx5 = MPU_ACC.accx;
	// tempACCx = qACCx1 + qACCx2 + qACCx3 + qACCx4 + qACCx5;
	// MPU_ACC.accx = tempACCx / 5;

	double tempACCx = 0, tempACCy = 0, tempACCz = 0;
	Enqueue(qACCx, 1.0 * MPU_ACC.accx);
	Enqueue(qACCy, 1.0 * MPU_ACC.accy);
	Enqueue(qACCz, 1.0 * MPU_ACC.accz);

	for (u8 i = 0; i < QUEUE_SIZE; i++)
	{
		tempACCx += qACCx->Array[i];
		tempACCy += qACCy->Array[i];
		tempACCz += qACCz->Array[i];
	}
	MPU_ACC.accx = tempACCx / QUEUE_SIZE;
	MPU_ACC.accy = tempACCy / QUEUE_SIZE;
	MPU_ACC.accz = tempACCz / QUEUE_SIZE;
}

int IsEmpty(Typedef_Queue *q) //判断是否空
{
	return (q->Size == 0); //如果为空，返回1；
}

int IsFull(Typedef_Queue *q) //判断是否满
{
	return (q->Size == QUEUE_SIZE); //如果满了，返回1；
}

Typedef_Queue *CreatQueue(int MaxCount) //创建
{
	Typedef_Queue *q;
	q = (Typedef_Queue *)malloc(sizeof(Typedef_Queue));
	if (q == NULL)
	{
		// DEBUG_printf("no enough space!!!\n");
		return 0;
	}
	q->Array = (float *)malloc(sizeof(float) * QUEUE_SIZE);
	if (q->Array == NULL)
	{
		// DEBUG_printf("no enough space!!!\n");
		return 0;
	}

	q->Capacity = QUEUE_SIZE;
	q->Front = 1;
	q->Rear = 0;
	q->Size = 0;

	return q;
}
void MakeEmpty(Typedef_Queue *q) //使其空
{
	q->Front = 1;
	q->Rear = 0;
	q->Size = 0;
}

int Succ(int value, Typedef_Queue *q)
{
	//环形队列的关键，如果元素是10个，标号0-9，如果尾巴到了9，++9就是10，那么就会自动跳到0
	if (++value == q->Capacity)
		return 0;
	else
		return value;
}

//第0个数据最为尾巴，即最后一个数据
//就是第一遍，不需要绕回来的时候，第0个事不存东西的，除非绕回来才存东西
void Enqueue(Typedef_Queue *q, float x) //入队
{
	if (IsFull(q))
	{
		// DEBUG_printf("Full queue!!!\n");
		FrontAndDequeue(q); //出队一个
		q->Size++;
		q->Rear = Succ(q->Rear, q);
		q->Array[q->Rear] = x;
	}
	else
	{
		q->Size++;
		q->Rear = Succ(q->Rear, q);
		q->Array[q->Rear] = x;
	}
}
void Dequeue(Typedef_Queue *q) //出队
{
}
float FrontAndDequeue(Typedef_Queue *q) //出队，返回出队的元素
{
	float back_char;
	if (IsEmpty(q))
	{
		// DEBUG_printf("Empty queue!!!\n");
		return 0;
	}
	else
	{
		q->Size--;
		back_char = q->Array[q->Front];
		q->Front = Succ(q->Front, q);
		return back_char;
	}
}
//---------------------------------------------------------------------------------//




