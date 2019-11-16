#include "data_process.h"


//加速度计滤波
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

//磁力计滤波
void GYRO_IMU_Filter(void)
{
	const float factor = 0.15f; //滤波因素
	static float tBuff[3];
	MPU_GYRO.gyrox = tBuff[0] = tBuff[0] * (1 - factor) + MPU_GYRO.gyrox * factor;
	MPU_GYRO.gyroy = tBuff[1] = tBuff[1] * (1 - factor) + MPU_GYRO.gyroy * factor;
	MPU_GYRO.gyroz = tBuff[2] = tBuff[2] * (1 - factor) + MPU_GYRO.gyroz * factor;
}

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
}
