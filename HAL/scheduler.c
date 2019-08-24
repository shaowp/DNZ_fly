#include "scheduler.h"

//定时器1的定时任务
//核心处理函数
void TIM1_UP_IRQHandler(void) //TIM3中断
{
	//空变量
	static uint32_t count = 0;
	short gx, gy, gz;
	float norm;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		count++;

		//5ms的任务
		//读取所有的数据
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		MPU_Get_Accelerometer(&gx, &gy, &gz);
		GYRO_IMU_Filter();
		ACC_IMU_Filter();
		//printf("%d\t%d\t%d\t", MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz);
		//printf("%d\t%d\t%d\n", MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz);
		//姿态解算
		IMUupdate(MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
				  MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
				  AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);

		//15ms的任务
		//更新磁力计的偏航角
		if (count % 3 == 0)
		{
			float TempX;
			float TempY;

			AK8975_Updata();
			MAG_IMU_Filter();
			//磁力计归一化
			// norm = Q_rsqrt(AK8975_MAG.mx * AK8975_MAG.mx + AK8975_MAG.my * AK8975_MAG.my + AK8975_MAG.mz * AK8975_MAG.mz);
			// AK8975_MAG.mx = AK8975_MAG.mx * norm;
			// AK8975_MAG.my = AK8975_MAG.my * norm;
			// AK8975_MAG.mz = AK8975_MAG.mz * norm;

			// printf("mx%.2f\tmy%.2f\tmz%.2f\t", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
			//利用罗盘来计算偏航角度
			TempX = AK8975_MAG.mx * IMU.Cos_Roll + AK8975_MAG.mz * IMU.Sin_Roll;
			TempY = AK8975_MAG.mx * IMU.Sin_Pitch * IMU.Sin_Roll + AK8975_MAG.my * IMU.Cos_Pitch - AK8975_MAG.mz * IMU.Cos_Roll * IMU.Sin_Pitch;
			/***********反正切得到磁力计观测角度*********/
			IMU.yaw_mag = atan2(TempX, TempY) * 57.296; //得到罗盘偏航角
			printf("mx %.2f\tmy %.2f\tmz %.2f\tyaw_mag %.2f\n", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, IMU.yaw_mag);
		}

		if (count == 120) //5ms,10ms,15ms,20ms都在这个120里面了
			count = 0;
	}
}
