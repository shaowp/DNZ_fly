#include "scheduler.h"

//定时器1的定时任务
//核心处理函数
void TIM1_UP_IRQHandler(void) //TIM3中断
{
	static uint32_t count = 0;
	short gx, gy, gz; //空变量
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
		//姿态解算
		IMUupdate(MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
				  MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
				  AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);

		//15ms的任务
		//更新磁力计
		if (count % 3 == 0)
		{
			// float TempX;
			// float TempY;
			AK8975_Updata();
			MAG_IMU_Filter();

			//这里的原理其实和上面的mahony互补滤波一样，都是转换坐标系，都可以实现
			// TempX = AK8975_MAG.mx * IMU.Cos_Roll + AK8975_MAG.mz * IMU.Sin_Roll;
			// TempY = AK8975_MAG.mx * IMU.Sin_Pitch * IMU.Sin_Roll + AK8975_MAG.my * IMU.Cos_Pitch - AK8975_MAG.mz * IMU.Cos_Roll * IMU.Sin_Pitch;
			/***********反正切得到磁力计观测角度*********/
			//IMU.yaw_mag = atan2(TempX, TempY) * 57.296; //得到罗盘偏航角
			// printf("mx %.2f\tmy %.2f\tmz %.2f\tyaw_mag %.2f\n", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, IMU.yaw_mag);
		}

		//50ms任务，发送数据到上位机
		if (count % 10 == 0)
		{
			//500000波特率到匿名上位机
			// usart1_report_imu(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
			// 				  MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
			// 				  (int)(IMU.roll * 100), (int)(IMU.pitch * 100), (int)(IMU.yaw * 10));
			//115200波特率到串口助手
			// printf("roll:%.2f\tpitch:%.2f\tyaw:%.2f\t \n", IMU.roll, IMU.pitch, IMU.yaw);
		}
		if (count == 120) //5ms,10ms,15ms,20ms都在这个120里面了
			count = 0;
	}
}
