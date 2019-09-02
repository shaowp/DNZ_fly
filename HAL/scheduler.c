#include "scheduler.h"
#include "control.h"
//定时器1的定时任务
//核心处理函数
void TIM1_UP_IRQHandler(void) //TIM3中断
{
	static uint32_t count = 0;
	short gx, gy, gz; //空变量
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		// printf("GetTicks:%d\n",GetTicks());
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		count++;

		//5ms的任务
		//读取所有的数据
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		MPU_Get_Accelerometer(&gx, &gy, &gz);
		GYRO_IMU_Filter();
		ACC_IMU_Filter();

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
		}
		if (count == 120) //5ms,10ms,15ms,20ms都在这个120里面了
			count = 0;

		//姿态解算
		IMUupdate(MPU_GYRO.gyrox * 1.0, MPU_GYRO.gyroy * 1.0, MPU_GYRO.gyroz * 1.0,
				  MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
				  AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
		AttitudePidControl(); //姿态PID控制
		MotorControl();		  //电机输出
	}
}
