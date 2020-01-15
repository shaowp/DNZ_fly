#include "scheduler.h"
#include "control.h"
#include "pid.h"
#include "timer.h"
#include "led.h"
//定时器1的定时任务
//核心处理函数
void TIM1_UP_IRQHandler(void) //TIM3中断
{
	static uint32_t count = 0;
	static u32 nowtime = 0;
	static u32 usertime = 0;
	short gx, gy, gz; //空变量
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		nowtime = get_system_time();
		// printf("GetTicks:%d\n",GetTicks());
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		count++;
		//5ms的任务
		//读取所有的数据
		MPU_Get_Gyroscope(&gx, &gy, &gz);
		MPU_Get_Accelerometer(&gx, &gy, &gz);
		// GYRO_IMU_Filter();
		ACC_IMU_Filter_ANO(); //加速度和磁力计原始数据滤波

		//15ms的任务
		//更新磁力计
		if (count % 3 == 0)
		{

			AK8975_Updata();  //获取校准过的数据
			MAG_IMU_Filter(); //滤波，且由上一次的状态得到本次的偏航角参数
		}

		//50ms任务，发送数据到上位机
		if (count % 10 == 0)
		{
		}
		//10s的任务，发送PID数据
		if (count % 2000 == 0)
		{
			//发送PID数据
			//初始化将数据发出去
		}
		if (count == 2000) //5ms,10ms,15ms,20ms都在这个120里面了
			count = 0;

		Unlock_check(); //检测是否上锁

		//姿态解算 mahony互补滤波，效果不太好
		// IMUupdate(MPU_GYRO.gyrox * 1.0, MPU_GYRO.gyroy * 1.0, MPU_GYRO.gyroz * 1.0,
		// 		  MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
		// 		  AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);

		//梯度下降法，效果很好
		AHRSUpdate_GraDes_Delay_Corretion(MPU_GYRO.gyrox * 1.0, MPU_GYRO.gyroy * 1.0, MPU_GYRO.gyroz * 1.0,
										  MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
										  AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
		AttitudePidControl(); //姿态PID控制
		MotorControl();		  //电机输出
		usertime = get_system_time() - nowtime;
		usertime = usertime; //去warning
	}
}
