#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "softiic.h"
#include "mpu6050.h"
#include "AK8975.h"
#include "data_process.h"
#include "timer.h"
/************************************************
DNZ_fly飞控程序 v0.1 @shaowp
main program
************************************************/

int main(void)
{
	//. xmax:75	ymax:38	zmax:131	xmin:-125	ymin:-196	zmin:-92
// xo-25.00	 yo-79.00	 zo19.50	 0.85	 0.90
	//short gx, gy, gz;
// 	xmax:78	ymax:19	zmax:129	xmin:-156	ymin:-200	zmin:-120
// xo-39.00	 yo-90.50	 zo4.50	 1.07	 0.94
	delay_init();									//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);								//串口初始化为115200
	SOFT_IIC_Init();								//初始化软件IIC
	MPU_Init();
	AK8975_Init();
	TIM1_Int_Init(4999, 71); //5ms,200HZ	//5ms作为基础
	// Mag_Calibartion();
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);
	// delay_ms(1000);


	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM3中断,允许更新中断
											   //	Gyro_Calibartion();

	while (1)
	{

		// AK8975_Updata();
		// printf("%d\t%d\t%d\t", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);
		// MAG_IMU_Filter();
		// printf("%d\t%d\t%d\n", AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz);

		//MPU_Get_Gyroscope(&gx, &gy, &gz);
		// printf("%d\t%d\t%d\t", MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz);
		//printf("%d\t%d\t%d\t", MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz);
		// GYRO_IMU_Filter();
		// ACC_IMU_Filter(); //滤波
		// printf("%d\t%d\t%d\n", MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz);
		// printf("%d\t%d\t%d\n", MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz);
		// delay_ms(10);
	}
}
