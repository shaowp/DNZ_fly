#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "softiic.h"
#include "mpu6050.h"
#include "AK8975.h"
#include "data_process.h"
#include "timer.h"
#include "IMU.h"
#include "ANO_USART.h"
#include "timer.h"
/************************************************
DNZ_fly飞控程序 v0.1 @shaowp
main program
************************************************/

int main(void)
{
	delay_init();									//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);								//串口初始化为115200
	SOFT_IIC_Init();								//初始化软件IIC
	MPU_Init();
	AK8975_Init();
	TIM1_Int_Init(4999, 71);  //5ms,200HZ	//5ms作为基础
	TIM2_PWM_Init(19999, 71); //20ms,50HZ
	TIM4_Cap_Init(19999, 71);
	// Mag_Calibartion();	//校准使用上位机ANTMAG校准，吧数据导出来为txt校准
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM3中断,允许更新中断
											   //	Gyro_Calibartion();

	while (1)
	{
		//发送传感器数据
		ANO_DT_Send_Senser(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
						   MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
						   AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, 0);
		//发送飞控状态
		ANO_DT_Send_Status(IMU.roll, IMU.pitch, IMU.yaw, 0, 0, 0);
	}
}
