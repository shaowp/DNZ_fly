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
#include "rc_process.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "usbio.h"
#include "control.h"
#include "pid.h"
#include "ANO_USART.h"
#include "stflash.h"
#include "led.h"
#define USE_ANO_HOST //串口1使用数传，妈的，这实在太慢了，只能用来调参了

/************************************************
DNZ_fly飞控程序 v0.1 @shaowp
main program
中断优先级列表（数字越小，优先级越高）
名称				主优先级			次优先级
TIM3 (系统运行时间)		1					1
TIM1（主中断）			2					0
TIM4（遥控器）			1					1
串口调参			    0					2
USB通讯				    1 					0


		电机分布		TIM2

MOTOR1							  MOTOR4
CH4 PA3				^			  CH1 PA0
		*			*			*
			*		*		*
				*	*	*
					*
				*	*	*
			*		*		*
		*			*			*
MOTOR2							   MOTOR3
CH3	PA2							   CH2 PA1

************************************************/
int main(void)
{
	Set_System();									//系统时钟初始化
	delay_init();									//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);								//串口初始化为115200 500000 // uart_init(500000);
	LED_Init();

	SOFT_IIC_Init(); //初始化软件IIC
	MPU_Init();
	AK8975_Init();
	ACC_IMU_Filter_Queue_init();
	// AT24CXX_Init();
	ACC_IMU_Filter_SOFT_INIT(200);
	
	TIM1_Int_Init(4999, 71);	//5ms,200HZ	//5ms作为基础
	TIM3_Int_Init(49999, 7199); //7200分频 	//单次计数5s，精度1/10s，0.1ms
	TIM2_PWM_Init(19999, 71);   //20ms,50HZ
	TIM4_Cap_Init(19999, 71);   //72000000/72=1MHZ,跑2W次，1/1M s*2W=0.02s
	
	
	delay_ms(1800);
	USB_Port_Set(0); //USB先断开
	delay_ms(700);
	USB_Port_Set(1); //USB再次连接
	USB_Interrupts_Config();
	Set_USBClock();
	USB_Init();
	PID_Init(); //PID参数初始化

	LED0 = 0;
	LED1 = 0;
	delay_ms(1000);
	LED0 = 1;
	LED1 = 1;
	Gyro_Calibartion(); //陀螺仪校准
	// Acc_Calibartion(); //简单的偏置校准

	// Accel_six_Calibartion();	//六面校准
	// Mag_Calibartion();  //校准使用上位机ANTMAG校准，吧数据导出来为txt校准

								//72000000/7200=10000Hz,10kHz, 1/10k=0.0001s,跑200次  10k/200=
	//BLDC_calibration();						   //电调校准	TIM2 //校准一次就好
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //使能指定的TIM3中断,允许更新中断

	while (1)
	{
#ifdef USE_ANO_HOST
		//在usb_endp.c  EP1_OUT_Callback  中修改的，查找 fuck_USB_REC 可以搜到
		//#ifndef USE_443_data
		//发送传感器数据
		ANO_DT_Send_Senser(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
						   MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
						   AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, 0);
		delay_ms(1);
		//发送飞控状态
		ANO_DT_Send_Status(IMU.roll, IMU.pitch, IMU.yaw, 0, 0, 0);
		delay_ms(1);

		//发送遥控器数据
		ANO_DT_Send_RCData(RC_data.thr, RC_data.yaw, RC_data.rol, RC_data.pit, 0, 0, 0, 0, 0, 0);
		delay_ms(1);

		//发送电机数据
		ANO_DT_Send_MotoPWM(MOTOR1 - 1000, MOTOR2 - 1000, MOTOR3 - 1000, MOTOR4 - 1000, 0, 0, 0, 0);
		delay_ms(1);

		//发送PID数据
		if (PID_send_flag)
		{
			send_PID();
			delay_ms(1);
			PID_send_flag = 0;
		}

		if (PID_save_flag)
		{
			//save_PID();
			delay_ms(1);
			PID_save_flag = 0;
		}
#else
		printf("ACC data:%.2f  %.2f  %.2f\n", MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz);
		printf("GYR data:%.2f  %.2f  %.2f\n", MPU_ACC.gyrox, MPU_ACC.gyroy, MPU_ACC.gyroz);
		printf("IMU data:%.2f  %.2f  %.2f\n", IMU.roll, IMU.pitch, IMU.yaw);

#endif
	}
}
