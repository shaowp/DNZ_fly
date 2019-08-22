#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "softiic.h"
#include "mpu6050.h"
#include "AK8975.h"
/************************************************
DNZ_fly飞控程序 v0.1 @shaowp
************************************************/

int main(void)
{
	short gx,gy,gz;
	delay_init();									//延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);								//串口初始化为115200
	SOFT_IIC_Init();								//初始化软件IIC
	MPU_Init();
	AK8975_Init();
	while (1)
	{
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		printf("%d\t%d\t%d\n",gx,gy,gz);
		delay_ms(10);

	}
}
