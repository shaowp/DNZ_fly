#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "softiic.h"
#include "mpu6050.h"
#include "AK8975.h"
/************************************************
DNZ_fly�ɿس��� v0.1 @shaowp
************************************************/

int main(void)
{
	short gx,gy,gz;
	delay_init();									//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);								//���ڳ�ʼ��Ϊ115200
	SOFT_IIC_Init();								//��ʼ�����IIC
	MPU_Init();
	AK8975_Init();
	while (1)
	{
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		printf("%d\t%d\t%d\n",gx,gy,gz);
		delay_ms(10);

	}
}
