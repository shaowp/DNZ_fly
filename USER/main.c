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
DNZ_fly�ɿس��� v0.1 @shaowp
main program
************************************************/

int main(void)
{
	delay_init();									//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(500000);								//���ڳ�ʼ��Ϊ115200
	SOFT_IIC_Init();								//��ʼ�����IIC
	MPU_Init();
	AK8975_Init();
	TIM1_Int_Init(4999, 71);  //5ms,200HZ	//5ms��Ϊ����
	TIM2_PWM_Init(19999, 71); //20ms,50HZ
	TIM4_Cap_Init(19999, 71);
	// Mag_Calibartion();	//У׼ʹ����λ��ANTMAGУ׼�������ݵ�����ΪtxtУ׼
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //ʹ��ָ����TIM3�ж�,��������ж�
											   //	Gyro_Calibartion();

	while (1)
	{
		//���ʹ���������
		ANO_DT_Send_Senser(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
						   MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
						   AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, 0);
		//���ͷɿ�״̬
		ANO_DT_Send_Status(IMU.roll, IMU.pitch, IMU.yaw, 0, 0, 0);
	}
}
