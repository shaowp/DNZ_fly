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
#define USE_ANO_HOST //����1ʹ����������ģ���ʵ��̫���ˣ�ֻ������������

/************************************************
DNZ_fly�ɿس��� v0.1 @shaowp
main program
�ж����ȼ��б�����ԽС�����ȼ�Խ�ߣ�
����				�����ȼ�			�����ȼ�
TIM3 (ϵͳ����ʱ��)		1					1
TIM1�����жϣ�			2					0
TIM4��ң������			1					1
���ڵ���			    0					2
USBͨѶ				    1 					0


		����ֲ�		TIM2

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
	Set_System();									//ϵͳʱ�ӳ�ʼ��
	delay_init();									//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);								//���ڳ�ʼ��Ϊ115200 500000 // uart_init(500000);
	LED_Init();

	SOFT_IIC_Init(); //��ʼ�����IIC
	MPU_Init();
	AK8975_Init();
	ACC_IMU_Filter_Queue_init();
	// AT24CXX_Init();
	ACC_IMU_Filter_SOFT_INIT(200);
	
	TIM1_Int_Init(4999, 71);	//5ms,200HZ	//5ms��Ϊ����
	TIM3_Int_Init(49999, 7199); //7200��Ƶ 	//���μ���5s������1/10s��0.1ms
	TIM2_PWM_Init(19999, 71);   //20ms,50HZ
	TIM4_Cap_Init(19999, 71);   //72000000/72=1MHZ,��2W�Σ�1/1M s*2W=0.02s
	
	
	delay_ms(1800);
	USB_Port_Set(0); //USB�ȶϿ�
	delay_ms(700);
	USB_Port_Set(1); //USB�ٴ�����
	USB_Interrupts_Config();
	Set_USBClock();
	USB_Init();
	PID_Init(); //PID������ʼ��

	LED0 = 0;
	LED1 = 0;
	delay_ms(1000);
	LED0 = 1;
	LED1 = 1;
	Gyro_Calibartion(); //������У׼
	// Acc_Calibartion(); //�򵥵�ƫ��У׼

	// Accel_six_Calibartion();	//����У׼
	// Mag_Calibartion();  //У׼ʹ����λ��ANTMAGУ׼�������ݵ�����ΪtxtУ׼

								//72000000/7200=10000Hz,10kHz, 1/10k=0.0001s,��200��  10k/200=
	//BLDC_calibration();						   //���У׼	TIM2 //У׼һ�ξͺ�
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //ʹ��ָ����TIM1�ж�,��������ж�
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //ʹ��ָ����TIM3�ж�,��������ж�

	while (1)
	{
#ifdef USE_ANO_HOST
		//��usb_endp.c  EP1_OUT_Callback  ���޸ĵģ����� fuck_USB_REC �����ѵ�
		//#ifndef USE_443_data
		//���ʹ���������
		ANO_DT_Send_Senser(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
						   MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
						   AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, 0);
		delay_ms(1);
		//���ͷɿ�״̬
		ANO_DT_Send_Status(IMU.roll, IMU.pitch, IMU.yaw, 0, 0, 0);
		delay_ms(1);

		//����ң��������
		ANO_DT_Send_RCData(RC_data.thr, RC_data.yaw, RC_data.rol, RC_data.pit, 0, 0, 0, 0, 0, 0);
		delay_ms(1);

		//���͵������
		ANO_DT_Send_MotoPWM(MOTOR1 - 1000, MOTOR2 - 1000, MOTOR3 - 1000, MOTOR4 - 1000, 0, 0, 0, 0);
		delay_ms(1);

		//����PID����
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
