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

#define USE_443_data	//����1ʹ����������ģ���ʵ��̫���ˣ�ֻ������������

/************************************************
DNZ_fly�ɿس��� v0.1 @shaowp
main program
�ж����ȼ��б�����ԽС�����ȼ�Խ�ߣ�
����				�����ȼ�			�����ȼ�
TIM1�����жϣ�			2					0
TIM4��ң������			1					1
���ڵ���			    0					2
USBͨѶ				    1 					0
************************************************/
int main(void)
{
	Set_System();									//ϵͳʱ�ӳ�ʼ��
	delay_init();									//��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);								//���ڳ�ʼ��Ϊ115200 500000
	SOFT_IIC_Init();								//��ʼ�����IIC
	MPU_Init();
	AK8975_Init();
	AT24CXX_Init();

	TIM1_Int_Init(4999, 71);  //5ms,200HZ	//5ms��Ϊ����
	TIM2_PWM_Init(19999, 71); //20ms,50HZ
	TIM4_Cap_Init(19999, 71);
	BLDC_calibration(); //���У׼

	delay_ms(1800);
	USB_Port_Set(0); //USB�ȶϿ�
	delay_ms(700);
	USB_Port_Set(1); //USB�ٴ�����
	USB_Interrupts_Config();
	Set_USBClock();
	USB_Init();
	PID_Init(); //PID������ʼ��

	// Accel_six_Calibartion();
	// Mag_Calibartion();  //У׼ʹ����λ��ANTMAGУ׼�������ݵ�����ΪtxtУ׼
	// Gyro_Calibartion(); //������У׼
	// uart_init(500000);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //ʹ��ָ����TIM1�ж�,��������ж�

	while (1)
	{
		//��usb_endp.c  EP1_OUT_Callback  ���޸ĵģ����� fuck_USB_REC �����ѵ�
//#ifndef USE_443_data
		//���ʹ���������
		ANO_DT_Send_Senser(MPU_ACC.accx, MPU_ACC.accy, MPU_ACC.accz,
						   MPU_GYRO.gyrox, MPU_GYRO.gyroy, MPU_GYRO.gyroz,
						   AK8975_MAG.mx, AK8975_MAG.my, AK8975_MAG.mz, 0);
		//���ͷɿ�״̬
		ANO_DT_Send_Status(IMU.roll, IMU.pitch, IMU.yaw, 0, 0, 0);

		//����ң��������
		ANO_DT_Send_RCData(RC_data.thr, RC_data.yaw, RC_data.rol, RC_data.pit, 0, 0, 0, 0, 0, 0);

		//���͵������
		ANO_DT_Send_MotoPWM(MOTOR1 - 1000, MOTOR2 - 1000, MOTOR3 - 1000, MOTOR4 - 1000, 0, 0, 0, 0);
//#endif
		//����PID����
		if (PID_send_flag)
		{
			send_PID();
			PID_send_flag = 0;
		}

		if (PID_save_flag)
		{
			save_PID();
			PID_save_flag = 0;
		}
	}
}
