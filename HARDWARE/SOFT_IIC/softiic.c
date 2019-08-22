#include "softiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
/********************************************	 
			���IIC����								  
			ʵ�ֶ�6050��8975�Լ�24c02�Ĳ����ĵײ㺯��
			��Ȩ����������
			�������ã����治��
			@shaowp
*********************************************/
//////////////////////////////////////////////////////////////////////////////////

//MPU IIC ��ʱ����
void SOFT_IIC_Delay(void)
{
	delay_us(2);
}

//��ʼ��IIC
void SOFT_IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//��ʹ������IO PORTBʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);			 //PB10,PB11 �����
}
//����IIC��ʼ�ź�
void SOFT_IIC_Start(void)
{
	SOFT_SDA_OUT(); //sda�����
	SOFT_IIC_SDA = 1;
	SOFT_IIC_SCL = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void SOFT_IIC_Stop(void)
{
	SOFT_SDA_OUT(); //sda�����
	SOFT_IIC_SCL = 0;
	SOFT_IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 1;
	SOFT_IIC_SDA = 1; //����I2C���߽����ź�
	SOFT_IIC_Delay();
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 SOFT_IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SOFT_SDA_IN(); //SDA����Ϊ����
	SOFT_IIC_SDA = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 1;
	SOFT_IIC_Delay();
	while (SOFT_READ_SDA)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			SOFT_IIC_Stop();
			return 1;
		}
	}
	SOFT_IIC_SCL = 0; //ʱ�����0
	return 0;
}
//����ACKӦ��
void SOFT_IIC_Ack(void)
{
	SOFT_IIC_SCL = 0;
	SOFT_SDA_OUT();
	SOFT_IIC_SDA = 0;
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 0;
}
//������ACKӦ��
void SOFT_IIC_NAck(void)
{
	SOFT_IIC_SCL = 0;
	SOFT_SDA_OUT();
	SOFT_IIC_SDA = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 0;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void SOFT_IIC_Send_Byte(u8 txd)
{
	u8 t;
	SOFT_SDA_OUT();
	SOFT_IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	for (t = 0; t < 8; t++)
	{
		SOFT_IIC_SDA = (txd & 0x80) >> 7;
		txd <<= 1;
		SOFT_IIC_SCL = 1;
		SOFT_IIC_Delay();
		SOFT_IIC_SCL = 0;
		SOFT_IIC_Delay();
	}
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 SOFT_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SOFT_SDA_IN(); //SDA����Ϊ����
	for (i = 0; i < 8; i++)
	{
		SOFT_IIC_SCL = 0;
		SOFT_IIC_Delay();
		SOFT_IIC_SCL = 1;
		receive <<= 1;
		if (SOFT_READ_SDA)
			receive++;
		SOFT_IIC_Delay();
	}
	if (!ack)
		SOFT_IIC_NAck(); //����nACK
	else
		SOFT_IIC_Ack(); //����ACK
	return receive;
}
