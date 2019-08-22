#include "softiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////
/********************************************	 
			软件IIC程序								  
			实现对6050，8975以及24c02的操作的底层函数
			版权不归我所有
			请勿商用，盗版不究
			@shaowp
*********************************************/
//////////////////////////////////////////////////////////////////////////////////

//MPU IIC 延时函数
void SOFT_IIC_Delay(void)
{
	delay_us(2);
}

//初始化IIC
void SOFT_IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//先使能外设IO PORTB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO
	GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);			 //PB10,PB11 输出高
}
//产生IIC起始信号
void SOFT_IIC_Start(void)
{
	SOFT_SDA_OUT(); //sda线输出
	SOFT_IIC_SDA = 1;
	SOFT_IIC_SCL = 1;
	SOFT_IIC_Delay();
	SOFT_IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void SOFT_IIC_Stop(void)
{
	SOFT_SDA_OUT(); //sda线输出
	SOFT_IIC_SCL = 0;
	SOFT_IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
	SOFT_IIC_Delay();
	SOFT_IIC_SCL = 1;
	SOFT_IIC_SDA = 1; //发送I2C总线结束信号
	SOFT_IIC_Delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 SOFT_IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SOFT_SDA_IN(); //SDA设置为输入
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
	SOFT_IIC_SCL = 0; //时钟输出0
	return 0;
}
//产生ACK应答
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
//不产生ACK应答
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void SOFT_IIC_Send_Byte(u8 txd)
{
	u8 t;
	SOFT_SDA_OUT();
	SOFT_IIC_SCL = 0; //拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 SOFT_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SOFT_SDA_IN(); //SDA设置为输入
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
		SOFT_IIC_NAck(); //发送nACK
	else
		SOFT_IIC_Ack(); //发送ACK
	return receive;
}
