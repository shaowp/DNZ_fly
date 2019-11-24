#include "pid.h"
#include "stflash.h"
#include "ANO_USART.h"
Struct_PID RateX_PID, RateY_PID, RateZ_PID; //内环
Struct_PID Pitch_PID, Roll_PID, Yaw_PID;	//外环

u8 PID_send_flag = 0; //向上位机发送PID数据
u8 PID_save_flag = 0; //将上位机发来的数据存储到内存中

//PID计算，分为内环和外环的计算
//9月2日：为了完成原型，使用分开编写内外环代码

//外环角度环
float pidupdate_Angle(Struct_PID *pid)
{
	//误差计算：期望值-测量值
	pid->error = pid->desired - pid->measured;

	//偏差限幅
	if (pid->error >= pid->Err_Max)
		pid->error = pid->Err_Max;
	if (pid->error <= -pid->Err_Max)
		pid->error = -pid->Err_Max;

	//积分计算
	pid->integ += pid->Ki * pid->error;
	//积分限幅
	pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);

	pid->out = pid->Kp * pid->error +
			   pid->Kd * (pid->error - pid->lasterror) +
			   pid->integ;
	pid->lasterror = pid->error;
	return pid->out;
}

//内环角速度环
float pidupdate_Rate(Struct_PID *pid)
{
	//误差计算：期望值-测量值
	pid->error = pid->desired - pid->measured;

	//偏差限幅
	if (pid->error >= pid->Err_Max)
		pid->error = pid->Err_Max;
	if (pid->error <= -pid->Err_Max)
		pid->error = -pid->Err_Max;

	//积分计算
	pid->integ += pid->Ki * pid->error;
	//积分限幅
	pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);

	pid->out = pid->Kp * pid->error +
			   pid->Kd * (pid->error - pid->lasterror) +
			   pid->integ;
	pid->lasterror = pid->error;
	return pid->out;
}

void PID_Init() //PID参数初始化
{

	/*************内环*******************/
	//参数随便写的
	RateX_PID.Kp = 0.1;
	RateX_PID.Ki = 0.2;
	RateX_PID.Kd = 0.3;
	RateX_PID.Err_Max = 30;
	RateX_PID.IntegLimitHigh = 20;
	RateX_PID.IntegLimitLow = -20;

	RateY_PID.Kp = 0.4;
	RateY_PID.Ki = 0.5;
	RateY_PID.Kd = 0.6;
	RateY_PID.Err_Max = 30;
	RateY_PID.IntegLimitHigh = 20;
	RateY_PID.IntegLimitLow = -20;

	RateZ_PID.Kp = 0.7;
	RateZ_PID.Ki = 0.7;
	RateZ_PID.Kd = 0.9;
	RateZ_PID.Err_Max = 30;
	RateZ_PID.IntegLimitHigh = 20;
	RateZ_PID.IntegLimitLow = -20;
	/////////////////////////////////////

	/*************外环*******************/
	Pitch_PID.Kp = 0;
	Pitch_PID.Ki = 0;
	Pitch_PID.Ki = 0;
	Pitch_PID.Kd = 0;
	Pitch_PID.Err_Max = 30;
	Pitch_PID.IntegLimitHigh = 20;
	Pitch_PID.IntegLimitLow = -20;

	Roll_PID.Kp = 0;
	Roll_PID.Ki = 0;
	Roll_PID.Kd = 0;
	Roll_PID.Err_Max = 30;
	Roll_PID.IntegLimitHigh = 20;
	RateY_PID.IntegLimitLow = -20;

	Yaw_PID.Kp = 0;
	Yaw_PID.Ki = 0;
	Yaw_PID.Kd = 0;
	Yaw_PID.Err_Max = 30;
	Yaw_PID.IntegLimitHigh = 20;
	Yaw_PID.IntegLimitLow = -20;

	read_PID();
}

void send_PID(void)
{
	//read_PID();	//如果有AT24C02则使用
	ANO_DT_Send_PID(1, RateX_PID.Kp, RateX_PID.Ki, RateX_PID.Kd,
					RateY_PID.Kp, RateY_PID.Ki, RateY_PID.Kd,
					RateZ_PID.Kp, RateZ_PID.Ki, RateZ_PID.Kd);
	//delay_ms(2);
	ANO_DT_Send_PID(2, Roll_PID.Kp, Roll_PID.Ki, Roll_PID.Kd,
					Pitch_PID.Kp, Pitch_PID.Ki, Pitch_PID.Kd,
					Yaw_PID.Kp, Yaw_PID.Ki, Yaw_PID.Kd);
	//delay_ms(2);
	PID_send_flag = 0;
}

void save_PID(void)
{
	//存数据的时候关掉主中断

	//写入的参数为  地址   数据   数据长度（几个字节）
	//内环数据的写入
	if (PID_save_flag == 1)
	{
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		AT24CXX_WriteLenByte(RateX_PID_kp, (u16)(RateX_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(RateX_PID_ki, (u16)(RateX_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(RateX_PID_kd, (u16)(RateX_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(RateY_PID_kp, (u16)(RateY_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(RateY_PID_ki, (u16)(RateY_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(RateY_PID_kd, (u16)(RateY_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(RateZ_PID_kp, (u16)(RateZ_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(RateZ_PID_ki, (u16)(RateZ_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(RateZ_PID_kd, (u16)(RateZ_PID.Kd * 1000), 2);

		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
	}
	if (PID_save_flag == 2) //外环
	{
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		AT24CXX_WriteLenByte(Pitch_PID_kp, (u16)(Pitch_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(Pitch_PID_ki, (u16)(Pitch_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(Pitch_PID_kd, (u16)(Pitch_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(Roll_PID_kp, (u16)(Roll_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(Roll_PID_ki, (u16)(Roll_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(Roll_PID_kd, (u16)(Roll_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(Yaw_PID_kp, (u16)(Yaw_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(Yaw_PID_ki, (u16)(Yaw_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(Yaw_PID_kd, (u16)(Yaw_PID.Kd * 1000), 2);
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
	}
	// if (PID_save_flag == 5)
	// {
	// 	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断

	// 	AT24CXX_WriteLenByte(Pitch_PID_kp, (u16)(Pitch_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(Pitch_PID_ki, (u16)(Pitch_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(Pitch_PID_kd, (u16)(Pitch_PID.Kd * 1000), 2);

	// 	AT24CXX_WriteLenByte(Roll_PID_kp, (u16)(Roll_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(Roll_PID_ki, (u16)(Roll_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(Roll_PID_kd, (u16)(Roll_PID.Kd * 1000), 2);

	// 	AT24CXX_WriteLenByte(Yaw_PID_kp, (u16)(Yaw_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(Yaw_PID_ki, (u16)(Yaw_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(Yaw_PID_kd, (u16)(Yaw_PID.Kd * 1000), 2);

	// 	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //使能指定的TIM1中断,允许更新中断
	// }
	PID_save_flag = 0;
}

void read_PID(void)
{
	//读取的   地址       数据的长度
	//读取内环数据
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
	RateX_PID.Kp = AT24CXX_ReadLenByte(RateX_PID_kp, 2) / 1000.0;
	RateX_PID.Ki = AT24CXX_ReadLenByte(RateX_PID_ki, 2) / 1000.0;
	RateX_PID.Kd = AT24CXX_ReadLenByte(RateX_PID_kd, 2) / 1000.0;
	RateY_PID.Kp = AT24CXX_ReadLenByte(RateY_PID_kp, 2) / 1000.0;
	RateY_PID.Ki = AT24CXX_ReadLenByte(RateY_PID_ki, 2) / 1000.0;
	RateY_PID.Kd = AT24CXX_ReadLenByte(RateY_PID_kd, 2) / 1000.0;
	RateZ_PID.Kp = AT24CXX_ReadLenByte(RateZ_PID_kp, 2) / 1000.0;
	RateZ_PID.Ki = AT24CXX_ReadLenByte(RateZ_PID_ki, 2) / 1000.0;
	RateZ_PID.Kd = AT24CXX_ReadLenByte(RateZ_PID_kd, 2) / 1000.0;

	// // //读取外环数据
	Pitch_PID.Kp = AT24CXX_ReadLenByte(Pitch_PID_kp, 2) / 1000.0;
	Pitch_PID.Ki = AT24CXX_ReadLenByte(Pitch_PID_ki, 2) / 1000.0;
	Pitch_PID.Kd = AT24CXX_ReadLenByte(Pitch_PID_kd, 2) / 1000.0;

	Roll_PID.Kp = AT24CXX_ReadLenByte(Roll_PID_kp, 2) / 1000.0;
	Roll_PID.Ki = AT24CXX_ReadLenByte(Roll_PID_ki, 2) / 1000.0;
	Roll_PID.Kd = AT24CXX_ReadLenByte(Roll_PID_kd, 2) / 1000.0;

	Yaw_PID.Kp = AT24CXX_ReadLenByte(Yaw_PID_kp, 2) / 1000.0;
	Yaw_PID.Ki = AT24CXX_ReadLenByte(Yaw_PID_ki, 2) / 1000.0;
	Yaw_PID.Kd = AT24CXX_ReadLenByte(Yaw_PID_kd, 2) / 1000.0;
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
}
