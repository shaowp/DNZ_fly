#include "pid.h"



Struct_PID RateX_PID, RateY_PID, RateZ_PID; //内环
Struct_PID Pitch_PID, Roll_PID, Yaw_PID;	//外环

u8 PID_send_flag;


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
}
