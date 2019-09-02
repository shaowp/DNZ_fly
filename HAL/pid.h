#ifndef _PID_H_
#define _PID_H_
#define ABS(X) (((X) > 0) ? (X) : -(X))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//PID结构体
typedef struct PID_object
{

	float Kp;
	float Ki;
	float Kd;
	float measured;
	float desired;
	float error;
	float lasterror;
	float out;
	float integ;
	float IntegLimitLow;  //积分限幅
	float IntegLimitHigh; //积分限幅
	float Err_Max;		  //误差限幅
} Struct_PID;

float pidupdate_Angle(Struct_PID *pid);
float pidupdate_Rate(Struct_PID *pid);
void PID_Init(void); //PID参数初始化

extern Struct_PID RateX_PID, RateY_PID, RateZ_PID; //内环
extern Struct_PID Pitch_PID, Roll_PID, Yaw_PID;	//外环

#endif
