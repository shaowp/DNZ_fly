#ifndef _PID_H_
#define _PID_H_
#include "sys.h"
#define ABS(X) (((X) > 0) ? (X) : -(X))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define RateX_PID_kp 0
#define RateX_PID_ki 2
#define RateX_PID_kd 4

#define RateY_PID_kp 6
#define RateY_PID_ki 8
#define RateY_PID_kd 10

#define RateZ_PID_kp 12
#define RateZ_PID_ki 14
#define RateZ_PID_kd 16

#define Pitch_PID_kp 18
#define Pitch_PID_ki 20
#define Pitch_PID_kd 22

#define Roll_PID_kp 24
#define Roll_PID_ki 26
#define Roll_PID_kd 28

#define Yaw_PID_kp 30
#define Yaw_PID_ki 32
#define Yaw_PID_kd 34




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
void save_PID(void);
void read_PID(void);
void send_PID(void);

extern Struct_PID RateX_PID, RateY_PID, RateZ_PID; //内环
extern Struct_PID Pitch_PID, Roll_PID, Yaw_PID;	//外环
extern u8 PID_send_flag;
extern u8 PID_save_flag; //将上位机发来的数据存储到内存中

#endif
