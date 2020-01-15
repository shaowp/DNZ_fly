#ifndef _PID_H_
#define _PID_H_
#include "sys.h"
#include "data_process.h"

#define ABS(X) (((X) > 0) ? (X) : -(X))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define neihuan_Roll_PID_kp 0
#define neihuan_Roll_PID_ki 2
#define neihuan_Roll_PID_kd 4

#define neihuan_Pitch_PID_kp 6
#define neihuan_Pitch_PID_ki 8
#define neihuan_Pitch_PID_kd 10

#define RateZ_PID_kp 12
#define RateZ_PID_ki 14
#define RateZ_PID_kd 16

#define waihuan_Pitch_PID_kp 18
#define waihuan_Pitch_PID_ki 20
#define waihuan_Pitch_PID_kd 22

#define waihuan_Roll_PID_kp 24
#define waihuan_Roll_PID_ki 26
#define waihuan_Roll_PID_kd 28

#define Yaw_PID_kp 30
#define Yaw_PID_ki 32
#define Yaw_PID_kd 34

#define measured FeedBack
#define desired Expect

//PID结构体
typedef struct PID_object
{
	float Expect; //期望
	float Offset;
	float FeedBack;					//反馈值
	float Err;						//偏差
	float Last_Err;					//上次偏差
	float Err_Max;					//偏差限幅值
	float Integrate_Separation_Err; //积分分离偏差值
	float Integrate;				//积分值
	float Integrate_Max;			//积分限幅值
	float Kp;						//控制参数Kp
	float Ki;						//控制参数Ki
	float Kd;						//控制参数Kd
	float Control_OutPut;			//控制器总输出
	float Last_Control_OutPut;		//上次控制器总输出
	float Control_OutPut_Limit;		//输出限幅
	/***************************************/
	float Last_FeedBack;		//上次反馈值
	float Dis_Err;				//微分量
	float Dis_Error_History[5]; //历史微分量
	float Err_LPF;
	float Last_Err_LPF;
	float Dis_Err_LPF;
	u8 Err_Limit_Flag : 1;						 //偏差限幅标志
	u8 Integrate_Limit_Flag : 1;				 //积分限幅标志
	u8 Integrate_Separation_Flag : 1;			 //积分分离标志
	Butter_BufferData Control_Device_LPF_Buffer; //控制器低通输入输出缓冲

} Struct_PID;

typedef enum
{
	Pitch_Angle_Controler = 0, //外环PITCH
	Pitch_Gyro_Controler = 1,  //内环PITCH
	Roll_Angle_Controler = 2,  //外环ROLL
	Roll_Gyro_Controler = 3,   //内环ROLL
	Yaw_Angle_Controler = 4,   //外环YAW
	Yaw_Gyro_Controler = 5,	//内环YAW
} Controler_Label;

//所有PID的结构体
typedef struct
{
	//基础PID
	Struct_PID Pitch;
	Struct_PID RateX;
	Struct_PID Roll;
	Struct_PID RateY;
	Struct_PID Yaw;
	Struct_PID RateZ;

	// Struct_PID HeightAccel;
	// Struct_PID HeightRate;
	// Struct_PID HeightHigh;

	// Struct_PID GPS_Longitude_Acce;
	// Struct_PID GPS_Latitude_Acce;
	// Struct_PID GPS_Latitude_rate;
	// Struct_PID GPS_Longitude_rate;
	// Struct_PID GPS_Latitude_position;
	// Struct_PID GPS_Longitude_position;
} _st_pidData;

extern _st_pidData pidData;

//废弃的PID控制器
// float pidupdate_Angle(Struct_PID *pid);
// float pidupdate_Rate(Struct_PID *pid);

void PID_Init(Struct_PID *Controler, Controler_Label Label); //PID参数初始化	from ZINA
void save_PID(void);
void read_PID(void);
void send_PID(void);
void PID_INIT_ALL(void);				  //from ZINA
float PID_Control(Struct_PID *Controler); //PID控制器	//from ZINA
float PID_Control_Div_LPF(Struct_PID *Controler);
float PID_Control_Yaw(Struct_PID *Controler);
extern Struct_PID neihuan_Roll_PID, neihuan_Pitch_PID, RateZ_PID; //内环
extern Struct_PID waihuan_Pitch_PID, waihuan_Roll_PID, Yaw_PID;   //外环
extern u8 PID_send_flag;
extern u8 PID_save_flag; //将上位机发来的数据存储到内存中

#endif
