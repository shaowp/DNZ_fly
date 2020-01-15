#include "control.h"
#include "pid.h"
#include "IMU.h"
#include "mpu6050.h"
#include "rc_process.h"
#include "led.h"
// #define MOTOR4 TIM2->CCR1 //对应飞控板PWM_OUT4
// #define MOTOR3 TIM2->CCR2 //对应飞控板PWM_OUT3
// #define MOTOR2 TIM2->CCR3 //对应飞控板PWM_OUT2
// #define MOTOR1 TIM2->CCR4 //对应飞控板PWM_OUT1

u8 LOCK_STATUS = 0;

short MOTOR1, MOTOR2, MOTOR3, MOTOR4; //四个电机

//电调校准
void BLDC_calibration(void)
{
	TIM2->CCR1 = 2000;
	TIM2->CCR2 = 2000;
	TIM2->CCR3 = 2000;
	TIM2->CCR4 = 2000;
	delay_ms(3000);
	TIM2->CCR1 = 1100;
	TIM2->CCR2 = 1100;
	TIM2->CCR3 = 1100;
	TIM2->CCR4 = 1100;
	delay_ms(1000);
}

void AttitudePidControl() //姿态PID控制
{

	//--------------------姿态控制-----------------------------------------
	pidData.RateX.measured = MPU_GYRO.gyrox * Gyro_G; //内环测量值 角度/秒
	pidData.RateY.measured = MPU_GYRO.gyroy * Gyro_G;
	pidData.RateZ.measured = MPU_GYRO.gyroz * Gyro_G;

	pidData.Pitch.measured = IMU.pitch; //外环测量值 单位：角度
	pidData.Roll.measured = IMU.roll;
	pidData.Yaw.measured =  IMU.yaw;

//遥控器数据处理 这里暂时不处理遥控器，直接赋值为0
// pidData.Pitch.desired = 0;
// pidData.Yaw.desired = 0;
// pidData.Roll.desired = 0;
#define DEADBAND 40

	pidData.Pitch.desired = (1500.0 - RC_data.pit) * 0.06;
	pidData.Roll.desired = -(1500.0 - RC_data.rol) * 0.06;
	const float pitch_roll_remote_ratio = 0.06f;
	// if (RC_data.pit > (1500 + DEADBAND))
	// {
	// 	pidData.Pitch.desired = -(RC_data.pit - (1500 + DEADBAND)) * pitch_roll_remote_ratio;
	// }
	// else if (RC_data.pit < (1500 - DEADBAND))
	// {
	// 	pidData.Pitch.desired = -(RC_data.pit - (1500 - DEADBAND)) * pitch_roll_remote_ratio;
	// }
	// else
	// 	pidData.Pitch.desired = 0;
	// if (RC_data.rol > (1500 + DEADBAND))
	// {
	// 	pidData.Roll.desired = -(RC_data.rol - (1500 + DEADBAND)) * pitch_roll_remote_ratio;
	// }
	// else if (RC_data.rol < (1500 - DEADBAND))
	// {
	// 	pidData.Roll.desired = -(RC_data.rol - (1500 - DEADBAND)) * pitch_roll_remote_ratio;
	// }
	// else
	// 	pidData.Roll.desired = 0;

	//--------------------横滚角控制----------------------------------------- roll
	PID_Control(&pidData.Roll);							 //调用PID处理函数来处理外环	横滚角PID
	pidData.RateX.desired = pidData.Roll.Control_OutPut; //将外环的PID输出作为内环PID的期望值即为串级PID
	PID_Control(&pidData.RateX);						 //再调用内环
	//--------------------俯仰角控制-----------------------------------------
	PID_Control(&pidData.Pitch); //调用PID处理函数来处理外环	俯仰角PID
	pidData.RateY.desired = pidData.Pitch.Control_OutPut;
	PID_Control_Div_LPF(&pidData.RateY); //再调用内环

	//--------------------偏航角控制-----------------------------------------
	static uint8_t lock_yaw_flag = 0;
	if (RC_data.yaw < 1700 && RC_data.yaw > 1300) //偏航杆置于中位
	{
		static float yaw_lock_value;
		if (lock_yaw_flag == 0)
		{
			lock_yaw_flag = 1;
			yaw_lock_value = pidData.Yaw.measured;
		}
		pidData.Yaw.Expect = yaw_lock_value;
		PID_Control_Yaw(&pidData.Yaw);					   //偏航角度控制
		pidData.RateZ.Expect = pidData.Yaw.Control_OutPut; //偏航角速度环期望，来源于偏航角度控制器输出
	}
	else //波动偏航方向杆后，只进行内环角速度控制
	{
		lock_yaw_flag = 0;
		pidData.Yaw.Expect = 0;			
		//PID_Control_Yaw(&pidData.Yaw);//偏航角度控制					 //偏航角期望给0,不进行角度控制
		pidData.RateZ.Expect = -(1500.0 - RC_data.yaw) * 0.15f; //偏航角速度环期望，直接来源于遥控器打杆量
	}
	PID_Control_Div_LPF(&pidData.RateZ);
	pidData.RateZ.Control_OutPut = pidData.RateZ.Control_OutPut;
	// if (pidData.RateZ.Control_OutPut >= pidData.Yaw.Control_OutPut_Limit)
	// 	pidData.RateZ.Control_OutPut = pidData.Yaw.Control_OutPut_Limit;
	// if (pidData.RateZ.Control_OutPut <= -pidData.Yaw.Control_OutPut_Limit)
	// 	pidData.RateZ.Control_OutPut = -pidData.Yaw.Control_OutPut_Limit;
	//内环测量值
	//	neihuan_Roll_PID.measured = MPU_GYRO.gyrox / 16.3825;
	//	neihuan_Pitch_PID.measured = MPU_GYRO.gyroy / 16.3825;
	//	// neihuan_Yaw_PID.measured = MPU_GYRO.gyroz / 16.3825 * 0.0174532;

	//	//外环测量值
	//	waihuan_Pitch_PID.measured = IMU.pitch;
	//	waihuan_Roll_PID.measured = IMU.roll;
	//	// waihuan_Yaw_PID.measured = IMU.yaw;

	//	//外环期望值  期望的角度
	//	waihuan_Pitch_PID.desired = 0;
	//	waihuan_Roll_PID.desired = 0;

	//先算外环，外环角度控制
	//先关闭外环
	// pidupdate_Angle(&waihuan_Pitch_PID);
	//	pidupdate_Angle(&waihuan_Roll_PID);

	//内环的输入为外环的输出
	//	neihuan_Roll_PID.desired =  waihuan_Roll_PID.out;
	// neihuan_Pitch_PID.desired = waihuan_Pitch_PID.out;

	//内环的输入改为遥控器的舵量
	// #define DEADBAND 40
	// 	const float pitch_roll_remote_ratio = 0.01f;
	// 	if (RC_data.rol > (1500 + DEADBAND))
	// 	{
	// 		neihuan_Roll_PID.desired = -((s32)RC_data.rol - (1500 + DEADBAND)) * pitch_roll_remote_ratio;
	// 	}
	// 	else if (RC_data.rol < (1500 - DEADBAND))
	// 	{
	// 		neihuan_Roll_PID.desired = -((s32)RC_data.rol - (1500 - DEADBAND)) * pitch_roll_remote_ratio;
	// 	}
	// 	else
	// 		neihuan_Roll_PID.desired = 0;
	//内环角速度控制
	//pidupdate_Rate(&neihuan_Pitch_PID);
	// pidupdate_Rate(&neihuan_Roll_PID);

	//内环的输出就是电机的输出
	MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT((RC_data.thr - 1000), 0, 900);
	//RC_data.thr-1000;
	//先调ROLL
	// MOTOR2= RC_data.thr;
	// MOTOR1 += (int16_t)(+neihuan_Pitch_PID.out + neihuan_Roll_PID.out - RateZ_PID.out); //M1
	// MOTOR2 += (int16_t)(+neihuan_Pitch_PID.out - neihuan_Roll_PID.out + RateZ_PID.out); //M2
	// MOTOR3 += (int16_t)(-neihuan_Pitch_PID.out - neihuan_Roll_PID.out - RateZ_PID.out); //M3
	// MOTOR4 += (int16_t)(-neihuan_Pitch_PID.out + neihuan_Roll_PID.out + RateZ_PID.out); //M4

	MOTOR1 += (-pidData.RateY.Control_OutPut + pidData.RateX.Control_OutPut - pidData.RateZ.Control_OutPut); //M1
	MOTOR2 += (-pidData.RateY.Control_OutPut - pidData.RateX.Control_OutPut + pidData.RateZ.Control_OutPut); //M2
	MOTOR3 += (+pidData.RateY.Control_OutPut - pidData.RateX.Control_OutPut - pidData.RateZ.Control_OutPut); //M3
	MOTOR4 += (+pidData.RateY.Control_OutPut + pidData.RateX.Control_OutPut + pidData.RateZ.Control_OutPut); //M4
	MOTOR1 = LIMIT(MOTOR1, 0, 1000);																		 //不管任何情况，输出都严格限制为电调所能接收的范围
	MOTOR2 = LIMIT(MOTOR2, 0, 1000);
	MOTOR3 = LIMIT(MOTOR3, 0, 1000);
	MOTOR4 = LIMIT(MOTOR4, 0, 1000);
}

void MotorControl() //电机输出
{
	if (LOCK_STATUS == 1)
	{
		TIM2->CCR1 = MOTOR1 * 1 + 1000;
		TIM2->CCR2 = MOTOR2 * 1 + 1000;
		TIM2->CCR3 = MOTOR3 * 1 + 1000;
		TIM2->CCR4 = MOTOR4 * 1 + 1000;
	}
	else if (LOCK_STATUS == 0)
	{
		TIM2->CCR1 = 1050;
		TIM2->CCR2 = 1050;
		TIM2->CCR3 = 1050;
		TIM2->CCR4 = 1050;
	}
}

void Unlock_check(void) //检测是否上锁或者解锁
{
	static u8 lock_temp_num = 0; //计数，5ms调用一次，超过1s就锁上或者开机
	if (RC_data.thr <= 1150 && RC_data.yaw >= 1900 && RC_data.pit <= 1150 && RC_data.rol <= 1150)
	{
		lock_temp_num++;
	}
	if (lock_temp_num >= 200)
	{
		lock_temp_num = 0;
		LOCK_STATUS = !LOCK_STATUS;
	}
	if (LOCK_STATUS == 1)
		LED1 = 0;
	else if (LOCK_STATUS == 0)
		LED1 = 1;
}
