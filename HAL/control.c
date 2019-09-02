#include "control.h"
#include "pid.h"
#include "IMU.h"
#include "mpu6050.h"

#define MOTOR1 TIM2->CCR1 //对应飞控板PWM_OUT1
#define MOTOR2 TIM2->CCR2 //对应飞控板PWM_OUT2
#define MOTOR3 TIM2->CCR3 //对应飞控板PWM_OUT3
#define MOTOR4 TIM2->CCR4 //对应飞控板PWM_OUT4

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

	//YAWd单独控制，确实不好搞

	//外环角度环的q期望值为遥控器数据，这里为 0，方便测试
	Pitch_PID.desired = 0;
	Pitch_PID.measured = IMU.pitch;
	Roll_PID.desired = 0;
	Roll_PID.measured = IMU.roll;

	//外环角度控制
	pidupdate_Angle(&Pitch_PID);
	pidupdate_Angle(&Roll_PID);

	RateX_PID.desired = Roll_PID.out;
	RateX_PID.measured = MPU_GYRO.gyrox / 16.3825 * 0.0174532;
	RateY_PID.desired = Pitch_PID.out;
	RateY_PID.measured = MPU_GYRO.gyroy / 16.3825 * 0.0174532;

	//内环角速度控制
	pidupdate_Rate(&Pitch_PID);
	pidupdate_Rate(&Roll_PID);

	//YAW的单独控制
	// Yaw_PID.desired = 0; //YAW暂时不加
	// Yaw_PID.measured = IMU.yaw;
	// pidupdate_Angle(&Yaw_PID);
	// RateZ_PID.desired = Yaw_PID.out;
	// RateZ_PID.measured = MPU_GYRO.gyroz / 16.3825 * 0.0174532;
	// pidupdate_Rate(&Yaw_PID);
}
void MotorControl() //电机输出
{
}
