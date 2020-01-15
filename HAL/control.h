#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
void AttitudePidControl(void); //姿态PID控制
void MotorControl(void);	   //电机输出
void BLDC_calibration(void);   //点调校准
void Unlock_check(void);	//检测是否上锁或者解锁

extern u8 LOCK_STATUS;	//解锁标志位，-1为上锁，1为解锁
extern short MOTOR1, MOTOR2, MOTOR3, MOTOR4; //四个电机
#endif
