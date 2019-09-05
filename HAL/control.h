#ifndef __CONTROL_H
#define __CONTROL_H

void AttitudePidControl(void); //姿态PID控制
void MotorControl(void);	   //电机输出
void BLDC_calibration(void);   //点调校准

extern short MOTOR1, MOTOR2, MOTOR3, MOTOR4; //四个电机
#endif
