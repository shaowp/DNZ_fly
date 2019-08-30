#ifndef __IMU_H
#define	__IMU_H
#include  <stdint.h>
#include "mpu6050.h"
#include <math.h>
#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005326

#define FILTER_ACC_NUM 10
 
float Q_rsqrt(float number);
void Collect_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Erjiehubu(float gx, float gy, float gz, float ax, float ay, float az);

typedef struct
{
	float yaw;
	float pitch;
	float roll;
	float yaw_mag; //单独由磁力计的出来的角度
	float Cos_Roll;
	float Sin_Roll;
	float Cos_Pitch;
	float Sin_Pitch;
	float Cos_Yaw;
	float Sin_Yaw;
} Struct_IMU;
extern Struct_IMU IMU;


#endif













