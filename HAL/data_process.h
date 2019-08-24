#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "kalman.h"
#include "mpu6050.h"
#include "AK8975.h"

void ACC_IMU_Filter(void);  //加速度计滤波
void GYRO_IMU_Filter(void); //磁力计滤波
void MAG_IMU_Filter(void);  //磁力计滤波
#endif
