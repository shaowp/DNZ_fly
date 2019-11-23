#ifndef __DATA_PROCESSING_H
#define __DATA_PROCESSING_H

#include "kalman.h"
#include "mpu6050.h"
#include "AK8975.h"

void ACC_IMU_Filter_SOFT(void);						//软件低通滤波
void ACC_IMU_Filter_SOFT_INIT(float accUpdateRate); //软件低通滤波
void ACC_IMU_Filter(void);							//加速度计滤波
void GYRO_IMU_Filter(void);							//磁力计滤波
void MAG_IMU_Filter(void);							//磁力计滤波
void ACC_IMU_Filter_ButterWorth(void);				//加速度计，巴特沃斯滤波



typedef struct QueUE
{
	float *Array;  //指向数组
	int Front;	//指向头
	int Rear;	 //指向尾巴
	int Size;	 //已有数据大小
	int Capacity; //栈的容量
} Typedef_Queue;

//队列

void ACC_IMU_Filter_Queue_init(void);
void ACC_IMU_Filter_Queue(void);

int IsEmpty(Typedef_Queue *q);			 //判断是否空
int IsFull(Typedef_Queue *q);			 //判断是否满
Typedef_Queue *CreatQueue(int MaxCount); //创建
void MakeEmpty(Typedef_Queue *q);		 //使其空
void Enqueue(Typedef_Queue *q, float x);  //入队
void Dequeue(Typedef_Queue *q);			 //出队
float FrontAndDequeue(Typedef_Queue *q);  //出队，返回出队的元素
int Succ(int value, Typedef_Queue *q);
void ACC_IMU_Filter_ANO(void);//匿名滤波

#endif
