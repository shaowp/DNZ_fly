#ifndef __PID_H
#define __PID_H

#include "usart.h"
#include "global_variable.h" //���е�ȫ�ֱ���
#include "oled.h"


float pidupdate(pid_TypeDef *pid);
void USAT_GET_PID(void);
void PID_INIT();	//PID������ʼ��


#endif



