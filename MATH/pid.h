#ifndef __PID_H
#define __PID_H

#include "usart.h"
#include "global_variable.h" //所有的全局变量
#include "oled.h"


float pidupdate(pid_TypeDef *pid);
void USAT_GET_PID(void);
void PID_INIT();	//PID参数初始化


#endif



