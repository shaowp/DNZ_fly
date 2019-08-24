#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "stm32f10x.h"

extern u32 tempup1; //捕获总高电平的时间
extern u32 tempup2; //捕获总高电平的时间
extern u32 tempup3; //捕获总高电平的时间
extern u32 tempup4; //捕获总高电平的时间
extern u32 tempup5; //捕获总高电平的时间
extern u32 tempup6; //捕获总高电平的时间
extern u32 tempup7; //捕获总高电平的时间
extern u32 tempup8; //捕获总高电平的时间

void TIM2_PWM_Init(u16 arr, u16 psc); //PWM的输出
void TIM4_Cap_Init(u16 arr, u16 psc); //输入捕获
void TIM1_Int_Init(u16 arr, u16 psc); //定时器1的计时任务，主要的定时器任务
#endif
