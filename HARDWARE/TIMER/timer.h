#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"
#include "stm32f10x.h"

extern u32 tempup1; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup2; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup3; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup4; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup5; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup6; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup7; //�����ܸߵ�ƽ��ʱ��
extern u32 tempup8; //�����ܸߵ�ƽ��ʱ��

void TIM2_PWM_Init(u16 arr, u16 psc); //PWM�����
void TIM4_Cap_Init(u16 arr, u16 psc); //���벶��
void TIM1_Int_Init(u16 arr, u16 psc); //��ʱ��1�ļ�ʱ������Ҫ�Ķ�ʱ������
#endif
