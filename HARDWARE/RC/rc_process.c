#include "rc_process.h"

/***************************************************/
//RC遥控器四通道的中断处理/////
//遥控器捕获使用的是TIME4
u8 TIM4CH1_CAPTURE_STA = 0; //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH1_CAPTURE_UPVAL;
u16 TIM4CH1_CAPTURE_DOWNVAL;

u8 TIM4CH2_CAPTURE_STA = 0; //通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH2_CAPTURE_UPVAL;
u16 TIM4CH2_CAPTURE_DOWNVAL;

u8 TIM4CH3_CAPTURE_STA = 0; //通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH3_CAPTURE_UPVAL;
u16 TIM4CH3_CAPTURE_DOWNVAL;

u8 TIM4CH4_CAPTURE_STA = 0; //通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志
u16 TIM4CH4_CAPTURE_UPVAL;
u16 TIM4CH4_CAPTURE_DOWNVAL;

u32 tim4_T1;
u32 tim4_T2;
u32 tim4_T3;
u32 tim4_T4;

// int pwmout1, pwmout2, pwmout3, pwmout4; //输出占空比
// int pwmout5, pwmout6, pwmout7, pwmout8; //输出占空比

//最终的遥控器的数据
//后期使用结构体
// u32 tempup1 = 0; //捕获总高电平的时间
// u32 tempup2 = 0; //捕获总高电平的时间
// u32 tempup3 = 0; //捕获总高电平的时间
// u32 tempup4 = 0; //捕获总高电平的时间
// u32 tempup5 = 0; //捕获总高电平的时间
// u32 tempup6 = 0; //捕获总高电平的时间
// u32 tempup7 = 0; //捕获总高电平的时间
// u32 tempup8 = 0; //捕获总高电平的时间

Struct_RC_DATA RC_data;

//定时器4中断服务程序
//捕获遥控器的高电平信号
void TIM4_IRQHandler(void)
{
	if ((TIM4CH1_CAPTURE_STA & 0X80) == 0) //还未成功捕获
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET) //捕获1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC1); //清除中断标志位
			if (TIM4CH1_CAPTURE_STA & 0X40)			 //捕获到一个下降沿
			{
				TIM4CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM4); //记录下此时的定时器计数值
				if (TIM4CH1_CAPTURE_DOWNVAL < TIM4CH1_CAPTURE_UPVAL)
				{
					tim4_T1 = 19999;
				}
				else
					tim4_T1 = 0;
				RC_data.rol = TIM4CH1_CAPTURE_DOWNVAL - TIM4CH1_CAPTURE_UPVAL + tim4_T1; //得到总的高电平的时间
				// pwmout1 = tempup1;													 //总的高电平的时间
				//RC_receive_data.yaw=pwmout1;
				TIM4CH1_CAPTURE_STA = 0;							//捕获标志位清零
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM4);		 //获取上升沿数据
				TIM4CH1_CAPTURE_STA |= 0X40;						 //标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling); //设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH2_CAPTURE_STA & 0X80) == 0) //还未成功捕获
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET) //捕获2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC2); //清除中断标志位
			if (TIM4CH2_CAPTURE_STA & 0X40)			 //捕获到一个下降沿
			{
				TIM4CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM4); //记录下此时的定时器计数值
				if (TIM4CH2_CAPTURE_DOWNVAL < TIM4CH2_CAPTURE_UPVAL)
				{
					tim4_T2 = 19999;
				}
				else
					tim4_T2 = 0;
				RC_data.pit = TIM4CH2_CAPTURE_DOWNVAL - TIM4CH2_CAPTURE_UPVAL + tim4_T2; //得到总的高电平的时间
				// pwmout2 = tempup2;													 //总的高电平的时间
				TIM4CH2_CAPTURE_STA = 0;							//捕获标志位清零
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM4);		 //获取上升沿数据
				TIM4CH2_CAPTURE_STA |= 0X40;						 //标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling); //设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH3_CAPTURE_STA & 0X80) == 0) //还未成功捕获
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) //捕获3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC3); //清除中断标志位
			if (TIM4CH3_CAPTURE_STA & 0X40)			 //捕获到一个下降沿
			{
				TIM4CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM4); //记录下此时的定时器计数值
				if (TIM4CH3_CAPTURE_DOWNVAL < TIM4CH3_CAPTURE_UPVAL)
				{
					tim4_T3 = 19999;
				}
				else
					tim4_T3 = 0;
				RC_data.thr = TIM4CH3_CAPTURE_DOWNVAL - TIM4CH3_CAPTURE_UPVAL + tim4_T3; //得到总的高电平的时间
				// pwmout3 = tempup3;													 //总的高电平的时间
				TIM4CH3_CAPTURE_STA = 0;							//捕获标志位清零
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM4);		 //获取上升沿数据
				TIM4CH3_CAPTURE_STA |= 0X40;						 //标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling); //设置为下降沿捕获
			}
		}
	}

	if ((TIM4CH4_CAPTURE_STA & 0X80) == 0) //还未成功捕获
	{
		if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET) //捕获4发生捕获事件
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_CC4); //清除中断标志位
			if (TIM4CH4_CAPTURE_STA & 0X40)			 //捕获到一个下降沿
			{
				TIM4CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM4); //记录下此时的定时器计数值
				if (TIM4CH4_CAPTURE_DOWNVAL < TIM4CH4_CAPTURE_UPVAL)
				{
					tim4_T4 = 19999;
				}
				else
					tim4_T4 = 0;
				RC_data.yaw = TIM4CH4_CAPTURE_DOWNVAL - TIM4CH4_CAPTURE_UPVAL + tim4_T4; //得到总的高电平的时间
				// pwmout4 = tempup4;													 //总的高电平的时间
				TIM4CH4_CAPTURE_STA = 0;							//捕获标志位清零
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising); //设置为上升沿捕获
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM4CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM4);		 //获取上升沿数据
				TIM4CH4_CAPTURE_STA |= 0X40;						 //标记已捕获到上升沿
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling); //设置为下降沿捕获
			}
		}
	}
}
