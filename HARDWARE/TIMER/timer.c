#include "timer.h"

//主程序的中断设置
//优先级不能太高，不然遥控器，串口接收等会有问题，因为这个频率高
void TIM1_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能

	//定时器TIM1初始化
	TIM_TimeBaseStructure.TIM_Period = arr;						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	TIM_Cmd(TIM1, ENABLE);							//使能TIMx
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;		  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);							  //初始化NVIC寄存器

	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
}

//遥控器输入端口
//PB6，7，8，9
void TIM4_Cap_Init(u16 arr, u16 psc)
{
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //使能TIM4时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB时钟
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9; //PB6,7,8,9 清除之前设置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;									 //PB6,7,8,9 输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9); //PB6,7,8,9  下拉

	//初始化定时器4 TIM4
	TIM_TimeBaseStructure.TIM_Period = arr;						//设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM4输入捕获参数 通道1
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;				 //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //配置输入分频,不分频
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道2
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;				 //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //配置输入分频,不分频
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道3
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;				 //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //配置输入分频,不分频
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//初始化TIM4输入捕获参数 通道4
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;				 //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	 //上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;			 //配置输入分频,不分频
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;						 //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;								   //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;					   //先占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;							   //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								   //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);												   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE); //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断
	TIM_Cmd(TIM4, ENABLE);
}

//电机输出端口
void TIM2_PWM_Init(u16 arr, u16 psc) //PWM的输出
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 使能GPIOA时钟时钟 */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //复用
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);		 //使能复用
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //禁止JTAG功能，把PB3，PB4作为普通IO口使用
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3); 

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = arr;						//定时器计数周期 0-999  1000
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//设置预分频：8+1分频   8K PWM频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//设置时钟分频系数：不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	//设置跳变值，当计数器计数到这个值时，电平发生跳变(即占空比) 初始值0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	//当定时器计数值小于定时设定值时为高电平
	/* 使能通道1 */
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	/* 使能通道2 */
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	/* 使能通道3 */
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	/* 使能通道4 */
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE); // 使能TIM2重载寄存器ARR
	TIM_Cmd(TIM2, ENABLE);				//使能定时器2
}
