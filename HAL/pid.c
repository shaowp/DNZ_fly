#include "pid.h"
#include "stflash.h"
#include "ANO_USART.h"
#include "rc_process.h"

Struct_PID neihuan_Roll_PID, neihuan_Pitch_PID, RateZ_PID; //内环
Struct_PID waihuan_Pitch_PID, waihuan_Roll_PID, Yaw_PID;   //外环

u8 PID_send_flag = 0; //向上位机发送PID数据
u8 PID_save_flag = 0; //将上位机发来的数据存储到内存中

//V1.7
//PID参数设置，copy from ZINA
/*
1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
5反馈            6偏差；        7上次偏差；       8偏差限幅值；
9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
17总输出限幅度
 {1  ,1 ,0 ,0 ,0 ,0 , 0 ,35  ,0  ,0 , 20,  1.2    ,0.000    ,0.00     ,0  ,0 , 250},//Pitch_Angle;偏航角度
 {0  ,1 ,0 ,0 ,0 ,0 , 0 ,500 ,0  ,0 ,150,  1.0     ,0.006     ,0.5     ,0  ,0 ,500},//Pitch_Gyro;偏航角速度*/
const float Control_Unit[15][17] =
	{
		/*                                      Kp    Ki        Kd            */
		/*1  2  3  4  5  6  7  8   	9  10 11    12    13        14  	15 16 17*/
		{1, 1, 0, 0, 0, 0, 0, 35, 	0, 0, 20, 	6.00, 0.0000, 	0.00, 	0, 0, 250},  //Pitch_Angle;俯仰角度 	外环PITCH
		{0, 1, 0, 0, 0, 0, 0, 500, 	0, 0, 200, 	0.20, 0.0000, 	4.00, 	0, 0, 500}, //Pitch_Gyro;俯仰角速度		内环PITCH
		{1, 1, 0, 0, 0, 0, 0, 35, 	0, 0, 20, 	6.00, 0.0000, 	0.00, 	0, 0, 250},  //Roll_Angle;横滚角		外环ROLL
		{0, 1, 0, 0, 0, 0, 0, 500, 	0, 0, 200, 	0.20, 0.0000, 	4.00, 	0, 0, 500}, //Roll_Gyro;横滚角速度		内环ROLL
		{1, 1, 0, 0, 0, 0, 0, 35, 	0, 0, 0, 	6.00, 0, 		0.00, 	0, 0, 100},		 //Yaw_Angle;偏航角			外环YAW
		{1, 1, 0, 0, 0, 0, 0, 100, 	0, 0, 30, 	2.20, 0.000, 	4.5, 	0, 0, 120},   //Yaw_Gyro;偏航角速度		内环YAW
};

_st_pidData pidData; //系统总控制器

//PID计算，分为内环和外环的计算
//PID外环？？？
float PID_Control(Struct_PID *Controler)
{
	/*******偏差计算*********************/
	Controler->Last_Err = Controler->Err;										  //保存上次偏差
	Controler->Err = Controler->Expect + Controler->Offset - Controler->FeedBack; //期望减去反馈得到偏差

	if (Controler->Err_Limit_Flag == 1) //偏差限幅度标志位									  //偏差限幅度标志位
	{
		if (Controler->Err >= Controler->Err_Max)
			Controler->Err = Controler->Err_Max;
		if (Controler->Err <= -Controler->Err_Max)
			Controler->Err = -Controler->Err_Max;
	}
	/*******积分计算*********************/
	if (Controler->Integrate_Separation_Flag == 1) //积分分离标志位
	{
		if (ABS(Controler->Err) <= Controler->Integrate_Separation_Err)
			Controler->Integrate += Controler->Ki * Controler->Err;
	}
	else
	{
		Controler->Integrate += Controler->Ki * Controler->Err;
	}
	/*******积分限幅*********************/
	if (Controler->Integrate_Limit_Flag == 1) //积分限制幅度标志
	{
		if (Controler->Integrate >= Controler->Integrate_Max)
			Controler->Integrate = Controler->Integrate_Max;
		if (Controler->Integrate <= -Controler->Integrate_Max)
			Controler->Integrate = -Controler->Integrate_Max;
	}
	/*******总输出计算*********************/
	Controler->Last_Control_OutPut = Controler->Control_OutPut;							  //输出值递推
	Controler->Control_OutPut = Controler->Kp * Controler->Err							  //比例
								+ Controler->Integrate									  //积分
								+ Controler->Kd * (Controler->Err - Controler->Last_Err); //微分
																						  /*******总输出限幅*********************/
	if (Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = Controler->Control_OutPut_Limit;
	if (Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
	/*******返回总输出*********************/
	return Controler->Control_OutPut;
}


//偏航角PID控制
float PID_Control_Yaw(Struct_PID *Controler)
{
	/*******偏差计算*********************/
	Controler->Last_Err = Controler->Err;										  //保存上次偏差
	Controler->Err = Controler->Expect + Controler->Offset - Controler->FeedBack; //期望减去反馈得到偏差
																				  /***********************偏航角偏差超过+-180处理*****************************/
	if (Controler->Err < -180)
		Controler->Err = Controler->Err + 360;
	if (Controler->Err > 180)
		Controler->Err = Controler->Err - 360;

	if (Controler->Err_Limit_Flag == 1) //偏差限幅度标志位
	{
		if (Controler->Err >= Controler->Err_Max)
			Controler->Err = Controler->Err_Max;
		if (Controler->Err <= -Controler->Err_Max)
			Controler->Err = -Controler->Err_Max;
	}
	/*******积分计算*********************/
	if (Controler->Integrate_Separation_Flag == 1) //积分分离标志位
	{
		if (ABS(Controler->Err) <= Controler->Integrate_Separation_Err)
			Controler->Integrate += Controler->Ki * Controler->Err;
	}
	else
	{
		Controler->Integrate += Controler->Ki * Controler->Err;
	}
	/*******积分限幅*********************/
	if (Controler->Integrate_Limit_Flag == 1) //积分限制幅度标志
	{
		if (Controler->Integrate >= Controler->Integrate_Max)
			Controler->Integrate = Controler->Integrate_Max;
		if (Controler->Integrate <= -Controler->Integrate_Max)
			Controler->Integrate = -Controler->Integrate_Max;
	}
	/*******总输出计算*********************/
	Controler->Last_Control_OutPut = Controler->Control_OutPut;							  //输出值递推
	Controler->Control_OutPut = Controler->Kp * Controler->Err							  //比例
								+ Controler->Integrate									  //积分
								+ Controler->Kd * (Controler->Err - Controler->Last_Err); //微分
																						  /*******总输出限幅*********************/
	if (Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = Controler->Control_OutPut_Limit;
	if (Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
	/*******返回总输出*********************/
	return Controler->Control_OutPut;
}



Butter_Parameter Control_Device_Div_LPF_Parameter = {
	//200---20hz
	1, -1.14298050254, 0.4128015980962,
	0.06745527388907, 0.1349105477781, 0.06745527388907};
//PID 内环？？？
float PID_Control_Div_LPF(Struct_PID *Controler)
{
	s16 i = 0;
	/*******偏差计算*********************/
	Controler->Last_Err = Controler->Err;										  //保存上次偏差
	Controler->Err = Controler->Expect + Controler->Offset - Controler->FeedBack; //期望减去反馈得到偏差
	Controler->Dis_Err = Controler->Err - Controler->Last_Err;					  //原始微分
	for (i = 4; i > 0; i--)														  //数字低通后微分项保存
	{
		Controler->Dis_Error_History[i] = Controler->Dis_Error_History[i - 1];
	}
	Controler->Dis_Error_History[0] = Control_Device_LPF(Controler->Dis_Err,
														 &Controler->Control_Device_LPF_Buffer,
														 &Control_Device_Div_LPF_Parameter); //巴特沃斯低通后得到的微分项,20hz

	if (Controler->Err_Limit_Flag == 1) //偏差限幅度标志位
	{
		if (Controler->Err >= Controler->Err_Max)
			Controler->Err = Controler->Err_Max;
		if (Controler->Err <= -Controler->Err_Max)
			Controler->Err = -Controler->Err_Max;
	}
	/*******积分计算*********************/
	if (Controler->Integrate_Separation_Flag == 1) //积分分离标志位
	{
		if (ABS(Controler->Err) <= Controler->Integrate_Separation_Err)
			Controler->Integrate += Controler->Ki * Controler->Err;
	}
	else
	{
		Controler->Integrate += Controler->Ki * Controler->Err;
	}
	/*******积分限幅*********************/
	if (Controler->Integrate_Limit_Flag == 1) //积分限制幅度标志
	{
		if (Controler->Integrate >= Controler->Integrate_Max)
			Controler->Integrate = Controler->Integrate_Max;
		if (Controler->Integrate <= -Controler->Integrate_Max)
			Controler->Integrate = -Controler->Integrate_Max;
	}
	/*******总输出计算*********************/
	Controler->Last_Control_OutPut = Controler->Control_OutPut; //输出值递推
	Controler->Control_OutPut = Controler->Kp * Controler->Err  //比例
								+ Controler->Integrate			//积分
								//+Controler->Kd*Controler->Dis_Err;//微分
								+ Controler->Kd * Controler->Dis_Error_History[0]; //微分项来源于巴特沃斯低通滤波器
																				   /*******总输出限幅*********************/
	if (Controler->Control_OutPut >= Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = Controler->Control_OutPut_Limit;
	if (Controler->Control_OutPut <= -Controler->Control_OutPut_Limit)
		Controler->Control_OutPut = -Controler->Control_OutPut_Limit;
	/*******返回总输出*********************/
	return Controler->Control_OutPut;
}

void PID_INIT_ALL(void)
{
	PID_Init(&pidData.Pitch, Pitch_Angle_Controler);
	PID_Init(&pidData.RateY, Pitch_Gyro_Controler);
	PID_Init(&pidData.Roll, Roll_Angle_Controler);
	PID_Init(&pidData.RateX, Roll_Gyro_Controler);
	PID_Init(&pidData.Yaw, Yaw_Angle_Controler);
	PID_Init(&pidData.RateZ, Yaw_Gyro_Controler);
}

void PID_Init(Struct_PID *Controler, Controler_Label Label) //PID参数初始化
{
	//ZINA飞控标志
	// 1偏差限幅标志；  2积分限幅标志；3积分分离标志；   4期望；
	// 5反馈            6偏差；        7上次偏差；       8偏差限幅值；
	// 9积分分离偏差值；10积分值       11积分限幅值；    12控制参数Kp；
	// 13控制参数Ki；   14控制参数Kd； 15控制器总输出；  16上次控制器总输出
	// 17总输出限幅度
	Controler->Err_Limit_Flag = (u8)(Control_Unit[Label][0]);			 //1偏差限幅标志
	Controler->Integrate_Limit_Flag = (u8)(Control_Unit[Label][1]);		 //2积分限幅标志
	Controler->Integrate_Separation_Flag = (u8)(Control_Unit[Label][2]); //3积分分离标志
	Controler->Expect = Control_Unit[Label][3];							 //4期望
	Controler->FeedBack = Control_Unit[Label][4];						 //5反馈值
	Controler->Err = Control_Unit[Label][5];							 //6偏差
	Controler->Last_Err = Control_Unit[Label][6];						 //7上次偏差
	Controler->Err_Max = Control_Unit[Label][7];						 //8偏差限幅值
	Controler->Integrate_Separation_Err = Control_Unit[Label][8];		 //9积分分离偏差值
	Controler->Integrate = Control_Unit[Label][9];						 //10积分值
	Controler->Integrate_Max = Control_Unit[Label][10];					 //11积分限幅值
	Controler->Kp = Control_Unit[Label][11];							 //12控制参数Kp
	Controler->Ki = Control_Unit[Label][12];							 //13控制参数Ki
	Controler->Kd = Control_Unit[Label][13];							 //14控制参数Ki
	Controler->Control_OutPut = Control_Unit[Label][14];				 //15控制器总输出
	Controler->Last_Control_OutPut = Control_Unit[Label][15];			 //16上次控制器总输出
	Controler->Control_OutPut_Limit = Control_Unit[Label][16];			 //16上次控制器总输出

	/*************内环*******************/
	//参数随便写的

	// neihuan_Roll_PID.Kp = 0.0;
	// neihuan_Roll_PID.Ki = 0.00;
	// neihuan_Roll_PID.Kd = 0.0;
	// neihuan_Roll_PID.Err_Max = 30;
	// neihuan_Roll_PID.IntegLimitHigh = 20;
	// neihuan_Roll_PID.IntegLimitLow = -20;

	// neihuan_Pitch_PID.Kp = 0;
	// neihuan_Pitch_PID.Ki = 0;
	// neihuan_Pitch_PID.Kd = 0;
	// neihuan_Pitch_PID.Err_Max = 30;
	// neihuan_Pitch_PID.IntegLimitHigh = 20;
	// neihuan_Pitch_PID.IntegLimitLow = -20;

	// RateZ_PID.Kp = 0;
	// RateZ_PID.Ki = 0;
	// RateZ_PID.Kd = 0;
	// RateZ_PID.Err_Max = 30;
	// RateZ_PID.IntegLimitHigh = 20;
	// RateZ_PID.IntegLimitLow = -20;
	// /////////////////////////////////////

	// /*************外环*******************/
	// waihuan_Pitch_PID.Kp = 0;
	// waihuan_Pitch_PID.Ki = 0;
	// waihuan_Pitch_PID.Ki = 0;
	// waihuan_Pitch_PID.Kd = 0;
	// waihuan_Pitch_PID.Err_Max = 30;
	// waihuan_Pitch_PID.IntegLimitHigh = 20;
	// waihuan_Pitch_PID.IntegLimitLow = -20;

	// waihuan_Roll_PID.Kp = 0;
	// waihuan_Roll_PID.Ki = 0;
	// waihuan_Roll_PID.Kd = 0;
	// waihuan_Roll_PID.Err_Max = 30;
	// waihuan_Roll_PID.IntegLimitHigh = 20;
	// neihuan_Pitch_PID.IntegLimitLow = -20;

	// Yaw_PID.Kp = 0;
	// Yaw_PID.Ki = 0;
	// Yaw_PID.Kd = 0;
	// Yaw_PID.Err_Max = 30;
	// Yaw_PID.IntegLimitHigh = 20;
	// Yaw_PID.IntegLimitLow = -20;

#ifdef USE_AT24C02
	read_PID();
#endif
}

void send_PID(void)
{
#ifdef USE_AT24C02
	read_PID(); //如果有AT24C02则使用
#endif
	//先发送内环
	//pidData.Pitch

	ANO_DT_Send_PID(1, 	pidData.RateX.Kp, pidData.RateX.Ki, pidData.RateX.Kd,
						pidData.RateY.Kp, pidData.RateY.Ki, pidData.RateY.Kd,
						pidData.RateZ.Kp, pidData.RateZ.Ki, pidData.RateZ.Kd);
	ANO_DT_Send_PID(2, 	pidData.Roll.Kp, pidData.Roll.Ki, pidData.Roll.Kd,
						pidData.Pitch.Kp, pidData.Pitch.Ki, pidData.Pitch.Kd,
						pidData.Yaw.Kp, pidData.Yaw.Ki, pidData.Yaw.Kd);

	// ANO_DT_Send_PID(1, neihuan_Roll_PID.Kp, neihuan_Roll_PID.Ki, neihuan_Roll_PID.Kd,
	// 				neihuan_Pitch_PID.Kp, neihuan_Pitch_PID.Ki, neihuan_Pitch_PID.Kd,
	// 				RateZ_PID.Kp, RateZ_PID.Ki, RateZ_PID.Kd);
	//delay_ms(2);
	//再发送外环
	// ANO_DT_Send_PID(2, 	pidData.Kp, pidData.Ki, pidData.Kd,
	// 					pidData.Kp, pidData.Ki, pidData.Kd,
	// 					pidData.Kp, pidData.Ki, pidData.Kd);
	//delay_ms(2);
	PID_send_flag = 0;
}

void save_PID(void)
{
	//存数据的时候关掉主中断

	//写入的参数为  地址   数据   数据长度（几个字节）
	//内环数据的写入
	if (PID_save_flag == 1)
	{
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		AT24CXX_WriteLenByte(neihuan_Roll_PID_kp, (u16)(neihuan_Roll_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(neihuan_Roll_PID_ki, (u16)(neihuan_Roll_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(neihuan_Roll_PID_kd, (u16)(neihuan_Roll_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(neihuan_Pitch_PID_kp, (u16)(neihuan_Pitch_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(neihuan_Pitch_PID_ki, (u16)(neihuan_Pitch_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(neihuan_Pitch_PID_kd, (u16)(neihuan_Pitch_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(RateZ_PID_kp, (u16)(RateZ_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(RateZ_PID_ki, (u16)(RateZ_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(RateZ_PID_kd, (u16)(RateZ_PID.Kd * 1000), 2);

		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
	}
	if (PID_save_flag == 2) //外环
	{
		TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		AT24CXX_WriteLenByte(waihuan_Pitch_PID_kp, (u16)(waihuan_Pitch_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(waihuan_Pitch_PID_ki, (u16)(waihuan_Pitch_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(waihuan_Pitch_PID_kd, (u16)(waihuan_Pitch_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(waihuan_Roll_PID_kp, (u16)(waihuan_Roll_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(waihuan_Roll_PID_ki, (u16)(waihuan_Roll_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(waihuan_Roll_PID_kd, (u16)(waihuan_Roll_PID.Kd * 1000), 2);

		AT24CXX_WriteLenByte(Yaw_PID_kp, (u16)(Yaw_PID.Kp * 1000), 2);
		AT24CXX_WriteLenByte(Yaw_PID_ki, (u16)(Yaw_PID.Ki * 1000), 2);
		AT24CXX_WriteLenByte(Yaw_PID_kd, (u16)(Yaw_PID.Kd * 1000), 2);
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
	}
	// if (PID_save_flag == 5)
	// {
	// 	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断

	// 	AT24CXX_WriteLenByte(waihuan_Pitch_PID_kp, (u16)(waihuan_Pitch_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(waihuan_Pitch_PID_ki, (u16)(waihuan_Pitch_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(waihuan_Pitch_PID_kd, (u16)(waihuan_Pitch_PID.Kd * 1000), 2);

	// 	AT24CXX_WriteLenByte(waihuan_Roll_PID_kp, (u16)(waihuan_Roll_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(waihuan_Roll_PID_ki, (u16)(waihuan_Roll_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(waihuan_Roll_PID_kd, (u16)(waihuan_Roll_PID.Kd * 1000), 2);

	// 	AT24CXX_WriteLenByte(Yaw_PID_kp, (u16)(Yaw_PID.Kp * 1000), 2);
	// 	AT24CXX_WriteLenByte(Yaw_PID_ki, (u16)(Yaw_PID.Ki * 1000), 2);
	// 	AT24CXX_WriteLenByte(Yaw_PID_kd, (u16)(Yaw_PID.Kd * 1000), 2);

	// 	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);  //使能指定的TIM1中断,允许更新中断
	// }
	PID_save_flag = 0;
}

void read_PID(void)
{
	//读取的   地址       数据的长度
	//读取内环数据
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
	neihuan_Roll_PID.Kp = AT24CXX_ReadLenByte(neihuan_Roll_PID_kp, 2) / 1000.0;
	neihuan_Roll_PID.Ki = AT24CXX_ReadLenByte(neihuan_Roll_PID_ki, 2) / 1000.0;
	neihuan_Roll_PID.Kd = AT24CXX_ReadLenByte(neihuan_Roll_PID_kd, 2) / 1000.0;
	neihuan_Pitch_PID.Kp = AT24CXX_ReadLenByte(neihuan_Pitch_PID_kp, 2) / 1000.0;
	neihuan_Pitch_PID.Ki = AT24CXX_ReadLenByte(neihuan_Pitch_PID_ki, 2) / 1000.0;
	neihuan_Pitch_PID.Kd = AT24CXX_ReadLenByte(neihuan_Pitch_PID_kd, 2) / 1000.0;
	RateZ_PID.Kp = AT24CXX_ReadLenByte(RateZ_PID_kp, 2) / 1000.0;
	RateZ_PID.Ki = AT24CXX_ReadLenByte(RateZ_PID_ki, 2) / 1000.0;
	RateZ_PID.Kd = AT24CXX_ReadLenByte(RateZ_PID_kd, 2) / 1000.0;

	// // //读取外环数据
	waihuan_Pitch_PID.Kp = AT24CXX_ReadLenByte(waihuan_Pitch_PID_kp, 2) / 1000.0;
	waihuan_Pitch_PID.Ki = AT24CXX_ReadLenByte(waihuan_Pitch_PID_ki, 2) / 1000.0;
	waihuan_Pitch_PID.Kd = AT24CXX_ReadLenByte(waihuan_Pitch_PID_kd, 2) / 1000.0;

	waihuan_Roll_PID.Kp = AT24CXX_ReadLenByte(waihuan_Roll_PID_kp, 2) / 1000.0;
	waihuan_Roll_PID.Ki = AT24CXX_ReadLenByte(waihuan_Roll_PID_ki, 2) / 1000.0;
	waihuan_Roll_PID.Kd = AT24CXX_ReadLenByte(waihuan_Roll_PID_kd, 2) / 1000.0;

	Yaw_PID.Kp = AT24CXX_ReadLenByte(Yaw_PID_kp, 2) / 1000.0;
	Yaw_PID.Ki = AT24CXX_ReadLenByte(Yaw_PID_ki, 2) / 1000.0;
	Yaw_PID.Kd = AT24CXX_ReadLenByte(Yaw_PID_kd, 2) / 1000.0;
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
}

//2019.11.28 废弃 简单的PID控制器不配存在
//外环角度环
// float pidupdate_Angle(Struct_PID *pid)
// {
// 	//误差计算：期望值-测量值
// 	// pid->error = pid->desired - pid->measured;

// 	// //偏差限幅
// 	// // if (pid->error >= pid->Err_Max)
// 	// // 	pid->error = pid->Err_Max;
// 	// // if (pid->error <= -pid->Err_Max)
// 	// // 	pid->error = -pid->Err_Max;

// 	// //积分计算
// 	// pid->integ += pid->Ki * pid->error;
// 	// //积分限幅
// 	// pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);
// 	// if (RC_data.thr <= 1100)
// 	// 	pid->integ = 0;
// 	// pid->out = pid->Kp * pid->error +
// 	// 		   pid->Kd * (pid->error - pid->lasterror) +
// 	// 		   pid->integ;
// 	// pid->lasterror = pid->error;
// 	// return pid->out;
// }

// //内环角速度环
// float pidupdate_Rate(Struct_PID *pid)
// {
// 	//误差计算：期望值-测量值
// 	// pid->error = pid->desired - pid->measured;

// 	// //偏差限幅
// 	// // if (pid->error >= pid->Err_Max)
// 	// // 	pid->error = pid->Err_Max;
// 	// // if (pid->error <= -pid->Err_Max)
// 	// // 	pid->error = -pid->Err_Max;

// 	// //积分计算
// 	// pid->integ += pid->Ki * pid->error;
// 	// //积分限幅
// 	// pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh);
// 	// if (RC_data.thr <= 1100)
// 	// 	pid->integ = 0;
// 	// pid->out = pid->Kp * pid->error +
// 	// 		   pid->Kd * (pid->error - pid->lasterror) +
// 	// 		   pid->integ;
// 	// pid->lasterror = pid->error;
// 	// return pid->out;
// }
