#include "pid.h"
#include "global_variable.h"
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void PID_INIT(void)
{
	HIGHT_PID_struct.Kp = 10;
	HIGHT_PID_struct.Ki = 0.0;
	HIGHT_PID_struct.Kd = 0.0;

	waihuan_ROLL.Kp = 3.0f;
	waihuan_ROLL.Ki = 0.01f;
	waihuan_ROLL.Kd = 0.1f;
	waihuan_ROLL.IntegLimitHigh = 300;
	waihuan_ROLL.IntegLimitLow = -300;
	neihuan_ROLL.Kp = 1.20f;
	neihuan_ROLL.Ki = 0.01f;
	neihuan_ROLL.Kd = 17.0f;
	neihuan_ROLL.IntegLimitHigh = 300;
	neihuan_ROLL.IntegLimitLow = -300;

	waihuan_PITCH.Kp = 3.0f;
	waihuan_PITCH.Ki = 0.01f;
	waihuan_PITCH.Kd = 0.1f;
	waihuan_PITCH.IntegLimitHigh = 300;
	waihuan_PITCH.IntegLimitLow = -300;
	neihuan_PITCH.Kp = 1.20f;
	neihuan_PITCH.Ki = 0.01f;
	neihuan_PITCH.Kd = 17.0f;
	neihuan_PITCH.IntegLimitHigh = 300;
	neihuan_PITCH.IntegLimitLow = -300;

	waihuan_YAW.Kp = 7.0f;
	waihuan_YAW.Ki = 0;
	waihuan_YAW.Kd = 0;
	neihuan_YAW.Kp = 2.0f;
	neihuan_YAW.Ki = 0;
	neihuan_YAW.Kd = 15.0f;
}

//位置式PID
float pidupdate(pid_TypeDef *pid)
{
	float termI;
	pid->error = pid->desired - pid->measured;
	pid->integ += pid->error * 0.01;
	pid->integ = LIMIT(pid->integ, pid->IntegLimitLow, pid->IntegLimitHigh); //进行积分限幅
	pid->out = pid->Kp * pid->error + pid->Kd * (pid->error - pid->lasterror) + pid->Ki * pid->integ;
	pid->lasterror = pid->error;
	return pid->out;
}

void USAT_GET_PID(void)
{
	static u16 t;
	static u16 len;
	if (USART_RX_STA & 0x8000)
	{
		len = USART_RX_STA & 0x3fff; //得到此次接收到的数据长度
		printf("\r\your TX DATA:\r\n\r\n");
		for (t = 0; t < len; t++)
		{
			USART_SendData(USART1, USART_RX_BUF[t]); //向串口1发送数据
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET)
				; //等待发送结束
		}
		USART_ROLL_neihuan_kp = (USART_RX_BUF[0] - 48) * 100 + (USART_RX_BUF[1] - 48) * 10 + (USART_RX_BUF[2] - 48);
		USART_ROLL_neihuan_kd = (USART_RX_BUF[4] - 48) * 100 + (USART_RX_BUF[5] - 48) * 10 + (USART_RX_BUF[6] - 48);
		USART_ROLL_waihuan_kp = (USART_RX_BUF[8] - 48) * 100 + (USART_RX_BUF[9] - 48) * 10 + (USART_RX_BUF[10] - 48);
		//USART_PITCH_neihuan_kp = (USART_RX_BUF[12] - 48) * 100 + (USART_RX_BUF[13] - 48) * 10 + (USART_RX_BUF[14] - 48);
		// USART_PITCH_neihuan_kd = (USART_RX_BUF[16] - 48) * 100 + (USART_RX_BUF[17] - 48) * 10 + (USART_RX_BUF[18] - 48);
		// USART_PITCH_waihuan_kp = (USART_RX_BUF[20] - 48) * 100 + (USART_RX_BUF[21] - 48) * 10 + (USART_RX_BUF[22] - 48);
		// USART_YAW_neihuan_kp = (USART_RX_BUF[24] - 48) * 100 + (USART_RX_BUF[25] - 48) * 10 + (USART_RX_BUF[26] - 48);
		// USART_YAW_neihuan_kd = (USART_RX_BUF[28] - 48) * 100 + (USART_RX_BUF[29] - 48) * 10 + (USART_RX_BUF[30] - 48);
		// USART_YAW_waihuan_kp = (USART_RX_BUF[32] - 48) * 100 + (USART_RX_BUF[33] - 48) * 10 + (USART_RX_BUF[34] - 48);
		//printf("%d",USART_RX_BUF[0]-48);
		//USART_SendData(USART1, USART_RX_BUF[0]);
		USART_RX_STA = 0;
		printf("\r\n\r\n"); //插入换行
		printf("usart_roll_kp:%d\tusart_roll_kd:%d\tusart_roll_waihuan_kp:%d\n", USART_ROLL_neihuan_kp, USART_ROLL_neihuan_kd, USART_ROLL_waihuan_kp);
		//printf("usart_pitch_kp:%d\tusart_pitch_kd:%d\tusart_pitch_waihuan_kp:%d\n", USART_PITCH_neihuan_kp, USART_PITCH_neihuan_kd, USART_PITCH_waihuan_kp);
		//printf("usart_yaw_kp:%d\tusart_yaw_kd:%d\tusart_yaw_waihuan_kp:%d\n", USART_YAW_neihuan_kp, USART_YAW_neihuan_kd, USART_YAW_waihuan_kp);

		neihuan_ROLL.Kp = 1.0 * USART_ROLL_neihuan_kp / 10;
		neihuan_ROLL.Kd = 1.0 * USART_ROLL_neihuan_kd / 10;
		waihuan_ROLL.Kp = 1.0 * USART_ROLL_waihuan_kp / 10;

		// neihuan_YAW.Kp = 1.0 * USART_YAW_neihuan_kp / 10;
		// neihuan_YAW.Kd = 1.0 * USART_YAW_neihuan_kd / 10;
		// waihuan_YAW.Kp = 1.0 * USART_YAW_waihuan_kp / 10;

		// neihuan_PITCH.Kp = 1.0 * USART_PITCH_neihuan_kp / 10;
		// neihuan_PITCH.Kd = 1.0 * USART_PITCH_neihuan_kd / 10;
		// waihuan_PITCH.Kp = 1.0 * USART_PITCH_waihuan_kp / 10;

		// printf("%.2f\t%.2f\t%.2f\n%.2f\t%.2f\t%.2f\n%.2f\t%.2f\t%.2f\n", neihuan_ROLL.Kd, neihuan_ROLL.Kp, waihuan_ROLL.Kp,
		// 	   neihuan_PITCH.Kd, neihuan_PITCH.Kp, waihuan_PITCH.Kp,
		// 	   neihuan_YAW.Kd, neihuan_YAW.Kp, waihuan_YAW.Kp);

		//调试ROLL
		printf("%.2f\t%.2f\t%.2f\n", neihuan_ROLL.Kd, neihuan_ROLL.Kp, waihuan_ROLL.Kp);
		OLED_ShowNum(0, 2, USART_ROLL_neihuan_kp, 3, 16); //显示ASCII字符的码值
		OLED_ShowNum(0, 4, USART_ROLL_neihuan_kd, 3, 16); //显示ASCII字符的码值
		OLED_ShowNum(0, 6, USART_ROLL_waihuan_kp, 3, 16); //显示ASCII字符的码值
	}
}