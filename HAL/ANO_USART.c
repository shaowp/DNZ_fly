#include "ANO_USART.h"
#include "usbio.h"
#include "pid.h"
#include "stflash.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

//匿名上位机程序
void usart1_send_char(u8 c)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		; //循环发送,直到发送完毕
	USART_SendData(USART1, c);
}

void usart1_send_str(u8 *str, u8 length)
{
	u8 i;
	for (i = 0; i < length; i++)
		usart1_send_char(str[i]); //发送数据到串口1
}

/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
/**********以下为正点原子里的老版本上位机*********************/

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun, u8 *data, u8 len)
{
	u8 send_buf[32];
	u8 i;
	if (len > 28)
		return;			   //最多28字节数据
	send_buf[len + 3] = 0; //校验数置零
	send_buf[0] = 0X88;	//帧头
	send_buf[1] = fun;	 //功能字
	send_buf[2] = len;	 //数据长度
	for (i = 0; i < len; i++)
		send_buf[3 + i] = data[i]; //复制数据
	for (i = 0; i < len + 3; i++)
		send_buf[len + 3] += send_buf[i]; //计算校验和
	for (i = 0; i < len + 4; i++)
		usart1_send_char(send_buf[i]); //发送数据到串口1
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
	//
	u8 tbuf[12];
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	usart1_niming_report(0XA1, tbuf, 12); //自定义帧,0XA1
}
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
	u8 tbuf[28];
	u8 i;
	for (i = 0; i < 28; i++)
		tbuf[i] = 0; //清0
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	tbuf[18] = (roll >> 8) & 0XFF;
	tbuf[19] = roll & 0XFF;
	tbuf[20] = (pitch >> 8) & 0XFF;
	tbuf[21] = pitch & 0XFF;
	tbuf[22] = (yaw >> 8) & 0XFF;
	tbuf[23] = yaw & 0XFF;
	usart1_niming_report(0XAF, tbuf, 28); //飞控显示帧,0XAF
}
/********************** E N D ********************************/
/////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
/*******************以下为新版上位机 v4.33********************/
//发送传感器数据到匿名最新的上位机
void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar)
{
	u8 _cnt = 0;
	vs16 _temp;
	u8 sum = 0;
	u8 i = 0;
	u8 data_to_send[50]; //发送数据缓存

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;

	_temp = a_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, _cnt);
}

//发送遥控器数据
void ANO_DT_Send_RCData(u16 thr, u16 yaw, u16 rol, u16 pit, u16 aux1, u16 aux2, u16 aux3, u16 aux4, u16 aux5, u16 aux6)
{
	u8 _cnt = 0;
	vs16 _temp;
	u8 sum = 0;
	u8 i = 0;
	u8 data_to_send[50]; //发送数据缓存
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x03;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = BYTE1(thr);
	data_to_send[_cnt++] = BYTE0(thr);
	data_to_send[_cnt++] = BYTE1(yaw);
	data_to_send[_cnt++] = BYTE0(yaw);
	data_to_send[_cnt++] = BYTE1(rol);
	data_to_send[_cnt++] = BYTE0(rol);
	data_to_send[_cnt++] = BYTE1(pit);
	data_to_send[_cnt++] = BYTE0(pit);
	data_to_send[_cnt++] = BYTE1(aux1);
	data_to_send[_cnt++] = BYTE0(aux1);
	data_to_send[_cnt++] = BYTE1(aux2);
	data_to_send[_cnt++] = BYTE0(aux2);
	data_to_send[_cnt++] = BYTE1(aux3);
	data_to_send[_cnt++] = BYTE0(aux3);
	data_to_send[_cnt++] = BYTE1(aux4);
	data_to_send[_cnt++] = BYTE0(aux4);
	data_to_send[_cnt++] = BYTE1(aux5);
	data_to_send[_cnt++] = BYTE0(aux5);
	data_to_send[_cnt++] = BYTE1(aux6);
	data_to_send[_cnt++] = BYTE0(aux6);

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, _cnt);
}

//发送飞控状态到上位机
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{

	u8 _cnt = 0;
	u8 sum = 0;
	u8 i = 0;
	u8 data_to_send[50]; //发送数据缓存
	vs16 _temp;
	vs32 _temp2 = alt;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;

	_temp = (int)(angle_rol * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_pit * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[_cnt++] = BYTE3(_temp2);
	data_to_send[_cnt++] = BYTE2(_temp2);
	data_to_send[_cnt++] = BYTE1(_temp2);
	data_to_send[_cnt++] = BYTE0(_temp2);

	data_to_send[_cnt++] = fly_model;

	data_to_send[_cnt++] = armed;

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, _cnt);
}

//发送电机输出
void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8)
{
	u8 _cnt = 0;
	u8 sum = 0;
	u8 i;
	u8 data_to_send[50]; //发送数据缓存
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x06;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = BYTE1(m_1);
	data_to_send[_cnt++] = BYTE0(m_1);
	data_to_send[_cnt++] = BYTE1(m_2);
	data_to_send[_cnt++] = BYTE0(m_2);
	data_to_send[_cnt++] = BYTE1(m_3);
	data_to_send[_cnt++] = BYTE0(m_3);
	data_to_send[_cnt++] = BYTE1(m_4);
	data_to_send[_cnt++] = BYTE0(m_4);
	data_to_send[_cnt++] = BYTE1(m_5);
	data_to_send[_cnt++] = BYTE0(m_5);
	data_to_send[_cnt++] = BYTE1(m_6);
	data_to_send[_cnt++] = BYTE0(m_6);
	data_to_send[_cnt++] = BYTE1(m_7);
	data_to_send[_cnt++] = BYTE0(m_7);
	data_to_send[_cnt++] = BYTE1(m_8);
	data_to_send[_cnt++] = BYTE0(m_8);

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, _cnt);
}

//向上位机发送PID数据
void ANO_DT_Send_PID(u8 group, float p1_p, float p1_i, float p1_d, float p2_p, float p2_i, float p2_d, float p3_p, float p3_i, float p3_d)
{
	u8 _cnt = 0;
	u8 sum = 0;
	u8 i;
	vs16 _temp;
	u8 data_to_send[50]; //发送数据缓存
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x10 + group - 1;
	data_to_send[_cnt++] = 0;

	_temp = p1_p * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p1_i * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p1_d * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p2_p * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p2_i * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p3_p * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p3_i * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	for (i = 0; i < _cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, _cnt);
}

/////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************/
/*
            以下程序均在USB中断中调用
*/
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0, _data_cnt = 0;
	static u8 state = 0;

	if (state == 0 && data == 0xAA)
	{
		state = 1;
		RxBuffer[0] = data;
	}
	else if (state == 1 && data == 0xAF)
	{
		state = 2;
		RxBuffer[1] = data;
	}
	else if (state == 2 && data < 0XF1)
	{
		state = 3;
		RxBuffer[2] = data;
	}
	else if (state == 3 && data < 50)
	{
		state = 4;
		RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if (state == 4 && _data_len > 0)
	{
		_data_len--;
		RxBuffer[4 + _data_cnt++] = data;
		if (_data_len == 0)
			state = 5;
	}
	else if (state == 5)
	{
		state = 0;
		RxBuffer[4 + _data_cnt] = data;
		ANO_DT_Data_Receive_Anl(RxBuffer, _data_cnt + 5);
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 data_to_send[50]; //发送数据缓存
	u8 i;

	data_to_send[0] = 0xAA;
	data_to_send[1] = 0xAA;
	data_to_send[2] = 0xEF;
	data_to_send[3] = 2;
	data_to_send[4] = head;
	data_to_send[5] = check_sum;

	for (i = 0; i < 6; i++)
		sum += data_to_send[i];
	data_to_send[6] = sum;

	Usb_Hid_Adddata(data_to_send, 7);
	Usb_Hid_Send();
	usart1_send_str(data_to_send, 7);
}

//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(u8 *data_buf, u8 num)
{
	u8 sum = 0;
	u8 i;
	for (i = 0; i < (num - 1); i++)
		sum += *(data_buf + i);
	if (!(sum == *(data_buf + num - 1)))
		return; //判断sum
	if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAF))
		return; //判断帧头

	//IMU校准功能
	// if(*(data_buf+2)==0X01)
	// {
	// 	if(*(data_buf+4)==0X01)
	// 		mpu6050.Acc_CALIBRATE = 1;
	// 	if(*(data_buf+4)==0X02)
	// 		mpu6050.Gyro_CALIBRATE = 1;
	// 	if(*(data_buf+4)==0X03)
	// 	{
	// 		mpu6050.Acc_CALIBRATE = 1;
	// 		mpu6050.Gyro_CALIBRATE = 1;
	// 	}
	// }

	if (*(data_buf + 2) == 0X02)
	{
		if (*(data_buf + 4) == 0X01)
		{
			PID_send_flag = 1; //发送PID标志位
							   //飞控向上位机发送PID的标志位
							   // f.send_pid1 = 1;
							   // f.send_pid2 = 1;
							   // f.send_pid3 = 1;
							   // f.send_pid4 = 1;
							   // f.send_pid5 = 1;
							   // f.send_pid6 = 1;
		}
		// if(*(data_buf+4)==0X02)
		// {

		// }
		// if(*(data_buf+4)==0XA0)		//读取版本信息
		// {
		// 	f.send_version = 1;
		// }
		// if(*(data_buf+4)==0XA1)		//恢复默认参数
		// {
		// 	Para_ResetToFactorySetup();
		// }
	}

	//存PID数据
	if (*(data_buf + 2) == 0X10) //PID1
	{
		// printf("get PID 1\n");
		RateX_PID.Kp = 0.001 * ((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		RateX_PID.Ki = 0.001 * ((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		RateX_PID.Kd = 0.001 * ((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		RateY_PID.Kp = 0.001 * ((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		RateY_PID.Ki = 0.001 * ((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		RateY_PID.Kd = 0.001 * ((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		RateZ_PID.Kp = 0.001 * ((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		RateZ_PID.Ki = 0.001 * ((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		RateZ_PID.Kd = 0.001 * ((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		PID_save_flag = 1;
		// Param_SavePID();
		// {
		// 	//把数据存到flash
		// 	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		// 	//写入的参数为  地址   数据   数据长度（几个字节）
		// 	//内环数据的写入
		// 	AT24CXX_WriteLenByte(RateX_PID_kp, (u16)(RateX_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateX_PID_ki, (u16)(RateX_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateX_PID_kd, (u16)(RateX_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateY_PID_kp, (u16)(RateY_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateY_PID_ki, (u16)(RateY_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateY_PID_kd, (u16)(RateY_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateZ_PID_kp, (u16)(RateZ_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateZ_PID_ki, (u16)(RateZ_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(RateZ_PID_kd, (u16)(RateZ_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
		// }
	}
	if (*(data_buf + 2) == 0X11) //PID2
	{
		// printf("get PID 2\n");
		Roll_PID.Kp = 0.001 * ((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		Roll_PID.Ki = 0.001 * ((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		Roll_PID.Kd = 0.001 * ((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		Pitch_PID.Kp = 0.001 * ((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		Pitch_PID.Ki = 0.001 * ((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		Pitch_PID.Kd = 0.001 * ((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		Yaw_PID.Kp = 0.001 * ((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		Yaw_PID.Ki = 0.001 * ((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		Yaw_PID.Kd = 0.001 * ((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		PID_save_flag = 2;
		
		
		
		
		// Param_SavePID();
		// {
		// 	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); //使能指定的TIM1中断,允许更新中断
		// 	//外环数据的写入
		// 	AT24CXX_WriteLenByte(Pitch_PID_kp, (u16)(Pitch_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Pitch_PID_ki, (u16)(Pitch_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Pitch_PID_kd, (u16)(Pitch_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Roll_PID_kp, (u16)(Roll_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Roll_PID_ki, (u16)(Roll_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Roll_PID_kd, (u16)(Roll_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Yaw_PID_kp, (u16)(Yaw_PID.Kp * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Yaw_PID_ki, (u16)(Yaw_PID.Ki * 1000), 2);
		// 	delay_ms(10);
		// 	AT24CXX_WriteLenByte(Yaw_PID_kd, (u16)(Yaw_PID.Kd * 1000), 2);
		// 	delay_ms(10);
		// 	//打开中断
		// 	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); //使能指定的TIM1中断,允许更新中断
		// }
	}
	if (*(data_buf + 2) == 0X12) //PID3
	{

		// printf("get PID 3\n");
		// ctrl_2.PID[PIDROLL].kp = 0.001 * ((vs16)(*(data_buf + 4) << 8) | *(data_buf + 5));
		// ctrl_2.PID[PIDROLL].ki = 0.001 * ((vs16)(*(data_buf + 6) << 8) | *(data_buf + 7));
		// ctrl_2.PID[PIDROLL].kd = 0.001 * ((vs16)(*(data_buf + 8) << 8) | *(data_buf + 9));
		// ctrl_2.PID[PIDPITCH].kp = 0.001 * ((vs16)(*(data_buf + 10) << 8) | *(data_buf + 11));
		// ctrl_2.PID[PIDPITCH].ki = 0.001 * ((vs16)(*(data_buf + 12) << 8) | *(data_buf + 13));
		// ctrl_2.PID[PIDPITCH].kd = 0.001 * ((vs16)(*(data_buf + 14) << 8) | *(data_buf + 15));
		// ctrl_2.PID[PIDYAW].kp = 0.001 * ((vs16)(*(data_buf + 16) << 8) | *(data_buf + 17));
		// ctrl_2.PID[PIDYAW].ki = 0.001 * ((vs16)(*(data_buf + 18) << 8) | *(data_buf + 19));
		// ctrl_2.PID[PIDYAW].kd = 0.001 * ((vs16)(*(data_buf + 20) << 8) | *(data_buf + 21));
		ANO_DT_Send_Check(*(data_buf + 2), sum);
		// Param_SavePID();
	}
	if (*(data_buf + 2) == 0X13) //PID4
	{
		// printf("get PID 4\n");
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	if (*(data_buf + 2) == 0X14) //PID5
	{
		// printf("get PID 5\n");
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
	if (*(data_buf + 2) == 0X15) //PID6
	{
		// printf("get PID 6\n");
		ANO_DT_Send_Check(*(data_buf + 2), sum);
	}
}

/********************** E N D ********************************/
/////////////////////////////////////////////////////////////////////
