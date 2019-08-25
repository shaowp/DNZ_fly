/**
  ******************************************************************************
  * @file    usbio.c
  * $Author: wdluo $
  * $Revision: 67 $
  * $Date:: 2012-08-15 19:00:29 +0800 #$
  * @brief   USB上层函数.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "usb_desc.h"
#include "usb_lib.h"
#include "usbio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
extern uint8_t USB_Receive_Buffer[REPORT_COUNT];
extern uint8_t USB_Send_Buffer[REPORT_COUNT];
uint8_t g_dataSerialNumber;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  通过USB发送数据
  * @param  data 数据存储首地址
  * @param  dataNum 发送的数据字节数
  * @retval 发送的字节数
  */
uint32_t USB_SendData(uint8_t *data, uint32_t dataNum)
{

#ifndef STM32F10X_CL
	//将数据通过USB发送出去
	UserToPMABufferCopy(data, ENDP2_TXADDR, dataNum);
	//SetEPTxCount(ENDP2, REPORT_COUNT);
	SetEPTxValid(ENDP2);
#else
	USB_SIL_Write(EP2_IN, data, dataNum);
#endif
	return dataNum;
}
/**
  * @brief  接收从USB获取的数据
  * @param  data 数据存储首地址
  * @param  dataNum 准备读取的数据字节数
  * @retval 读取的字节数
  */
uint32_t USB_GetData(uint8_t *data, uint32_t dataNum)
{
	uint32_t len = 0;
	if (dataNum > sizeof(USB_Receive_Buffer))
	{
		dataNum = sizeof(USB_Receive_Buffer);
	}
	for (len = 0; len < dataNum; len++)
	{
		*data = USB_Receive_Buffer[len];
		data++;
	}
	return dataNum;
}

u8 Hid_RxData[64];
u8 HID_SEND_TIMEOUT = 5;			//hid发送不足一帧时，等待HID_SEND_TIMEOUT周期进行发送
u8 hid_datatemp[256];					//hid环形缓冲区
u8 hid_datatemp_begin = 0;		//环形缓冲区数据指针，指向应当发送的数据
u8 hid_datatemp_end = 0;			//环形缓冲区数据结尾

void Usb_Hid_Adddata(u8 *dataToSend , u8 length)
{
	u8 i=0;
	for(i=0; i<length; i++)
	{
		hid_datatemp[hid_datatemp_end++] = dataToSend[i];
	}
}


void Usb_Hid_Send (void)
{
	static u8 notfull_timeout=0;
	u8 i;
	if(hid_datatemp_end > hid_datatemp_begin)
	{
		if((hid_datatemp_end - hid_datatemp_begin) >= 63)
		{
			notfull_timeout = 0;
			USB_Receive_Buffer[0] = 63;
			for(i=0; i<63; i++)
			{
				USB_Receive_Buffer[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			UserToPMABufferCopy(USB_Receive_Buffer, ENDP2_TXADDR, 64);
			SetEPTxValid(ENDP2);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				USB_Receive_Buffer[0] = hid_datatemp_end - hid_datatemp_begin;
				for(i=0; i<63; i++)
				{
					if(i<hid_datatemp_end - hid_datatemp_begin)
						USB_Receive_Buffer[i+1] = hid_datatemp[hid_datatemp_begin+i];
					else
						USB_Receive_Buffer[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				UserToPMABufferCopy(USB_Receive_Buffer, ENDP2_TXADDR, 64);
				SetEPTxValid(ENDP2);
			}
		}
	}
	else if(hid_datatemp_end < hid_datatemp_begin)
	{
		if((256 - hid_datatemp_begin + hid_datatemp_end) >= 63)
		{
			notfull_timeout = 0;
			USB_Receive_Buffer[0] = 63;
			for(i=0; i<63; i++)
			{
				USB_Receive_Buffer[i+1] = hid_datatemp[hid_datatemp_begin++];
			}
			UserToPMABufferCopy(USB_Receive_Buffer, ENDP2_TXADDR, 64);
			SetEPTxValid(ENDP2);
		}
		else
		{
			notfull_timeout++;
			if(notfull_timeout == HID_SEND_TIMEOUT)
			{
				notfull_timeout = 0;
				USB_Receive_Buffer[0] = 256 - hid_datatemp_begin + hid_datatemp_end;
				for(i=0; i<63; i++)
				{
					if(i<256 - hid_datatemp_begin + hid_datatemp_end)
						USB_Receive_Buffer[i+1] = hid_datatemp[(u8)(hid_datatemp_begin+i)];
					else
						USB_Receive_Buffer[i+1] = 0;
				}
				hid_datatemp_begin = hid_datatemp_end;
				UserToPMABufferCopy(USB_Receive_Buffer, ENDP2_TXADDR, 64);
				SetEPTxValid(ENDP2);
			}
		}
	}
}


/*********************************END OF FILE**********************************/
