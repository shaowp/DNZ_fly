/**
  ******************************************************************************
  * @file    usbio.c
  * $Author: wdluo $
  * $Revision: 67 $
  * $Date:: 2012-08-15 19:00:29 +0800 #$
  * @brief   USB�ϲ㺯��.
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
  * @brief  ͨ��USB��������
  * @param  data ���ݴ洢�׵�ַ
  * @param  dataNum ���͵������ֽ���
  * @retval ���͵��ֽ���
  */
uint32_t USB_SendData(uint8_t *data, uint32_t dataNum)
{

#ifndef STM32F10X_CL
	//������ͨ��USB���ͳ�ȥ
	UserToPMABufferCopy(data, ENDP2_TXADDR, dataNum);
	//SetEPTxCount(ENDP2, REPORT_COUNT);
	SetEPTxValid(ENDP2);
#else
	USB_SIL_Write(EP2_IN, data, dataNum);
#endif
	return dataNum;
}
/**
  * @brief  ���մ�USB��ȡ������
  * @param  data ���ݴ洢�׵�ַ
  * @param  dataNum ׼����ȡ�������ֽ���
  * @retval ��ȡ���ֽ���
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
u8 HID_SEND_TIMEOUT = 5;			//hid���Ͳ���һ֡ʱ���ȴ�HID_SEND_TIMEOUT���ڽ��з���
u8 hid_datatemp[256];					//hid���λ�����
u8 hid_datatemp_begin = 0;		//���λ���������ָ�룬ָ��Ӧ�����͵�����
u8 hid_datatemp_end = 0;			//���λ��������ݽ�β

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
