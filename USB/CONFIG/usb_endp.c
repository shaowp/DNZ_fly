/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : usb_endp.c
* Author             : MCD Application Team
* Version            : V3.0.1
* Date               : 04/27/2009
* Description        : Endpoint routines
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_desc.h"
#include "ANO_USART.h"
#include "usbio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t USB_Receive_Buffer[REPORT_COUNT];
uint8_t USB_Send_Buffer[REPORT_COUNT];
volatile uint8_t USB_Received_Flag = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : EP1_IN_Callback.
* Description    : EP1 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
}
/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t data[64];
uint32_t i = 0, ret = 0;
void EP1_OUT_Callback(void)
{

#ifndef STM32F10X_CL
	PMAToUserBufferCopy(USB_Receive_Buffer, ENDP1_RXADDR, REPORT_COUNT);
	SetEPRxStatus(ENDP1, EP_RX_VALID);
	USB_Received_Flag = 1;

	/**************************************************************************/
	//fuck_USB_REC
	//USB中断接收
	//ret = USB_GetData(data, sizeof(data));
	// printf("usb get data %d byte data\n\r", ret);
	ret = sizeof(USB_Receive_Buffer);
	for (i = 0; i < ret; i++)
	{
		ANO_DT_Data_Receive_Prepare(USB_Receive_Buffer[i]);
		//printf("0x%02X ", data[i]);
	}
	//表示一帧数据接收完成
	/**************************************************************************/

#else
	USB_SIL_Read(EP1_OUT, USB_Receive_Buffer);
	USB_Received_Flag = 1;
#endif
}
/*******************************************************************************
* Function Name  : EP2_IN_Callback.
* Description    : EP2 IN Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP2_IN_Callback(void)
{
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
