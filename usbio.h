/**
  ******************************************************************************
  * @file    usbio.h
  * $Author: wdluo $
  * $Revision: 67 $
  * $Date:: 2012-08-15 19:00:29 +0800 #$
  * @brief   USBÉÏ²ãº¯ÊýÉùÃ÷.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, ViewTool</center>
  *<center><a href="http:\\www.viewtool.com">http://www.viewtool.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _USBIO_H_
#define _USBIO_H_

/* Exported Functions --------------------------------------------------------*/
extern uint32_t USB_SendData(uint8_t *data,uint32_t dataNum);
extern uint32_t USB_GetData(uint8_t *data,uint32_t dataNum);

#endif //_USBIO_H_
