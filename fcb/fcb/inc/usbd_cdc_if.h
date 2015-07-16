/**
  ******************************************************************************
  * @file    usbd_cdc_if.h
  * @author  ÅF Dragonfly - Embedded Systems
  * @version v. 0.1.0
  * @date    2015-07-16
  * @brief   USB CDC Interface header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"

#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void InitUSBCom(void);
USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* data, uint16_t size);
USBD_StatusTypeDef USBComSendString(const char* sendString, const uint32_t maxMutexWaitTicks, const uint32_t maxQueueWaitTicks);
USBD_StatusTypeDef USBComSendData(const uint8_t* sendData, const uint16_t sendDataSize, const uint32_t maxMutexWaitTicks, const uint32_t maxQueueWaitTicks);
void CreateUSBComThreads(void);
void CreateUSBComQueues(void);
void CreateUSBComSemaphores(void);

#endif /* __USBD_CDC_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
