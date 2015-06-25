/********************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    13-June-2014
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private define ------------------------------------------------------------*/

/* USB handler declaration */
extern USBD_HandleTypeDef  hUSBDDevice;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

/* Private variables ---------------------------------------------------------*/

uint8_t USBCOMRxBuffer[CDC_DATA_FS_IN_PACKET_SIZE]; /* Receive Data over USB stored in this buffer */
uint8_t USBCOMTxBuffer[CDC_DATA_FS_OUT_PACKET_SIZE]; /* Transmit Data over USB (CDC interface) stored in this buffer */ // TODO: Is this even used?!

extern xQueueHandle usbComRxQueue;

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  CDC_Itf_Init,
  CDC_Itf_DeInit,
  CDC_Itf_Control,
  CDC_Itf_Receive
};

USBD_CDC_LineCodingTypeDef LineCoding =
  {
    115200, /* baud rate */
    0x00,   /* stop bits-1 */
    0x00,   /* parity - none */
    0x08    /* nb. of data bits 8 */
  };

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CDC_Itf_Init callback
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /*# Set CDC Buffers ####################################################### */
  USBD_CDC_SetTxBuffer(&hUSBDDevice, USBCOMTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&hUSBDDevice, USBCOMRxBuffer);

  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_DeInit callback
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_DeInit(void)
{
  return (USBD_OK);
}

/**
  * @brief  CDC_Itf_Control callback
  *         Manage the CDC class requests
  * @param  Cmd: Command code            
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format     = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype   = pbuf[6];
    /* Add your code here */
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype;
    
    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
}

/**
  * @brief  USB CDC receive callback. Data received over USB OUT endpoint sent over
  *         CDC interface through this function.
  * @param  Buf: Buffer of data
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
  uint8_t result = USBD_OK;

  // TODO: Check if Len can be > 64?

  if(hUSBDDevice.dev_state == USBD_STATE_CONFIGURED)
    {
      result = USBD_CDC_ReceivePacket(&hUSBDDevice);
      if(result == USBD_OK)
        {
        /* # Add to RTOS queue #### */
        portBASE_TYPE xHigherPriorityTaskWoken;
        // We have not woken a task at the start of the ISR.
        xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(usbComRxQueue, USBCOMRxBuffer, xHigherPriorityTaskWoken);
      }
    }
  else
    result = USBD_FAIL;

  return result;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Data transmit over USB IN endpoint sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  if(hUSBDDevice.dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CDC_SetTxBuffer(&hUSBDDevice, Buf, Len);
      result = USBD_CDC_TransmitPacket(&hUSBDDevice);
    }
  else
    result = USBD_FAIL;
  return result;
}

/**
  * @brief  Send a string over the USB IN endpoint CDC com port interface
  * @param  sendString : Reference to the string to be sent
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
USBD_StatusTypeDef USBComSendString(char* sendString)
{
  return CDC_Transmit_FS((uint8_t*)sendString, strlen(sendString));
}


/**
  * @brief  Send data over the USB IN endpoint CDC com port interface
  * @param  sendData : Reference to the data to be sent
  * @param  sendDataSize : Size of data to be sent
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
USBD_StatusTypeDef USBComSendData(uint8_t* sendData, uint16_t sendDataSize)
{
  return CDC_Transmit_FS(sendData, sendDataSize);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
//static void Error_Handler(void)
//{
//  /* Add your own code here */
//}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

