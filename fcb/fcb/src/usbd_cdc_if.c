/**
  ******************************************************************************
  * @file    usbd_cdc_if.c
  * @author  ÅF Dragonfly - Embedded Systems
  * @version v. 0.1.0
  * @date    2015-07-16
  * @brief   USB CDC Interface functions for the ÅF Dragonfly quadrotor UAV
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "main.h"
#include "fifo_buffer.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint16_t dataTxSize;
  volatile FIFOBuffer_TypeDef* FIFOBuffer;
  xSemaphoreHandle* FIFOBufferMutex;
}UsbComPortTxQueueItem_TypeDef;

/* Private define ------------------------------------------------------------*/
#define USB_COM_TX_BUFFER_SIZE          1024
#define USB_COM_RX_BUFFER_SIZE          1024

#define USB_COM_RX_THREAD_PRIO          1
#define USB_COM_TX_THREAD_PRIO          1

#define USB_COM_TX_QUEUE_ITEMS          16

/* USB handler declaration */
USBD_HandleTypeDef hUSBDDevice;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* rxData, uint32_t *rxDataLen);

/* Private variables ---------------------------------------------------------*/
uint8_t USBCOMRxPacketBuffer[CDC_DATA_HS_OUT_PACKET_SIZE];

/* USB CDC Receive FIFO buffer */
uint8_t USBCOMRxBufferArray[USB_COM_RX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef USBCOMRxFIFOBuffer;

/* USB CDC Transmit FIFO buffer */
uint8_t USBCOMTxBufferArray[USB_COM_TX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef USBCOMTxFIFOBuffer;

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

/* USB RTOS variables */
xTaskHandle USB_ComPortRx_Thread_Handle;
xTaskHandle USB_ComPortTx_Thread_Handle;

xQueueHandle usbComTxQueue;

xSemaphoreHandle USBCOMRxDataSem;
xSemaphoreHandle USBCOMTxBufferMutex;
xSemaphoreHandle USBTxMutex;

/* Private functions ---------------------------------------------------------*/
static void USB_ComPort_RX_Thread(void const *argument);
static void USB_ComPort_TX_Thread(void const *argument);

/**
  * @brief  CDC_Itf_Init callback
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Itf_Init(void)
{
  /*# Set CDC Buffers ####################################################### */
  USBD_CDC_SetRxBuffer(&hUSBDDevice, USBCOMRxPacketBuffer);

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
static int8_t CDC_Itf_Receive(uint8_t* rxData, uint32_t *rxDataLen)
{
  uint8_t result = USBD_OK;
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE; // We have not woken a task at the start of the ISR.

  if(hUSBDDevice.dev_state == USBD_STATE_CONFIGURED)
    {
      result = USBD_CDC_ReceivePacket(&hUSBDDevice);
      if(result == USBD_OK)
        {
          if(BufferPutData(&USBCOMRxFIFOBuffer, rxData, *rxDataLen))
            {
              /* # Signal RX thread that new USB CDC data has arrived #### */
              xSemaphoreGiveFromISR(USBCOMRxDataSem, &xHigherPriorityTaskWoken);
            }
          else
            {
              result = USBD_FAIL;
            }
        }
    }
  else
    result = USBD_FAIL;

  return result;
}

/**
  * @brief  Thread code handles the USB Com Port Rx communication
  * @param  argument : Unused parameter
  * @retval None
  */
static void USB_ComPort_RX_Thread(void const *argument)
{
  (void) argument;

  for (;;)
    {
      /* Wait forever for incoming data over USB by pending on the USB Rx queue */
      if(pdPASS == xSemaphoreTake(USBCOMRxDataSem, portMAX_DELAY))
        {
          // TODO Get new contents from FIFO buffer, perhaps scan it for commands X amount of bytes before updating indicies?
          // TODO Do command/data parsing

          USBComSendString("Hello, this is Dragonfly!\n", portMAX_DELAY, portMAX_DELAY);
        }
    }
}

/**
  * @brief  Thread code handles the USB Com Port Tx communication
  * @param  argument : Unused parameter
  * @retval None
  */
static void USB_ComPort_TX_Thread(void const *argument)
{
  (void) argument;

  static UsbComPortTxQueueItem_TypeDef CompPortTxQueueItem;

  for (;;)
    {
      /* Wait forever for incoming data over USB by pending on the USB Tx queue */
      if(pdPASS == xQueueReceive(usbComTxQueue, &CompPortTxQueueItem, portMAX_DELAY))
        {
          if(xSemaphoreTake(USBTxMutex, portMAX_DELAY) == pdPASS) // Pend on mutex while previous transmission is in progress
            {
              if(xSemaphoreTake(*CompPortTxQueueItem.FIFOBufferMutex, portMAX_DELAY) == pdPASS)
                {
                  uint8_t* txDataPtr;
                  uint16_t tmpSize = BufferGetData(CompPortTxQueueItem.FIFOBuffer, &txDataPtr, CompPortTxQueueItem.dataTxSize);
                  CDC_Transmit_FS(txDataPtr, tmpSize);

                  // If not all data received from buffer (due to FIFO wrap-around), get the rest
                  if(tmpSize < CompPortTxQueueItem.dataTxSize)
                    {
                      tmpSize = BufferGetData(CompPortTxQueueItem.FIFOBuffer, &txDataPtr, CompPortTxQueueItem.dataTxSize-tmpSize);
                      CDC_Transmit_FS(txDataPtr, tmpSize);
                    }
                  xSemaphoreGive(*CompPortTxQueueItem.FIFOBufferMutex);
                }

              xSemaphoreGive(USBTxMutex); // Give mutex when transmission completed
            }
        }
    }
}

/* Exported functions --------------------------------------------------------*/

void InitUSBCom(void)
{
  /* Create CDC RX FIFO Buffer */
  FIFOBufferInit(&USBCOMRxFIFOBuffer, USBCOMRxBufferArray, sizeof(USBCOMRxBufferArray));

  /* Create CDC TX FIFO Buffer */
  FIFOBufferInit(&USBCOMTxFIFOBuffer, USBCOMTxBufferArray, sizeof(USBCOMTxBufferArray));

  /* Init Device Library */
  USBD_Init(&hUSBDDevice, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&hUSBDDevice, &USBD_CDC);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&hUSBDDevice, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&hUSBDDevice);
}

/**
  * @brief  Data transmit over USB IN endpoint sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* data, uint16_t size)
{
  uint8_t result = USBD_OK;

  if(hUSBDDevice.dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CDC_SetTxBuffer(&hUSBDDevice, data, size);
      while((result = USBD_CDC_TransmitPacket(&hUSBDDevice)) == USBD_BUSY) { } // Wait while busy with previous transmission
      if(CDC_DATA_FS_MAX_PACKET_SIZE)
        {
          /*
           * According to the USB specification, a packet size of 64 bytes (CDC_DATA_FS_MAX_PACKET_SIZE)
           * gets held at the USB host until the next packet is sent.  This is because a
           * packet of maximum size is considered to be part of a longer chunk of data, and
           * the host waits for all data to arrive (ie, waits for a packet < max packet size).
           * To flush a packet of exactly max packet size, we need to send a zero-size packet or
           * short packet of less than CDC_DATA_FS_MAX_PACKET_SIZE.
           * See eg http://www.cypress.com/?id=4&rID=92719
           * */
          USBD_CDC_SetTxBuffer(&hUSBDDevice, NULL, 0);      // Zero-length packet (ZLP)
          while((result = USBD_CDC_TransmitPacket(&hUSBDDevice)) == USBD_BUSY) { } // Wait while busy with previous transmission
        }
    }
  else
    {
      result = USBD_FAIL; // USB not connected and/or configured
    }
  return result;
}

/**
  * @brief  Send a string over the USB IN endpoint CDC com port interface
  * @param  sendString : Reference to the string to be sent
  * @param  maxMutexWaitTicks : Max ticks to wait for the FIFO buffer mutex
  * @param  maxQueueWaitTicks : Max ticks to wait for the queue
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
USBD_StatusTypeDef USBComSendString(const char* sendString, const uint32_t maxMutexWaitTicks, const uint32_t maxQueueWaitTicks)
{
  return USBComSendData((uint8_t*)sendString, strlen(sendString), maxMutexWaitTicks, maxQueueWaitTicks);
}


/**
  * @brief  Send data over the USB IN endpoint CDC com port interface
  * @param  sendData : Reference to the data to be sent
  * @param  sendDataSize : Size of data to be sent
  * @param  maxMutexWaitTicks : Max ticks to wait for the FIFO buffer mutex
  * @param  maxQueueWaitTicks : Max ticks to wait for the queue
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
USBD_StatusTypeDef USBComSendData(const uint8_t* sendData, const uint16_t sendDataSize, const uint32_t maxMutexWaitTicks, const uint32_t maxQueueWaitTicks)
{
  UsbComPortTxQueueItem_TypeDef CompPortTxQueueItem;
  USBD_StatusTypeDef result = USBD_OK;

  if(xSemaphoreTake(USBCOMTxBufferMutex, maxMutexWaitTicks) == pdPASS)
    {
      // Mutex obtained - access the shared resource
      if(BufferPutData(&USBCOMTxFIFOBuffer, sendData, sendDataSize))
        {
          CompPortTxQueueItem.FIFOBuffer = &USBCOMTxFIFOBuffer;
          CompPortTxQueueItem.FIFOBufferMutex = &USBCOMTxBufferMutex;
          CompPortTxQueueItem.dataTxSize = sendDataSize;

          // The Tx Queue needs to be accessed in critical region since we don't want items from the same FIFO entering it in the wrong order!
          if(xQueueSend(usbComTxQueue, &CompPortTxQueueItem, maxQueueWaitTicks) != pdPASS)
            {
              // Queue full, delete data from FIFO
              BufferDeleteLastEnteredBytes(&USBCOMTxFIFOBuffer, sendDataSize);
              result = USBD_FAIL;
            }
          xSemaphoreGive(USBCOMTxBufferMutex);      // We have finished accessing the shared resource. Release the mutex.
        }
      else
        {
          result = USBD_FAIL;
        }
    }
  else
    {
      result = USBD_FAIL;
    }

  return result;
}

void CreateUSBComThreads(void)
{
  /* USB Virtual Com Port Rx handler thread creation
   * Task function pointer: USB_ComPort_RX_Thread
   * Task name: USB_COM_RX
   * Stack depth: configMINIMAL_STACK_SIZE (128 byte)
   * Parameter: NULL
   * Priority: USB_COM_RX_THREAD_PRIO ([0, inf] possible)
   * Handle: USB_ComPortRx_Thread_Handle
   * */
  if(pdPASS != xTaskCreate((pdTASK_CODE)USB_ComPort_RX_Thread, (signed portCHAR*)"USB_COM_RX", configMINIMAL_STACK_SIZE, NULL, USB_COM_RX_THREAD_PRIO, &USB_ComPortRx_Thread_Handle))
    {
      Error_Handler();
    }

  /* USB Virtual Com Port Tx handler thread creation
   * Task function pointer: USB_ComPort_RX_Thread
   * Task name: USB_COM_TX
   * Stack depth: configMINIMAL_STACK_SIZE (128 byte)
   * Parameter: NULL
   * Priority: USB_COM_TX_THREAD_PRIO ([0, inf] possible)
   * Handle: USB_ComPortTx_Thread_Handle
   * */
  if(pdPASS != xTaskCreate((pdTASK_CODE)USB_ComPort_TX_Thread, (signed portCHAR*)"USB_COM_TX", configMINIMAL_STACK_SIZE, NULL, USB_COM_TX_THREAD_PRIO, &USB_ComPortTx_Thread_Handle))
    {
      Error_Handler();
    }
}

void CreateUSBComQueues(void)
{
  /* # Create queue for outgoing data ####################################### */
  usbComTxQueue = xQueueCreate(USB_COM_TX_QUEUE_ITEMS, sizeof(UsbComPortTxQueueItem_TypeDef));

  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(usbComTxQueue, (signed char*) "usbComTxQueue");
}

void CreateUSBComSemaphores(void)
{
  USBCOMRxDataSem = xSemaphoreCreateBinary();
  if( USBCOMRxDataSem == NULL )
    {
      Error_Handler();
    }

  USBCOMTxBufferMutex = xSemaphoreCreateMutex();
  if( USBCOMTxBufferMutex == NULL )
    {
      Error_Handler();
    }

  /* Create binary semaphore to synchronize USB CDC class output. The semaphore is taken when device is
   * transmitting (USB CDC busy) and given when transfer has completed. */
  USBTxMutex = xSemaphoreCreateMutex();
  if( USBTxMutex == NULL )
    {
      Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
//static void Error_Handler(void)
//{
//  /* Add your own code here */
// TODO USB error handler?
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

