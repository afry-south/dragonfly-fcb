/******************************************************************************
 * @file    fcb/main.c
 * @author  ÅF Dragonfly
 * @version v. 0.0.2
 * @date    2015-05-28
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common.h"
#include "motor_control.h"
#include "flight_control.h"
#include "sensors.h"
#include "receiver.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private define ------------------------------------------------------------*/
#define USB_COM_RX_THREAD_PRIO  1
#define USB_COM_TX_THREAD_PRIO  1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef hUSBDDevice;
volatile uint8_t UserButtonPressed;

xTaskHandle USB_ComPortRx_Thread_Handle;
xTaskHandle USB_ComPortTx_Thread_Handle;

xQueueHandle usbComRxQueue;
xQueueHandle usbComTxQueue;

// TODO Make struct with buffer and mutex?
uint8_t testTxData[128];
xSemaphoreHandle testTxDataMutex;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void System_Init(void);
static void RTOS_Init(void);
static void PVD_Config(void);

static void USB_ComPort_RX_Thread(void const *argument);
static void USB_ComPort_TX_Thread(void const *argument);

/* Called every system tick to drive the RTOS */
extern void xPortSysTickHandler(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
  /* At this stage the microcontroller clock setting is already configured,
   * this is done through SystemInit() function which is called from startup
   * file (startup_stm32f303.c) before to branch to application main.
   * To reconfigure the default setting of SystemInit() function, refer to
   * system_stm32f30x.c file
   */

  /* Init system at low level */
  System_Init();

  /* Initialize RTOS tasks */
  RTOS_Init();

  while (1)
    {
      ToggleLEDs();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on */
  LEDs_Off();
  BSP_LED_On(LED3);

  // TODO: Based on the current state of the quadrotor, appropriate
  // action should be taken (i.e. attempt to land and shut down
  // motors within 20 seconds)

  while(1)
  {
  }
}

/**
  * @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
   UserButtonPressed++;
   if (UserButtonPressed > 0x7)
    {
      UserButtonPressed = 0x0;
    }
  }
}

/**
  * @brief  PWR PVD interrupt callback
  * @param  none
  * @retval none
  */
void HAL_PWR_PVDCallback(void)
{
  /* Voltage drop detected - Go to error handler */
  Error_Handler();
}

/**
 * @brief  SYSTICK callback
 * @param  None
 * @retval None
 */
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    xPortSysTickHandler();
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes system at low level, setting up clocks and configuring
  *         peripheral operation.
  * @param  None
  * @retval None
  */
static void System_Init(void)
{
  /* STM32F3xx HAL library initialization:
         - Configure the Flash prefetch
         - Systick timer is configured by default as source of time base, but user
           can eventually implement his proper time base source (a general purpose
           timer for example or other time source), keeping in mind that Time base
           duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
           handled in milliseconds basis.
         - Set NVIC Group Priority to 4
         - Low Level Initialization
   */
  HAL_Init();

  /* Initialize Programmable Voltage Detection (PVD) */
  PVD_Config();

  /* Configure the system clock to 72 Mhz */
  SystemClock_Config();

  /* Init Device Library */
  USBD_Init(&hUSBDDevice, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&hUSBDDevice, &USBD_CDC);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&hUSBDDevice, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&hUSBDDevice);

  /* Init on-board LEDs */
  Init_LEDs();

  /* Init User button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

#ifdef TODO
  /* Setup sensors */
  GyroConfig();
  CompassConfig();
  InitPIDControllers();
#endif

  /* Setup motor output timer */
  MotorControl_Config();

  /* Setup receiver timers for receiver input */
  ReceiverInput_Config();
}

/**
  * @brief  Initializes the Real-time operating system (RTOS) and starts the
  *         OS kernel
  * @param  None
  * @retval None
  */
static void RTOS_Init(void)
{
  /* # CREATE THREADS ####################################################### */

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

  /* # CREATE QUEUES ######################################################## */
  usbComRxQueue = xQueueCreate(16, CDC_DATA_FS_IN_PACKET_SIZE); // TODO change 16 to a defined item number

  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(usbComRxQueue, (signed char*) "usbComRxQueue");

  usbComTxQueue = xQueueCreate(8, CDC_DATA_FS_OUT_PACKET_SIZE); // TODO change 16 to a defined item number

  /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
  vQueueAddToRegistry(usbComTxQueue, (signed char*) "usbComTxQueue");

  /* # CREATE SEMAPHORES AND MUTEXES ######################################## */
  testTxDataMutex = xSemaphoreCreateMutex();
  if( testTxDataMutex == NULL )
    {
      Error_Handler();
    }

  /* # Start the RTOS scheduler #############################################
   *
   * since we use heap1.c, we must create all tasks and queues before the OS kernel
   * is started according to ST UM1722 manual section 1.6.
   */
  vTaskStartScheduler();
}

/**
  * @brief  Thread code handles the USB Com Port Rx communication
  * @param  argument : Unused parameter
  * @retval None
  */
static void USB_ComPort_RX_Thread(void const *argument)
{
  (void) argument;

  static uint8_t usbRxDataPacket[CDC_DATA_FS_IN_PACKET_SIZE];

  for (;;)
    {
      /* Wait forever for incoming data over USB */
      if(pdPASS == xQueueReceive(usbComRxQueue, usbRxDataPacket, portMAX_DELAY))
        {
          // Here usbRxDataPacket contains the sent data
          // TODO Do some parsing, perhaps we should keep a thread local buffer with more than 64 bytes?
          // In case data/commands are longer than this?

          if( xSemaphoreTake( testTxDataMutex, 10 ) == pdTRUE )
            {
              // We were able to obtain the semaphore and can now access the
              // shared resource.

              memcpy(testTxData, "Welcome to the Dragonfly UAV Flight Control program!\nDeveloped by ÅF Embedded Systems South\n\0", 93);
              UsbComPortTxData_TypeDef txData;
              txData.dataPtr = &testTxData[0];
              txData.dataSize = strlen((char*)testTxData);
              xQueueSend(usbComTxQueue, &txData, portMAX_DELAY);

              // We have finished accessing the shared resource.  Release the
              // semaphore.
              xSemaphoreGive( testTxDataMutex );
            }
          BSP_LED_Toggle(LED9);
        }
      // TODO: Check if not pdPASS
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

  // static uint8_t usbTxDataPacket[CDC_DATA_FS_OUT_PACKET_SIZE];
  static UsbComPortTxData_TypeDef txData;

  for (;;)
    {
      /* Wait forever for incoming data over USB */
      if(pdPASS == xQueueReceive(usbComTxQueue, &txData, portMAX_DELAY))
        {
          // TODO: Fix this issue below - detect end of transmission and flush with empty packet
          // According to the USB specification, a packet size of 64 bytes (CDC_DATA_FS_MAX_PACKET_SIZE)
          // gets held at the USB host until the next packet is sent.  This is because a
          // packet of maximum size is considered to be part of a longer chunk of data, and
          // the host waits for all data to arrive (ie, waits for a packet < max packet size).
          // To flush a packet of exactly max packet size, we need to send a zero-size packet.
          // See eg http://www.cypress.com/?id=4&rID=92719

          if( xSemaphoreTake( testTxDataMutex, 100 ) == pdTRUE )
            {
              // We were able to obtain the semaphore and can now access the
              // shared resource.

              // Here usbTxDataPacket contains the data package to be send - TODO: Sort out so that we don't get max buffer size +1 (exceeding the size)
              if(txData.dataSize % CDC_DATA_FS_OUT_PACKET_SIZE == 0)
                txData.dataSize++;

              CDC_Transmit_FS(txData.dataPtr, txData.dataSize);
              BSP_LED_Toggle(LED8);
              // We have finished accessing the shared resource.  Release the
              // semaphore.
              xSemaphoreGive( testTxDataMutex );
            }
        }

      // TODO Check if not pdPass
    }
}

/**
  * @brief  Configures the Programmable Voltage Detection (PVD) resources.
  * @param  None
  * @retval None
  */
static void PVD_Config(void)
{
  PWR_PVDTypeDef sConfigPVD;

  /*##-1- Enable Power Clock #################################################*/
  __PWR_CLK_ENABLE();

  /*##-2- Configure the NVIC for PVD #########################################*/
  HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);

  /* Configure the PVD level to and generate an interrupt on falling
     edges (Detection level set to 2.47V, refer to the electrical characteristics
     of the device datasheet for more details) */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;
  HAL_PWR_PVDConfig(&sConfigPVD);

  /* Enable the PVD Output */
  HAL_PWR_EnablePVD();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLLMUL                         = RCC_PLL_MUL9 (9)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Configures the USB clock */
  HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);
  RCC_PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;   // 72/1.5 = 48 MHz USB clock
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     // APB1 is limited to 36 MHz according to reference manual
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
    {
    }
}
#endif

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
