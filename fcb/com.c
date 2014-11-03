/**
******************************************************************************
* @file    fcb/com.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter.
*          File contains USB communication functionality.
*          To communicate with PC, STM32 Virtual COM Port Driver is needed.
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "com.h"

/** @addtogroup STM32F3-Discovery_Demo
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t TimingDelay = 0;
__IO uint32_t USBConnectTimeOut = 100;
__IO uint32_t UserButtonPressed = 0;
__IO uint8_t DataReady = 0;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length;
uint8_t Send_Buffer[64];
uint32_t packet_sent = 1;
uint32_t packet_receive = 1;

uint8_t localBuffer[64];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
void rwUSB(void)
{
	if (bDeviceState == CONFIGURED)
	{
		STM_EVAL_LEDOn(LED6);

		CDC_Receive_DATA();

		/* Check to see if we have data yet */
		if (Receive_length  != 0)
		{
			STM_EVAL_LEDOn(LED4);

			int i;
			for (i = 0; i < Receive_length; i++) {
				localBuffer[i] = Receive_Buffer[i]+1;
			}

			CDC_Send_DATA(localBuffer, Receive_length);

			Receive_length = 0;
		}
	}
}

/**
 * @brief  Configure the USB.
 * @param  None
 * @retval None
 */
void initUSB(void)
{
	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();

	USB_Init();

	while ((bDeviceState != CONFIGURED) && (USBConnectTimeOut != 0))
	{
		STM_EVAL_LEDOn(LED8);
	}
	STM_EVAL_LEDOff(LED8);
}
