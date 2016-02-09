/*****************************************************************************
 * @file    uart.c
 * @brief   Module contains UART initialization and handling functions.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uart.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes and configures UART.
 * @param  None.
 * @retval None.
 */
void UartConfig(void) {
	UART_HandleTypeDef UartHandle;

	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as defined by header file definitions
	 * Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance        = UART;
	UartHandle.Init.BaudRate   = UART_BAUDRATE;
	UartHandle.Init.WordLength = UART_WORDLENGTH;
	UartHandle.Init.StopBits   = UART_STOPBITS;
	UartHandle.Init.Parity     = UART_PARITY;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/* Private functions ---------------------------------------------------------*/


/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
