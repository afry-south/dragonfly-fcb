/*****************************************************************************
 * @file    motor_control.h
 * @brief   Header file for UART handling module
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Definition for USARTx clock resources */
#define UART                           	USART1
#define UART_CLK_ENABLE()             	__USART2_CLK_ENABLE();
#define DMA_CLK_ENABLE()				__DMA1_CLK_ENABLE()
#define UART_RX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()
#define UART_TX_GPIO_CLK_ENABLE()		__GPIOA_CLK_ENABLE()

/* Definition for UART Pins */
#define UART_TX_PIN                    	GPIO_PIN_2
#define UART_TX_GPIO_PORT              	GPIOA
#define UART_TX_AF                     	GPIO_AF7_USART2
#define UART_RX_PIN                    	GPIO_PIN_3
#define UART_RX_GPIO_PORT              	GPIOA
#define UART_RX_AF                     	GPIO_AF7_USART2

/* Definition for UART's DMA */
#define UART_TX_DMA_STREAM            	DMA1_Channel4
#define UART_RX_DMA_STREAM              DMA1_Channel5

/* Definition for UART's NVIC */
#define UART_DMA_TX_IRQn               	DMA1_Channel4_IRQn
#define UART_DMA_RX_IRQn                DMA1_Channel5_IRQn
#define UART_DMA_TX_IRQHandler          DMA1_Channel4_IRQHandler
#define UART_DMA_RX_IRQHandler          DMA1_Channel5_IRQHandler

/* UART Setup values */
#define UART_BAUDRATE					115200
#define UART_WORDLENGTH					UART_WORDLENGTH_8B
#define UART_STOPBITS					UART_STOPBITS_1
#define UART_PARITY						UART_PARITY_NONE

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void UartConfig(void);

#endif /* __UART_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
