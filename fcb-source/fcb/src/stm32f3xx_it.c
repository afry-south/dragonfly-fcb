/*
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  �F Embedded Systems Syd
  * @version V1.0.0
  * @date    24-May-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_it.h"
#include "stm32f3_discovery.h"

#include "fcb_error.h"
#include "task_status.h"
#include "receiver.h"
#include "state_estimation.h"
#include "uart.h"

/** @addtogroup STM32F3-Discovery_Demo STM32F3-Discovery_Demo
 * @{
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd;

/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*             Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void) {
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {

	/* Determine which stack is used */
	__asm volatile
	(
			" tst lr, #4                                                \n"
			" ite eq                                                    \n"
			" mrseq r0, msp                                             \n"
			" mrsne r0, psp                                             \n"
			" ldr r1, [r0, #24]                                         \n"
			" ldr r2, handler2_address_const                            \n"
			" bx r2                                                     \n"
			" handler2_address_const: .word GetRegistersFromStack       \n"
	);

	ErrorHandler();
	while (1) {
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
/*void SVC_Handler(void)
 {
 }*/

/**
 * @brief  This function handles PendSV_Handler exception.
 * @param  None
 * @retval None
 */
/*void PendSV_Handler(void)
 {
 }*/

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
	HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles the PVD Output interrupt request.
 * @param  None
 * @retval None
 */
void PVD_IRQHandler(void) {
	HAL_PWR_PVD_IRQHandler();
}

/**
 * @brief  This function handles the PRIMARY_RECEIVER_TIM timer interrupt request.
 * @param  None
 * @retval None
 */
void PRIMARY_RECEIVER_TIM_IRQHandler(void) {
	HAL_TIM_IRQHandler(&PrimaryReceiverTimHandle);
}

/**
 * @brief  This function handles the AUX_RECEIVER_TIM timer interrupt request.
 * @param  None
 * @retval None
 */
void AUX_RECEIVER_TIM_IRQHandler(void) {
	HAL_TIM_IRQHandler(&AuxReceiverTimHandle);
}

/**
 * @brief  This function handles the TASK_STATUS_TIM timer interrupt request.
 * @param  None
 * @retval None
 */
void TASK_STATUS_TIM_IRQHandler(void) {
	HAL_TIM_IRQHandler(&TaskStatusTimHandle);
}

/**
 * @brief  This function handles the STATE_ESTIMATION_UPDATE_TIM timer interrupt request.
 * @param  None
 * @retval None
 */
void STATE_ESTIMATION_UPDATE_TIM_IRQHandler(void) {
    HAL_TIM_IRQHandler(&StateEstimationTimHandle);
}

/**
 * @brief  This function handles USB Handler.
 * @param  None
 * @retval None
 */
#if defined (USE_USB_INTERRUPT_DEFAULT)
void USB_LP_CAN_RX0_IRQHandler(void)
#elif defined (USE_USB_INTERRUPT_REMAPPED)
void USB_LP_IRQHandler(void)
#endif
{
	HAL_PCD_IRQHandler(&hpcd);
}

/**
 * @brief  This function handles USB WakeUp interrupt request.
 * @param  None
 * @retval None
 */
#if defined (USE_USB_INTERRUPT_DEFAULT)
void USBWakeUp_IRQHandler(void)
#elif defined (USE_USB_INTERRUPT_REMAPPED)
void USBWakeUp_RMP_IRQHandler(void)
#endif
{
	__HAL_USB_EXTI_CLEAR_FLAG();
}

/**
 * @brief  This function handles EXTI0 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
}

void EXTI1_IRQHandler(void) {
  /* gyroscope data ready */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI4_IRQHandler(void)
{
  /* accelerometer data ready */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI2_TS_IRQHandler(void)
{
  /* magnetometer data ready */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "uart.h" and related to DMA channel
  *         used for UART data transmission
  */
void UART_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "uart.h" and related to DMA channel
  *         used for UART data reception
  */
void UART_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
/*void PPP_IRQHandler(void)
 {
 }*/

/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
