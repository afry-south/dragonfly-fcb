/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  ÅF Embedded Systems Syd
  * @version V1.0.0
  * @date    24-May-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
#include "rc_input.h"

/** @addtogroup STM32F3-Discovery_Demo STM32F3-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd;
extern USBD_HandleTypeDef hUSBDDevice;

extern TIM_HandleTypeDef PrimaryReceiverTimHandle;
extern TIM_HandleTypeDef AuxReceiverTimHandle;

/* Private function prototypes -----------------------------------------------*/


/******************************************************************************/
/*             Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

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
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F3xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f3xx.s).                                               */
/******************************************************************************/

void PRIMARY_RECEIVER_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&PrimaryReceiverTimHandle);
}

void AUX_RECEIVER_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&AuxReceiverTimHandle);
}

#ifdef TODO
/*
 * @brief       Timer 2 interrupt handler.
 */
//void TIM2_IRQHandler(void)
//{
//
//  /* If interrupt concerns TIM2 CH1 */
//  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
//      UpdateThrottleChannel();
//    }
//  /* If interrupt concerns TIM2 CH2 */
//  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
//      UpdateAileronChannel();
//    }
//
//  /* If interrupt concerns TIM2 CH3 */
//  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
//      UpdateElevatorChannel();
//    }
//
//  /* If interrupt concerns TIM2 CH4 */
//  if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
//      UpdateRudderChannel();
//    }
//}
#endif

#ifdef TODO
/*
 * @brief       Timer 3 interrupt handler.
 */
//void TIM3_IRQHandler(void)
//{
//  /* If interrupt concerns TIM3 CH1 */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//      UpdateGearChannel();
//    }
//  /* If interrupt concerns TIM3 CH2 */
//  if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
//    {
//      TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//      UpdateAuxiliaryChannel();
//    }
//}
#endif

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
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
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
