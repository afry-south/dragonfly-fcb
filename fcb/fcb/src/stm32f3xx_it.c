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
  /* Get hard fault information */
//  unsigned int stacked_r0;
//  unsigned int stacked_r1;
//  unsigned int stacked_r2;
//  unsigned int stacked_r3;
//  unsigned int stacked_r12;
//  unsigned int stacked_lr;
//  unsigned int stacked_pc;
//  unsigned int stacked_psr;
//  uint32_t* hardfault_args = (uint32_t*) 0x20000400;
//
//  asm( "TST LR, #4 \n"
//      "ITTE EQ \n"
//      "MRSEQ R0, MSP \n"
//      "ADDEQ R0, #8 \n"
//      "MRSNE R0, PSP \n");
//
//  stacked_r0 = ((unsigned long) hardfault_args[0]);
//  stacked_r1 = ((unsigned long) hardfault_args[1]);
//  stacked_r2 = ((unsigned long) hardfault_args[2]);
//  stacked_r3 = ((unsigned long) hardfault_args[3]);
//
//  stacked_r12 = ((unsigned long) hardfault_args[4]);
//  stacked_lr = ((unsigned long) hardfault_args[5]);
//  stacked_pc = ((unsigned long) hardfault_args[6]);
//  stacked_psr = ((unsigned long) hardfault_args[7]);

  // TODO: Print Hard fault message

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
void PVD_IRQHandler(void)
{
  HAL_PWR_PVD_IRQHandler();
}

/**
  * @brief  This function handles the PRIMARY_RECEIVER_TIM timer interrupt request.
  * @param  None
  * @retval None
  */
void PRIMARY_RECEIVER_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&PrimaryReceiverTimHandle);
}

/**
  * @brief  This function handles the AUX_RECEIVER_TIM timer interrupt request.
  * @param  None
  * @retval None
  */
void AUX_RECEIVER_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&AuxReceiverTimHandle);
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
