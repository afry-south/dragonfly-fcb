/**
 ******************************************************************************
 * @file    stm32f3xx_it.h
 * @author  Dragonfly
 * @version V1.0.0
 * @date    2015-08-12
 * @brief   This file contains the headers of the interrupt handlers.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "receiver.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void EXTI0_IRQHandler(void);
void PVD_IRQHandler(void);
void PRIMARY_RECEIVER_TIM_IRQHandler(void);
void AUX_RECEIVER_TIM_IRQHandler(void);

#if defined (USE_USB_INTERRUPT_DEFAULT)
void USB_LP_CAN_RX0_IRQHandler(void);
void USBWakeUp_IRQHandler(void);
#elif defined (USE_USB_INTERRUPT_REMAPPED)
void USB_LP_IRQHandler(void);
void USBWakeUp_RMP_IRQHandler(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* __STM32F3xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
