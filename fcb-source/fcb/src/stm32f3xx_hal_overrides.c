/******************************************************************************
 * @author  Dragonfly
 * @brief   This module contains override functions for weak defined fuctions in
 *          the STM32F3xx HAL Driver library.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static __IO uint32_t sysTick;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  This function is called to increment  a global variable "uwTick"
  *         used as application time base.
  * @note   In the default implemetation, this variable is incremented each 1ms
  *         in Systick ISR.
 *         The function is declared as __Weak  to be overwritten  in case of other
 *         implementations  in user file.
 * @retval None
 */
__weak void HAL_IncTick(void) {
    sysTick++;
}

/**
 * @brief  Provides a tick value in millisecond. It has been modified to prevent an
 *         RTOS thread from hanging on timeout loops when calling the HAL library.
 * @retval tick value
 */
__weak uint32_t HAL_GetTick(void) {
    if (SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) // Interrupt is Enabled
    {
        if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)     // Interrupt is Blocked
        {
            SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;        // Clear Pend Bit
            sysTick++;
        }
    } else if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
        sysTick++;                                     // Interrupt is Disabled
    }

    return sysTick;
}
