/**
 ******************************************************************************
 * @file    stm32f3xx_hal_msp.c
 * @author  Dragonfly
 *          Daniel Stenberg
 * @version V1.0.0
 * @date    2015-05-07
 * @brief   HAL MSP (MCU Specific Package) module. These functions are called
 *          from the HAL library.
 *
 @verbatim
 ===============================================================================
 ##### How to use this driver #####
 ===============================================================================
 [..]
 This file is generated automatically by MicroXplorer and eventually modified
 by the user

 @endverbatim
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"

#include "motor_control.h"
#include "receiver.h"

#include "task_status.h"

/** @addtogroup STM32F3xx_HAL_Driver
 * @{
 */

/** @defgroup HAL_MSP HAL MSP module
 * @brief HAL MSP module.
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Exported_Functions HAL MSP Exported Functions
 * @{
 */

/**
 * @brief  Initializes the Global MSP.
 * @retval None
 */
void HAL_MspInit(void) {
	/* NOTE : This function is generated automatically by MicroXplorer and eventually
	 modified by the user
	 */
}

/**
 * @brief  DeInitializes the Global MSP.
 * @retval None
 */
void HAL_MspDeInit(void) {
	/* NOTE : This function is generated automatically by MicroXplorer and eventually
	 modified by the user
	 */
}

/**
 * @brief  Initializes the PPP MSP.
 * @retval None
 */
void HAL_PPP_MspInit(void) {
	/* NOTE : This function is generated automatically by MicroXplorer and eventually
	 modified by the user
	 */
}

/**
 * @brief  DeInitializes the PPP MSP.
 * @retval None
 */
void HAL_PPP_MspDeInit(void) {
	/* NOTE : This function is generated automatically by MicroXplorer and eventually
	 modified by the user
	 */
}

/**
 * @brief CRC MSP Initialization
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc) {
	(void) hcrc; // Avoid compile warning

	/* CRC Peripheral clock enable */
	__CRC_CLK_ENABLE();
}

/**
 * @brief CRC MSP De-Initialization
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc) {
	(void) hcrc; // Avoid compile warning

	/* CRC Peripheral clock disable */
	__CRC_CLK_DISABLE();
}

/**
 * @brief TIM MSP Initialization
 *        This function configures the hardware resources used in this example:
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Primary Receiver TIM Peripheral clock enable */
		PRIMARY_RECEIVER_TIM_CLK_ENABLE();

		/*##-2- Configure the NVIC for PRIMARY_RECEIVER_TIM ########################*/
		HAL_NVIC_SetPriority(PRIMARY_RECEIVER_TIM_IRQn, PRIMARY_RECEIVER_TIM_IRQ_PREEMPT_PRIO,
		PRIMARY_RECEIVER_TIM_IRQ_SUB_PRIO);

		/* Enable the PRIMARY_RECEIVER_TIM global Interrupt */
		HAL_NVIC_EnableIRQ(PRIMARY_RECEIVER_TIM_IRQn);
	} else if (htim->Instance == AUX_RECEIVER_TIM) {

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Aux Receiver TIM Peripheral clock enable */
		AUX_RECEIVER_TIM_CLK_ENABLE();

		/*##-2- Configure the NVIC for AUX_RECEIVER_TIM ############################*/
		HAL_NVIC_SetPriority(AUX_RECEIVER_TIM_IRQn, AUX_RECEIVER_TIM_IRQ_PREEMPT_PRIO, AUX_RECEIVER_TIM_IRQ_SUB_PRIO);

		/* Enable the AUX_RECEIVER_TIM global Interrupt */
		HAL_NVIC_EnableIRQ(AUX_RECEIVER_TIM_IRQn);
	} else if (htim->Instance == TASK_STATUS_TIM){ //for task status

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/*TIM6 Peripheral clock enable */
		TASK_STATUS_TIM_CLK_ENABLE();

		/*##-2- Configure the NVIC for AUX_RECEIVER_TIM ############################*/
		HAL_NVIC_SetPriority(TASK_STATUS_TIM_IRQn, AUX_RECEIVER_TIM_IRQ_PREEMPT_PRIO, AUX_RECEIVER_TIM_IRQ_SUB_PRIO);

		/* Enable the AUX_RECEIVER_TIM global Interrupt */
		HAL_NVIC_EnableIRQ(TASK_STATUS_TIM_IRQn);

	}
}

/**
 * @brief TIM MSP Deinitialization
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		/* Primary receiver TIM Peripheral clock disable */
		PRIMARY_RECEIVER_TIM_CLK_DISABLE();
	} else if (htim->Instance == AUX_RECEIVER_TIM) {
		/* Aux receiver TIM Peripheral clock disable */
		AUX_RECEIVER_TIM_CLK_DISABLE();
	}
}

/**
 * @brief TIM PWM MSP Initialization
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM_MOTOR) {
		GPIO_InitTypeDef GPIO_InitStruct;
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Motor TIM Peripheral clock enable */
		MOTOR_TIM_CLK_ENABLE();

		/* Enable Motor GPIO Channels Clock */
		MOTOR_TIM_CHANNEL_GPIO_PORT();

		/* Configure Motor GPIO Pins */
		/* Common configuration for all channels */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = MOTOR_TIM_AF;

		/* Motor channel 1 pin */
		GPIO_InitStruct.Pin = MOTOR_GPIO_PIN_CHANNEL1;
		HAL_GPIO_Init(MOTOR_PIN_PORT, &GPIO_InitStruct);

		/* Motor channel 2 pin */
		GPIO_InitStruct.Pin = MOTOR_GPIO_PIN_CHANNEL2;
		HAL_GPIO_Init(MOTOR_PIN_PORT, &GPIO_InitStruct);

		/* Motor channel 3 pin */
		GPIO_InitStruct.Pin = MOTOR_GPIO_PIN_CHANNEL3;
		HAL_GPIO_Init(MOTOR_PIN_PORT, &GPIO_InitStruct);

		/* Motor channel 4 pin */
		GPIO_InitStruct.Pin = MOTOR_GPIO_PIN_CHANNEL4;
		HAL_GPIO_Init(MOTOR_PIN_PORT, &GPIO_InitStruct);
	}
}

/**
 * @brief TIM PWM MSP Initialization
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM_MOTOR) {
		/* Motor TIM Peripheral clock disable */
		MOTOR_TIM_CLK_DISABLE();
	}
}

/**
 * @brief TIM IC MSP Initialization
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		GPIO_InitTypeDef GPIO_InitStruct;

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Primary Receiver TIM Peripheral clock enable */
		PRIMARY_RECEIVER_TIM_CLK_ENABLE();

		/* Enable GPIO channels Clock */
		PRIMARY_RECEIVER_TIM_CHANNEL_GPIO_PORT();

		/* Configure  channels for Alternate function, push-pull and 100MHz speed */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = PRIMARY_RECEIVER_TIM_AF;

		/* Primary Receiver Channel 1 pin */
		GPIO_InitStruct.Pin = PRIMARY_RECEIVER_PIN_CHANNEL1;
		HAL_GPIO_Init(PRIMARY_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/* Primary Receiver Channel 2 pin */
		GPIO_InitStruct.Pin = PRIMARY_RECEIVER_PIN_CHANNEL2;
		HAL_GPIO_Init(PRIMARY_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/* Primary Receiver Channel 3 pin */
		GPIO_InitStruct.Pin = PRIMARY_RECEIVER_PIN_CHANNEL3;
		HAL_GPIO_Init(PRIMARY_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/* Primary Receiver Channel 4 pin */
		GPIO_InitStruct.Pin = PRIMARY_RECEIVER_PIN_CHANNEL4;
		HAL_GPIO_Init(PRIMARY_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/*##-2- Configure the NVIC for PRIMARY_RECEIVER_TIM ########################*/
		HAL_NVIC_SetPriority(PRIMARY_RECEIVER_TIM_IRQn, PRIMARY_RECEIVER_TIM_IRQ_PREEMPT_PRIO,
				PRIMARY_RECEIVER_TIM_IRQ_SUB_PRIO);

		/* Enable the PRIMARY_RECEIVER_TIM global Interrupt */
		HAL_NVIC_EnableIRQ(PRIMARY_RECEIVER_TIM_IRQn);
	} else if (htim->Instance == AUX_RECEIVER_TIM) {
		GPIO_InitTypeDef GPIO_InitStruct;

		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Aux Receiver TIM Peripheral clock enable */
		AUX_RECEIVER_TIM_CLK_ENABLE();

		/* Enable GPIO channels Clock */
		AUX_RECEIVER_TIM_CHANNEL_GPIO_PORT();

		/* Configure channels for Alternate function, push-pull and 100MHz speed */
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = AUX_RECEIVER_TIM_AF;

		/* Aux Receiver Channel 1 pin */
		GPIO_InitStruct.Pin = AUX_RECEIVER_PIN_CHANNEL1;
		HAL_GPIO_Init(AUX_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/* Aux Receiver Channel 2 pin */
		GPIO_InitStruct.Pin = AUX_RECEIVER_PIN_CHANNEL2;
		HAL_GPIO_Init(AUX_RECEIVER_TIM_PIN_PORT, &GPIO_InitStruct);

		/*##-2- Configure the NVIC for AUX_RECEIVER_TIM ############################*/
		HAL_NVIC_SetPriority(AUX_RECEIVER_TIM_IRQn, AUX_RECEIVER_TIM_IRQ_PREEMPT_PRIO, AUX_RECEIVER_TIM_IRQ_SUB_PRIO);

		/* Enable the AUX_RECEIVER_TIM global Interrupt */
		HAL_NVIC_EnableIRQ(AUX_RECEIVER_TIM_IRQn);
	}
}

/**
 * @brief TIM IC MSP Deinitialization
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		/* Primary receiver TIM Peripheral clock disable */
		PRIMARY_RECEIVER_TIM_CLK_DISABLE();
	} else if (htim->Instance == AUX_RECEIVER_TIM) {
		/* Aux receiver TIM Peripheral clock disable */
		AUX_RECEIVER_TIM_CLK_DISABLE();
	}
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/*****END OF FILE****/
