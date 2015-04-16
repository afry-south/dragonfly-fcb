/*****************************************************************************
* @file    fcb/motor_output.c
* @author  ÅF Dragonfly
* Daniel Stenberg, Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          File contains pwm output
******************************************************************************/

/* @TIM4_IOconfig
 * @brief	Initializes and configures output pins used to
 * 			physically transmit PWM motor control signals.
 * @param	None.
 * @retval	None.
 */
#include "motor_output.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

/* PWM output variables */
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure4; // TIM4

//TODO comment
TIM_OCInitTypeDef TIM_OCInitStructure;

void TIM4_IOconfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIO D clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_2);
}

/* @TIM4_Setup
 * @brief	Sets up the Timer 4 timebase. Timer 4 is responsible
 * 			for producing the PWM motor control signals at 400 Hz.
 * @param	None.
 * @retval	None.
 */
void TIM4_Setup(void)
{
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);

	/* TIM4 Time Base configuration */
	TIM_TimeBaseStructure4.TIM_Prescaler = SystemCoreClock/MOTOR_OUT_SAMPLECLOCK-1; // SystemCoreClock (72 MHz on STM32F303) to 24 MHz (Prescaler = 3)
	TIM_TimeBaseStructure4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure4.TIM_Period = TIM4_Period-1;
	TIM_TimeBaseStructure4.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure4.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure4);

	/* TIM4 counter enable */
	TIM_Cmd(TIM4, ENABLE);
}

/* @TIM4_SetupOC
 * @brief  Configures the Timer 4 OC registers for PWM output
 * 		   Channel 1 outputs on pin PD12
 * 		   Channel 2 outputs on pin PD13
 * 		   Channel 3 outputs on pin PD14
 * 		   Channel 4 outputs on pin PD15
 * @param  None
 * @retval None
 */
void TIM4_SetupOC(void)
{
	/* Channel 1, 2,3, 4 configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	// Initializes Timer 4 OC registers (Channels 1, 2, 3, 4)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	/* TIM4 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}
