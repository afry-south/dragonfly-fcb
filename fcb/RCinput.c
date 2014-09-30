/**
******************************************************************************
* @file    fcb/RCinput.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          File contains functionality for reading signals from the RC receiver
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_it.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"
#include "RCinput.h"


#define PWM_INPUT_SAMPLE_CLOCK 2400000

/* PWM input variables */
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;		// TIM2

TIM_ICInitTypeDef TIM_CH1_ICInitStructure;
TIM_ICInitTypeDef TIM_CH2_ICInitStructure;
TIM_ICInitTypeDef TIM_CH3_ICInitStructure;
TIM_ICInitTypeDef TIM_CH4_ICInitStructure;
volatile uint16_t PWM_IN_DOWN[4] = {0, 0, 0, 0}; 	// Current falling edge (for PWM input 1, 2, 3, 4)
volatile uint16_t PWM_IN_UP[4] = {0, 0, 0, 0};		// Current rising edge (for PWM input 1, 2, 3, 4)
volatile uint16_t PWM_IN_UP2[4] = {0, 0, 0, 0};		// Previous rising edge (for PWM input 1, 2, 3, 4)
volatile char pulseState[4] = {0, 0, 0, 0};			// 0 = rising, 1 = falling detection (for PWM input 1, 2, 3, 4)
//TODO implement getter for Duty and Period
volatile uint16_t PWM_IN_DutyCycleTicks[4] = {0, 0, 0, 0};	// Number of PWM duty cycle ticks (for PWM input 1, 2, 3, 4)
volatile uint16_t PWM_IN_PeriodTicks[4] = {0, 0, 0, 0};		// Number of PWM period ticks (for PWM input 1, 2, 3, 4)

PWM_TimeTypeDef PWMTimes;

//PWM_TimeTypeDef PWMTimes[2];
//volatile uint PWMSync = 0;

/**
 * TODO blabla
 */
/*
static void SetPWMInputTimes(float PWM_Time1, float PWM_Time2, float PWM_Time3, float PWM_Time4, float PWM_Time5, float PWM_Time6)
{
	if(PWMSync)
	{
		PWMTimes[0]->PWM_Time1 = PWM_Time1;
		PWMTimes[0]->PWM_Time2 = PWM_Time2;
		PWMTimes[0]->PWM_Time3 = PWM_Time3;
		PWMTimes[0]->PWM_Time4 = PWM_Time4;
		PWMTimes[0]->PWM_Time5 = PWM_Time5;
		PWMTimes[0]->PWM_Time6 = PWM_Time6;

		PWMSync = 0;
	}
	else
	{
		PWMTimes[1]->PWM_Time1 = PWM_Time1;
		PWMTimes[1]->PWM_Time2 = PWM_Time2;
		PWMTimes[1]->PWM_Time3 = PWM_Time3;
		PWMTimes[1]->PWM_Time4 = PWM_Time4;
		PWMTimes[1]->PWM_Time5 = PWM_Time5;
		PWMTimes[1]->PWM_Time6 = PWM_Time6;

		PWMSync = 1;
	}
}
*/

/**
 * TODO Blabla
 */
void GetPWMInputTimes(PWM_TimeTypeDef *PWM_Time)
{
		memcpy((void*)PWM_Time ,(const void*)(&PWMTimes), sizeof(PWM_TimeTypeDef));
}



/* @TIM2_IRQHandler
 * @brief	Timer 2 interrupt handler.
 * @param	None.
 * @retval	None.
 */
void TIM2_IRQHandler()
{
	/* If interrupt concerns CH1 */
	if (TIM2->SR & TIM_IT_CC1)
	{
		TIM2->SR &= (~TIM_IT_CC1);

		if (pulseState[0] == 0)	// Rising edge
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_IN_UP2[0] = PWM_IN_UP[0];
			PWM_IN_UP[0] = TIM_GetCapture1(TIM2);

			if(PWM_IN_UP[0] >= PWM_IN_UP2[0])
				PWM_IN_PeriodTicks[0] = PWM_IN_UP[0] - PWM_IN_UP2[0];
			else
				PWM_IN_PeriodTicks[0] = PWM_IN_UP[0] + 0xFFFF - PWM_IN_UP2[0];

				//printf("Channel 1, Rising edge, Timer ticks = %d \nPeriod ticks = %d", PWM_1_IN_UP, PWM_1_IN_PeriodTicks);
				GPIO_SetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 high
		}
		else	//Falling edge
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[0] = TIM_GetCapture1(TIM2);

			if (PWM_IN_DOWN[0] >= PWM_IN_UP[0])
				PWM_IN_DutyCycleTicks[0] = PWM_IN_DOWN[0] - PWM_IN_UP[0];
			else
				PWM_IN_DutyCycleTicks[0] = PWM_IN_DOWN[0] + 0xFFFF - PWM_IN_UP[0];

			//TODO sanity check
			PWMTimes.PWM_Time1 = (float)((float)PWM_IN_DutyCycleTicks[0])/((float)PWM_INPUT_SAMPLE_CLOCK);

			//printf("Channel 1, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_1_IN_DOWN, PWM_1_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 low
		}
		pulseState[0] = !pulseState[0];
		TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);	// Reverse polarity
	}


	/* If interrupt concerns CH2 */
	if (TIM2->SR & TIM_IT_CC2)
	{
		TIM2->SR &= (~TIM_IT_CC2);

		if (pulseState[1] == 0)	// Rising edge
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[1] = PWM_IN_UP[1];
			PWM_IN_UP[1] = TIM_GetCapture2(TIM2);

			if(PWM_IN_UP[1] >= PWM_IN_UP2[1])
				PWM_IN_PeriodTicks[1] = PWM_IN_UP[1] - PWM_IN_UP2[1];
			else
				PWM_IN_PeriodTicks[1] = PWM_IN_UP[1] + 0xFFFF - PWM_IN_UP2[1];

			//printf("Channel 2, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_2_IN_UP, PWM_2_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 high
		}
		else	//Falling edge
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[1] = TIM_GetCapture2(TIM2);

			if (PWM_IN_DOWN[1] >= PWM_IN_UP[1])
				PWM_IN_DutyCycleTicks[1] = PWM_IN_DOWN[1] - PWM_IN_UP[1];
			else
				PWM_IN_DutyCycleTicks[1] = PWM_IN_DOWN[1] + 0xFFFF - PWM_IN_UP[1];

			PWMTimes.PWM_Time2 = (float)((float)PWM_IN_DutyCycleTicks[1])/((float)PWM_INPUT_SAMPLE_CLOCK);
			//printf("Channel 2, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_2_IN_DOWN, PWM_2_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 low
		}
		pulseState[1] = !pulseState[1];
		TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);	// Reverse polarity
	}


	/* If interrupt concerns CH3 */
	if (TIM2->SR & TIM_IT_CC3)
	{
		TIM2->SR &= (~TIM_IT_CC3);

		if (pulseState[2] == 0)	// Rising edge
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[2] = PWM_IN_UP[2];
			PWM_IN_UP[2] = TIM_GetCapture3(TIM2);

			if(PWM_IN_UP[2] >= PWM_IN_UP2[2])
				PWM_IN_PeriodTicks[2] = PWM_IN_UP[2] - PWM_IN_UP2[2];
			else
				PWM_IN_PeriodTicks[2] = PWM_IN_UP[2] + 0xFFFF - PWM_IN_UP2[2];

			//printf("Channel 3, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_3_IN_UP, PWM_3_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 high

		}
		else	//Falling edge
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[2] = TIM_GetCapture3(TIM2);

			if (PWM_IN_DOWN[2] >= PWM_IN_UP[2])
				PWM_IN_DutyCycleTicks[2] = PWM_IN_DOWN[2] - PWM_IN_UP[2];
			else
				PWM_IN_DutyCycleTicks[2] = PWM_IN_DOWN[2] + 0xFFFF - PWM_IN_UP[2];

			PWMTimes.PWM_Time3 = (float)((float)PWM_IN_DutyCycleTicks[2])/((float)PWM_INPUT_SAMPLE_CLOCK);
			//printf("Channel 3, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_3_IN_DOWN, PWM_3_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 low
		}
		pulseState[2] = !pulseState[2];
		TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure); // Reverse polarity
	}

	/* If interrupt concerns CH4 */
	if (TIM2->SR & TIM_IT_CC4)
	{
		TIM2->SR &= (~TIM_IT_CC4);

		if (pulseState[3] == 0)	// Rising edge
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[3] = PWM_IN_UP[3];
			PWM_IN_UP[3] = TIM_GetCapture4(TIM2);

			if(PWM_IN_UP[3] >= PWM_IN_UP2[3])
				PWM_IN_PeriodTicks[3] = PWM_IN_UP[3] - PWM_IN_UP2[3];
			else
				PWM_IN_PeriodTicks[3] = PWM_IN_UP[3] + 0xFFFF - PWM_IN_UP2[3];

			//printf("Channel 4, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_4_IN_UP, PWM_4_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 high
		}
		else	//Falling edge
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[3] = TIM_GetCapture4(TIM2);

			if (PWM_IN_DOWN[3] >= PWM_IN_UP[3])
				PWM_IN_DutyCycleTicks[3] = PWM_IN_DOWN[3] - PWM_IN_UP[3];
			else
				PWM_IN_DutyCycleTicks[3] = PWM_IN_DOWN[3] + 0xFFFF - PWM_IN_UP[3];

			PWMTimes.PWM_Time4 = (float)((float)PWM_IN_DutyCycleTicks[3])/((float)PWM_INPUT_SAMPLE_CLOCK);
			//printf("Channel 4, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_4_IN_DOWN, PWM_4_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 low
		}
		pulseState[3] = !pulseState[3];
		TIM_ICInit(TIM2, &TIM_CH4_ICInitStructure);	// Reverse polarity
	}
}

#define PWM_INPUT_SAMPLE_CLOCK 2400000

/* @TIM2_Setup
 * @brief	Timer 2 setup
 * @param	None.
 * @retval	None.
 */
void TIM2_Setup(void) {
	//TIM2 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	// TIM2 Time Base configuration
	TIM_TimeBaseStructure2.TIM_Prescaler = SystemCoreClock/PWM_INPUT_SAMPLE_CLOCK -1;	// 72 MHz to 2.4 MHz
	TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure2.TIM_Period = 0xFFFF;	// Can take receiver input down to less than 40 Hz (50 Hz is common?)
	TIM_TimeBaseStructure2.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure2.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);

	// TIM2 counter enable
	TIM_Cmd(TIM2, ENABLE);
}

/* @PWM_In_Setup
 * @brief  Sets up timer 2, GPIO and NVIC for PWM input
 * @param  None
 * @retval None
 */
void PWM_In_Setup(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* GPIOD clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* TIM2 GPIO pin configuration : CH1=PD3, C2=PD4, CH3=PD7, CH4=PD6 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect pins to TIM2 AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_2);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Timers 1-4 IC init structs, not used until interrupt handler
	TIM_CH1_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH1_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);

	TIM_CH2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH2_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);

	TIM_CH3_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH3_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure);

	TIM_CH4_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH4_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH4_ICInitStructure);

	//Enable TIM2
	TIM_Cmd(TIM2, ENABLE);

	//Enable CC1-4 interrupt
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	//Clear CC1 Flag
	TIM_ClearFlag(TIM2, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 );
}
