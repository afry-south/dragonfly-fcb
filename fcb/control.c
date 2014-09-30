/**
******************************************************************************
* @file    fcb/control.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          File contains flight controller logic (input->control->output).
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "RCinput.h"
#include "sensors.h"
#include "motor_output.h"
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;	// TIM3 init struct
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};	// Sensor readings arrays
PWM_TimeTypeDef pwmTimes;	// 6-channel PWM input width in seconds

/* @TIM3_IRQHandler
 * @brief	Timer 3 interrupt handler.
 * 			Continually performs program duties with regular intervals.
 * @param	None.
 * @retval	None.
 */
void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		/* Read Gyro Angular rate data */

//		GyroReadAngRate(GyroBuffer);	// BLOCKING...
		/* Read Compass data */
//		CompassReadMag(MagBuffer);		// BLOCKING...
//		CompassReadAcc(AccBuffer);		// BLOCKING...

		// TODO Everything counted in seconds, make microsends (10^(-6) s) to avoid using floats?
		GetPWMInputTimes(&pwmTimes);

		// Set motor output PWM
		TIM4->CCR1 = GetPWM_CCR(pwmTimes.PWM_Time1);
		TIM4->CCR2 = GetPWM_CCR(pwmTimes.PWM_Time2);
		TIM4->CCR3 = GetPWM_CCR(pwmTimes.PWM_Time3);
		TIM4->CCR4 = GetPWM_CCR(pwmTimes.PWM_Time4);
		// TODO pwmTimes.PWM_Time5
		// TODO pwmTimes.PWM_Time6
	}
}

/* @TIM3_Setup
 * @brief	Sets up the Timer 3 timebase. Timer 3 is responsible for
 * 			generating system interrupts at well-defined intervals used
 * 			to execute code at discrete time intervals.
 * @param	None.
 * @retval	None.
 */
void TIM3_Setup(void)
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	/* TIM3 Time Base configuration */
	TIM_TimeBaseStructure3.TIM_Prescaler = SystemCoreClock/1000000 - 1;	// 72 MHz to 1 MHz
	TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure3.TIM_Period = 20000 - 1;						// 1 Mhz to 50 Hz
	TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);
}

/* @getPWM_CCR
 * @brief	Recalculates a time pulse width to number of TIM4 clock ticks.
 * @param	t is the pulse width in seconds.
 * @retval	TIM4 clock ticks to be written to TIM4 CCR output.
 */
uint16_t GetPWM_CCR(float t)
{
	return (uint16_t) (t * SystemCoreClock/TIM_GetPrescaler(TIM4));
}

/* TIM3_SetupIRQ
 * @brief  Configures the Timer 3 IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM3_SetupIRQ(void)
{
	/* Interrupt config */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}
