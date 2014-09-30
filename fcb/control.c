/**
******************************************************************************
* @file    fcb/control.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          File contains flight controller logic.
******************************************************************************
**/
/* @getPWM_CCR
 * @brief	Returns the CCR (Capture compare register) value for a specified PWM duty cycle.
 * @param	dutycycle: The dutycycle percentage in decimal form (i.e. 45.2% = 0.452)
 * @retval	CCR value (Nbr of PWM clock ticks with voltage) corresponding to the desired duty cycle.
 */
#include "control.h"
#include "RCinput.h"
#include "sensors.h"
#include "motor_output.h"

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3; // TIM3

/* Sensor readings variables  */
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};


/* @getPWM_CCR
 * @brief	TODO Blabla.
 * @param	None.
 * @retval	None.
 */
uint16_t getPWM_CCR(float dutycycle, uint16_t period)
{
	return (uint16_t) (dutycycle * period);
}

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
//		GyroReadAngRate(GyroBuffer); // BLOCKING
		/* Read Compass data */
//		CompassReadMag(MagBuffer); // BLOCKING
//		CompassReadAcc(AccBuffer);

		// Set motor output PWM
		TIM4->CCR1 = getPWM_CCR((float)(PWM_IN_DutyCycleTicks[0]/PWM_IN_PeriodTicks[0]), TIM4_Period);
		TIM4->CCR2 = getPWM_CCR((float)(PWM_IN_DutyCycleTicks[1]/PWM_IN_PeriodTicks[1]), TIM4_Period);
		TIM4->CCR3 = getPWM_CCR((float)(PWM_IN_DutyCycleTicks[2]/PWM_IN_PeriodTicks[2]), TIM4_Period);
		TIM4->CCR4 = getPWM_CCR((float)(PWM_IN_DutyCycleTicks[3]/PWM_IN_PeriodTicks[3]), TIM4_Period);
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
	TIM_TimeBaseStructure3.TIM_Period = 10000 - 1;						// 1 Mhz to 100 Hz
	TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);
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
