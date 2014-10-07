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
#include <math.h>

/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;	// TIM3 init struct
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};	// Body-frame sensor readings arrays
float Attitude[3] = {0.0, 0.0, 0.0};	// Roll, pitch, yaw angles
float h = 0.0; // Control sample time

PWM_TimeTypeDef pwmTimes;	// 6-channel PWM input width in seconds
/*	pwmTimes.PWM_Time1; // Throttle
	pwmTimes.PWM_Time2; // Aileron
	pwmTimes.PWM_Time3; // Elevator
	pwmTimes.PWM_Time4; // Rudder
	pwmTimes.PWM_Time5; // Function 1
	pwmTimes.PWM_Time6; // Function 2
*/

// Physical control signals
float U[4] = {0.0, 0.0, 0.0, 0.0}; // Thrust force, roll, pitch, yaw moments.
// Motor output PWM widths [s]
float t_out[4] = {0.0, 0.0, 0.0, 0.0};
float out_temp[4] = {0.0, 0.0, 0.0, 0.0};

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

		/* Read Gyro Angular rate data and update attitude */
		GyroReadAngRate(GyroBuffer);
		UpdateAttitude();
		/* Read Compass data */
		CompassReadMag(MagBuffer);
		CompassReadAcc(AccBuffer);

		GetPWMInputTimes(&pwmTimes);

		SetControlSignals();

		ControlAllocation();

		// Set motor output PWM
		TIM4->CCR1 = GetPWM_CCR(t_out[0]);	// To motor 1 (PD12)
		TIM4->CCR2 = GetPWM_CCR(t_out[1]);	// To motor 2 (PD13)
		TIM4->CCR3 = GetPWM_CCR(t_out[2]);	// To motor 3 (PD14)
		TIM4->CCR4 = GetPWM_CCR(t_out[3]);	// To motor 4 (PD15)
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
	TIM_TimeBaseStructure3.TIM_Prescaler = SystemCoreClock/TIM3_FREQ - 1;	// Scaling of system clock freq
	TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure3.TIM_Period = TIM3_FREQ/TIM3_CTRLFREQ - 1;		// Counter reset value
	TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

	/* Set control sample time */
	h = 1 / (SystemCoreClock/(TIM_GetPrescaler(TIM3)+1)/(TIM3->ARR+1));

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
	return (uint16_t) ((float)(t * SystemCoreClock/((float)(TIM_GetPrescaler(TIM4)+1))));
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

void UpdateAttitude(void)
{
	// Needs transformation of Gyro rate Body->Earth frame for accuracy
	Attitude[0] = Attitude[0] + GyroBuffer[0]*1/TIM3_CTRLFREQ;
	Attitude[1] = Attitude[1] + GyroBuffer[1]*1/TIM3_CTRLFREQ;
	Attitude[2] = Attitude[2] + GyroBuffer[2]*1/TIM3_CTRLFREQ;
}

/* SetControlSignals
 * @brief  Calculates the desired forces and moments to actuate the system with.
 * @param  None
 * @retval None
 */
void SetControlSignals(void)
{
	// Set thrust output (max thrust approx 48 N)
	if(pwmTimes.PWM_Time1 >= 0.001)
		U[0] = 48000*(pwmTimes.PWM_Time1-0.001);
	else
		U[0] = 0.0;

	// Set roll output (max roll moment approx +/- 5.1 Nm)
	if (pwmTimes.PWM_Time2 >= 0.001 && pwmTimes.PWM_Time2 < 0.002)
		U[1] = -10200*(pwmTimes.PWM_Time2-0.0015);
	else
		U[1] = 0;

	// Set pitch output (max pitch moment approx +/- 5.1 Nm)
	if (pwmTimes.PWM_Time3 >= 0.001 && pwmTimes.PWM_Time3 < 0.002)
		U[2] = 10200*(pwmTimes.PWM_Time3-0.0015);
	else
		U[2] = 0;

	// Set yaw output (max yaw moment approx +/- 0.7 Nm)
	if (pwmTimes.PWM_Time4 >= 0.001 && pwmTimes.PWM_Time4 < 0.002)
		U[3] = -1400*(pwmTimes.PWM_Time4-0.0015);
	else
		U[3] = 0;
}

/* ControlAllocation
 * @brief  Allocates the desired thrust force and moments to corresponding motor action.
 * 		   Data has been fitted to map thrust force [N] and roll/pitch/yaw moments [Nm] to
 * 		   motor output PWM widths [s] of each of the four motors.
 * @param  None
 * @retval None
 */
void ControlAllocation(void)
{
	out_temp[0] = (Bq*L*U[0] - 4*Bq*Ct*L - M_SQRT2*Bq*U[1] - M_SQRT2*Bq*U[2] - At*L*U[3]) / ((double)4*At*Bq*L);
	out_temp[1] = (Bq*L*U[0] - 4*Bq*Ct*L - M_SQRT2*Bq*U[1] + M_SQRT2*Bq*U[2] + At*L*U[3]) / ((double)4*At*Bq*L);
	out_temp[2] = (Bq*L*U[0] - 4*Bq*Ct*L + M_SQRT2*Bq*U[1] + M_SQRT2*Bq*U[2] - At*L*U[3]) / ((double)4*At*Bq*L);
	out_temp[3] = (Bq*L*U[0] - 4*Bq*Ct*L + M_SQRT2*Bq*U[1] - M_SQRT2*Bq*U[2] + At*L*U[3]) / ((double)4*At*Bq*L);

	if(out_temp[0] >= 0)
		t_out[0] = sqrtf(out_temp[0]);
	else
		t_out[0] = 0.001;

	if(out_temp[1] >= 0)
		t_out[1] = sqrtf(out_temp[1]);
	else
		t_out[1] = 0.001;

	if(out_temp[2] >= 0)
		t_out[2] = sqrtf(out_temp[2]);
	else
		t_out[2] = 0.001;

	if(out_temp[3] >= 0)
		t_out[3] = sqrtf(out_temp[3]);
	else
		t_out[3] = 0.001;

// Set ESC limits

	if(t_out[0] < 0.001)
		t_out[0] = 0.001;
	else if(t_out[0] > 0.002)
		t_out[0] = 0.002;

	if(t_out[1] < 0.001)
		t_out[1] = 0.001;
	else if(t_out[1] > 0.002)
		t_out[1] = 0.002;

	if(t_out[2] < 0.001)
		t_out[2] = 0.001;
	else if(t_out[2] > 0.002)
		t_out[2] = 0.002;

	if(t_out[3] < 0.001)
		t_out[3] = 0.001;
	else if(t_out[3] > 0.002)
		t_out[3] = 0.002;

}
