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

/* Timer and sampling */
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;		// TIM3 init struct
volatile float h = 1/((float)TIM3_CTRLFREQ);		// Control sample time

/* Measured states */
float BodyAttitude[3] = {0.0f};	// Body-frame Roll, pitch, yaw angles (you can't really talk about angles in the body frame,
								// but these may be approximated to angles from the starting position. TODO: DCM needed.
float BodyVelocity[3] = {0.0f};	// Body-frame velocities
float YawRate = 0.0;

/* RC input */
PWM_TimeTypeDef PWMInputTimes;	// 6-channel PWM input width in seconds
/*	PWMInputTimes.PWM_Time1; // Throttle / Thrust
	PWMInputTimes.PWM_Time2; // Aileron  / Roll
	PWMInputTimes.PWM_Time3; // Elevator / Pitch
	PWMInputTimes.PWM_Time4; // Rudder   / Yaw
	PWMInputTimes.PWM_Time5; // Function 1
	PWMInputTimes.PWM_Time6; // Function 2
*/

/* Control variables */
float U[4] = {0.0, 0.0, 0.0, 0.0};	// Physical control signals
/*	U[0] Thrust force, max 48 N
 *  U[1] Roll moment, max +/- 5.1 Nm
 *  U[2] Pitch moment, max +/- 5.1 Nm
 *  U[3] Yaw moment, max +/- 0.7 Nm (very uncertain)
 */
float u[4] = {0.0, 0.0, 0.0, 0.0};	// Controller output signals
float refs[4] = {0.0, 0.0, 0.0, 0.0};	// Controller reference signals
/*	refs[0] Vertical velocity ref [m/s]
 *  refs[1] Roll angle ref [rad]
 *  refs[2] Pitch angle ref [rad]
 *  refs[3] Yaw angular rate ref [rad/s]
 */

float Ivz = 0.0, Dvz = 0.0, Ir = 0.0, Dr = 0.0, Ip = 0.0, Dp = 0.0, Iy = 0.0, Dy = 0.0;
float bodyZvelocityPrev = 0.0, bodyRollPrev = 0.0, bodyPitchPrev = 0.0, bodyYawRatePrev = 0.0;

/* Motor output */
float t_out[4] = {0.0, 0.0, 0.0, 0.0};		// Motor output PWM widths [s]
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

		GetPWMInputTimes(&PWMInputTimes);
		SetReferenceSignals();
		ReadSensors();

		if(!GetAccCalibrated() || !GetGyroCalibrated())
		{
			CalibrateAcc();
			CalibrateGyro();
		}
		else
		{
			/* Read Magnetometer data */
			//CompassReadMag(MagBuffer);
			//Heading = GetHeading();	// Heading in rad (0 rad is north)

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();

			AltitudeControl();
			RollControl();
			PitchControl();
			YawControl();

			ControlAllocation();

			// Set motor output PWM
			TIM4->CCR1 = GetPWM_CCR(t_out[0]);	// To motor 1 (PD12)
			TIM4->CCR2 = GetPWM_CCR(t_out[1]);	// To motor 2 (PD13)
			TIM4->CCR3 = GetPWM_CCR(t_out[2]);	// To motor 3 (PD14)
			TIM4->CCR4 = GetPWM_CCR(t_out[3]);	// To motor 4 (PD15)
			// TODO PWMInputTimes.PWM_Time5
			// TODO PWMInputTimes.PWM_Time6
		}

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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* TIM3 Time Base configuration */
	TIM_TimeBaseStructure3.TIM_Prescaler = SystemCoreClock/TIM3_FREQ - 1;	// Scaling of system clock freq
	TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure3.TIM_Period = TIM3_FREQ/TIM3_CTRLFREQ - 1;		// Counter reset value
	TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);
}

/* @AltitudeControl
 * @brief	Controls the thrust force to achieve a desired vertical velocity
 * @param	None.
 * @retval	None.
 */
void AltitudeControl(void)
{
	float bodyZvelocity = BodyVelocity[2]*cosf(BodyAttitude[0])*cosf(BodyAttitude[1]);

	// Need inertial (vertical body velocity)
	float Pvz = Kvz*(BETAvz*refs[0] - bodyZvelocity);

	// Backward difference, derivative part with zero set-point weighting
	Dvz = TDvz/(TDvz+Nvz*h)*Dvz - Kvz*TDvz*Nvz/(TDvz+Nvz*h)*(bodyZvelocity - bodyZvelocityPrev);

	// TODO: Saturation levels
	U[0] = (Pvz + Ivz + Dvz + g) * M; // /cosf(BodyAttitude[0])*cosf(BodyAttitude[1]);

	// Forward difference, so updated after control
	if(TIvz != 0.0)
		Ivz = Ivz + Kvz*h/TIvz * (refs[0] - bodyZvelocity);
	bodyZvelocityPrev = bodyZvelocity;
}

/* @RollControl
 * @brief	Controls the roll moment to achieve a desired roll angle
 * @param	None.
 * @retval	None.
 */
void RollControl(void)
{
	float Pr = Krp*(BETArp*refs[1] - BodyAttitude[0]);

	// Backward difference, derivative part with zero set-point weighting
	Dr = TDrp/(TDrp+Nrp*h)*Dr - Krp*TDrp*Nrp/(TDrp+Nrp*h)*(BodyAttitude[0] - bodyRollPrev);

	// TODO: Saturation levels
	U[1] = (Pr + Ir + Dr) * IXX;

	// Forward difference, so updated after control
	if(TIrp != 0.0)
		Ir = Ir + Krp*h/TIrp * (refs[1] - BodyAttitude[0]);
	bodyRollPrev = BodyAttitude[0];
}

/* @PitchControl
 * @brief	Controls the pitch moment to achieve a desired pitch angle
 * @param	None.
 * @retval	None.
 */
void PitchControl(void)
{
	float Pp = Krp*(BETArp*refs[2] - BodyAttitude[1]);

	// Backward difference, derivative part with zero set-point weighting
	Dp = TDrp/(TDrp+Nrp*h)*Dp - Krp*TDrp*Nrp/(TDrp+Nrp*h)*(BodyAttitude[1] - bodyPitchPrev);

	// TODO: Saturation levels
	U[2] = (Pp + Ip + Dp) * IYY;

	// Forward difference, so updated after control
	if(TIrp != 0.0)
		Ip = Ip + Krp*h/TIrp * (refs[2] - BodyAttitude[1]);
	bodyPitchPrev = BodyAttitude[1];
}

/* @YawControl
 * @brief	Controls the yaw moment to achieve a desired yaw rate
 * @param	None.
 * @retval	None.
 */
void YawControl(void)
{
	float Py = Kyr*(BETAyr*refs[3] - YawRate);

	// Backward difference
	Dy = TDyr/(TDyr+Nyr*h)*Dy - Kyr*TDyr*Nyr/(TDyr+Nyr*h)*(YawRate - bodyYawRatePrev);

	// TODO: Saturation levels
	U[3] = (Py + Iy + Dy) * IZZ;

	// Forward difference, so updated after control
	if(TIyr != 0.0)
		Iy = Iy + Kyr*h/TIyr * (refs[3] - YawRate);
	bodyYawRatePrev = YawRate;
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

/* SetControlSignals
 * @brief  Sets the reference values based on RC controller input
 * @param  None
 * @retval None
 */
void SetReferenceSignals(void)
{
	// Set velocity reference limits
	if(PWMInputTimes.PWM_Time1 >= 0.001 && PWMInputTimes.PWM_Time1 < 0.002)
		refs[0] = 2*MAX_Z_VELOCITY*1000*(PWMInputTimes.PWM_Time1-0.0015);
	else
		refs[0] = 0.0;

	// Set roll reference limits
	if (PWMInputTimes.PWM_Time2 >= 0.001 && PWMInputTimes.PWM_Time2 < 0.002)
		refs[1] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.PWM_Time2-0.0015);
	else
		refs[1] = 0;

	// Set pitch reference limits
	if (PWMInputTimes.PWM_Time3 >= 0.001 && PWMInputTimes.PWM_Time3 < 0.002)
		refs[2] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.PWM_Time3-0.0015);
	else
		refs[2] = 0;

	// Set yaw rate reference limits
	if (PWMInputTimes.PWM_Time4 >= 0.001 && PWMInputTimes.PWM_Time4 < 0.002)
		refs[3] = 2*MAX_YAW_RATE*1000*(PWMInputTimes.PWM_Time4-0.0015);
	else
		refs[3] = 0;
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

/* @getPWM_CCR
 * @brief	Recalculates a time pulse width to number of TIM4 clock ticks.
 * @param	t is the pulse width in seconds.
 * @retval	TIM4 clock ticks to be written to TIM4 CCR output.
 */
uint16_t GetPWM_CCR(float t)
{
	return (uint16_t) ((float)(t * SystemCoreClock/((float)(TIM_GetPrescaler(TIM4)+1))));
}
