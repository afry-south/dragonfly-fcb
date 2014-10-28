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
__I float h = 1/((float)TIM3_CTRLFREQ);	// Control sample time [s]

/* Measured states */
float BodyAttitude[3] = {0.0f};	// Body-frame roll, pitch, yaw angles [rad]
float BodyVelocity[3] = {0.0f};	// Body-frame velocities [m/s]
float YawRate = 0.0;
float Heading = 0.0;

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

/* RC input limits - subject to calibration */
float RCmin = 0.0010, RCmid = 0.0015, RCmax = 0.0020;

/* Controller reference signals */
float refs[4] = {0.0, 0.0, 0.0, 0.0};
/*	refs[0] Vertical velocity ref [m/s]
 *  refs[1] Roll angle ref [rad]
 *  refs[2] Pitch angle ref [rad]
 *  refs[3] Yaw angular rate ref [rad/s]
 */

float Ivz = 0.0, Dvz = 0.0, Ir = 0.0, Dr = 0.0, Ip = 0.0, Dp = 0.0, Iy = 0.0, Dy = 0.0;
float bodyZvelocityPrev = 0.0, bodyRollPrev = 0.0, bodyPitchPrev = 0.0, bodyYawRatePrev = 0.0;

/* Motor output PWM widths [s] */
float t_out[4] = {0.0f};

/* Flight mode */
char flightMode = ATTITUDE;

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

		/* TODO bodyZVelocity calc? */
		/* TODO Refine sensor settings and algorithm (extended Kalman? Kalman? Quaternions) */
		/* TODO Port to STM32F4 (it has SPI (faster than I2C!) for accelerometer) and faster CPU - maybe later */
		/* TODO Be more clever about g, measure it while calibrating? It may still have vertical offset? */
		/* TODO dynamic h / dt in sensor integration and controller? (measure with GetCounter()) */
		/* TODO Calibrate RC input (min, max, midpoint) and map to according position and angle references */
		/* TODO Failsafe - what happens when no RC signal is received? Check receiver PWM - does it keep outputting the same after transmitter shutdown? */
		/* TODO Use on-board LEDs to indicate calibration, warnings, modes etc. */
		/* TODO PWM input chan 5 chan 6 (what controller input maps to chan 6?) - set mode (manual / control / autonomous) */
		/* TODO Better identify drag coefficient (for yaw control allocation) and also thrust coefficient - experiment setup needed */
		/* TODO If STM32F3Discovery not placed in middle of quadcopter, translate sensor rotations? - wait until FCB has been mounted, then measure distances */
		/* TODO Control integration anti-windup */
		/* TODO Control bumpless transfer between modes */
		/* TODO Flight modes and control performance settings (slow, normal, aggressive) */
		/* TODO Trajectory (from x, y, z and heading refs) and hold position at destinations */
		/* TODO Calibration reset if not satisfactory */
		/* TODO Magnetometer soft-iron distortion calibration scheme */

		if(!GetMagCalibrated())
		{
			STM_EVAL_LEDOn(LED3);

			ReadSensors();
			CalibrateMag();

			// Set motor output to lowest
			t_out[0] = t_out[1] = t_out[2] = t_out[3] = MIN_ESC_VAL;
			SetMotors();
		}
		else if(!GetAccCalibrated() || !GetGyroCalibrated())
		{
			STM_EVAL_LEDOn(LED5);

			ReadSensors();
			CalibrateAcc();
			CalibrateGyro();

			// Set motor output to lowest
			t_out[0] = t_out[1] = t_out[2] = t_out[3] = MIN_ESC_VAL;
			SetMotors();
		}
		else if(flightMode == ATTITUDE)
		{
			STM_EVAL_LEDOn(LED7);

			ReadSensors();

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();
			Heading = GetHeading();

			GetPWMInputTimes(&PWMInputTimes);
			SetReferenceSignals();

			AltitudeControl();
			RollControl();
			PitchControl();
			YawControl();

			ControlAllocation();
			SetMotors();
		}
		else if(flightMode == SHUTDOWN){
			// Set motor output to lowest
			// TODO soft stop

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();

			GetPWMInputTimes(&PWMInputTimes);
			SetReferenceSignals();

			AltitudeControl();
			RollControl();
			PitchControl();
			YawControl();

			// Set motor output to lowest
			t_out[0] = t_out[1] = t_out[2] = t_out[3] = MIN_ESC_VAL;
			SetMotors();
		}
	}
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

	U[0] = (Pvz + Ivz + Dvz + g) * MASS; // /cosf(BodyAttitude[0])*cosf(BodyAttitude[1]) angle boost

	// Saturation
	if(U[0] < 0)
		U[0] = 0;
	else if(U[0] > MAX_THRUST)
		U[0] = MAX_THRUST;

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

/* SetControlSignals
 * @brief  Sets the reference values based on RC controller input
 * @param  None
 * @retval None
 */
void SetReferenceSignals(void)
{
	// Set velocity reference limits
	if(PWMInputTimes.PWM_Time1 >= RCmin && PWMInputTimes.PWM_Time1 <= RCmax)
		refs[0] = 2*MAX_Z_VELOCITY*1000*(PWMInputTimes.PWM_Time1-RCmid);
	else
		refs[0] = -MAX_Z_VELOCITY;

	// Set roll reference limits
	if (PWMInputTimes.PWM_Time2 >= RCmin && PWMInputTimes.PWM_Time2 < RCmax)
		refs[1] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.PWM_Time2-RCmid);
	else
		refs[1] = 0;

	// Set pitch reference limits
	if (PWMInputTimes.PWM_Time3 >= RCmin && PWMInputTimes.PWM_Time3 < RCmax)
		refs[2] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.PWM_Time3-RCmid);
	else
		refs[2] = 0;

	// Set yaw rate reference limits
	if (PWMInputTimes.PWM_Time4 >= RCmin && PWMInputTimes.PWM_Time4 < RCmax)
		refs[3] = 2*MAX_YAW_RATE*1000*(PWMInputTimes.PWM_Time4-RCmid);
	else
		refs[3] = 0;
}

/* ControlAllocation
 * @brief  Allocates the desired thrust force and moments to corresponding motor action.
 * 		   Data has been fitted to map thrust force [N] and roll/pitch/yaw moments [Nm] to
 * 		   motor output PWM widths [us] of each of the four motors.
 * @param  None
 * @retval None
 */
void ControlAllocation(void)
{
	t_out[0] = (BQ*LENGTH_ARM*U[0] - M_SQRT2*BQ*U[1] - M_SQRT2*BQ*U[2] - AT*LENGTH_ARM*U[3] - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	t_out[1] = (BQ*LENGTH_ARM*U[0] + M_SQRT2*BQ*U[1] - M_SQRT2*BQ*U[2] + AT*LENGTH_ARM*U[3] - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	t_out[2] = (BQ*LENGTH_ARM*U[0] + M_SQRT2*BQ*U[1] + M_SQRT2*BQ*U[2] - AT*LENGTH_ARM*U[3] - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	t_out[3] = (BQ*LENGTH_ARM*U[0] - M_SQRT2*BQ*U[1] + M_SQRT2*BQ*U[2] + AT*LENGTH_ARM*U[3] - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);

	if(t_out[0] > MAX_ESC_VAL)
		t_out[0] = MAX_ESC_VAL;
	else if(t_out[0] >= MIN_ESC_VAL)
		t_out[0] = t_out[0];
	else
		t_out[0] = MIN_ESC_VAL;

	if(t_out[1] > MAX_ESC_VAL)
			t_out[1] = MAX_ESC_VAL;
	else if(t_out[1] >= MIN_ESC_VAL)
		t_out[1] = t_out[1];
	else
		t_out[1] = MIN_ESC_VAL;

	if(t_out[2] > MAX_ESC_VAL)
		t_out[2] = MAX_ESC_VAL;
	else if(t_out[2] >= MIN_ESC_VAL)
		t_out[2] = t_out[2];
	else
		t_out[2] = MIN_ESC_VAL;

	if(t_out[3] > MAX_ESC_VAL)
		t_out[3] = MAX_ESC_VAL;
	else if(t_out[3] >= MIN_ESC_VAL)
		t_out[3] = t_out[3];
	else
		t_out[3] = MIN_ESC_VAL;
}

/* @SetMotors
 * @brief	Sets the motor PWM, which is sent to the ESCs
 * @param	None.
 * @retval	None.
 */
void SetMotors()
{
	TIM4->CCR1 = GetPWM_CCR(t_out[0]);	// To motor 1 (PD12)
	TIM4->CCR2 = GetPWM_CCR(t_out[1]);	// To motor 2 (PD13)
	TIM4->CCR3 = GetPWM_CCR(t_out[2]);	// To motor 3 (PD14)
	TIM4->CCR4 = GetPWM_CCR(t_out[3]);	// To motor 4 (PD15)
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

/* @TIM3_Setup
 * @brief	Sets up the Timer 3 timebase. Timer 3 is responsible for
 * 			generating system interrupts at well-defined intervals used
 * 			to execute code at discrete time intervals.
 * @param	None.
 * @retval	None.
 */
void TIM3_Setup(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;		// TIM3 init struct

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

/* TIM3_SetupIRQ
 * @brief  Configures the Timer 3 IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM3_SetupIRQ(void)
{

	/* Interrupt config */
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x00;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}
