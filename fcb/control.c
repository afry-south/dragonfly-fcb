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
#include "com.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/

/* Timer and sampling */
volatile const float h = 1/((float)TIM7_CTRLFREQ);	// Control sample time [s]

/* Measured states */
float BodyAttitude[3] = {0.0f};	// Body-frame roll, pitch, yaw angles [rad]
float BodyVelocity[3] = {0.0f};	// Body-frame velocities [m/s]
float YawRate = 0.0;
float Heading = 0.0;

PWMRC_TimeTypeDef PWMInputTimes;	// 6-channel PWM input width in seconds

CtrlSignals_TypeDef CtrlSignals;	// Physical control signals

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
PWMMotor_TimeTypeDef PWMMotorTimes;

/* Flight mode */
char flightMode = MANUAL;

/* @TIM7_IRQHandler
 * @brief	Timer 7 interrupt handler.
 * 			Continually performs program duties with regular intervals.
 * @param	None.
 * @retval	None.
 */
void TIM7_IRQHandler()
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);

		ReadSensors();	// Reads gyroscope, accelerometer and magnetometer
		GetPWMInputTimes(&PWMInputTimes);	// Get 6 channel RC input pulse widths
		SetFlightMode();

		// Read/write USB Virtual COM
		//rwUSB();

		if(!GetMagCalibrated())
		{
			/* _COMPASS CALIBRATION INSTRUCTIONS_
			 * Rotate the quadcopter around each of the positive and negative 3D axes (6 directions)
			 * at least 360 deg (not too fast).
			 * The alignment does not need to be exact and further arbitrary rotatation will
			 * only be beneficial for the calibration.
			 * It does not matter in which direction the quadcopter is rotated.
			 * */

			STM_EVAL_LEDOn(LED3);

			ReadSensors();
			CalibrateMag();

			// Set motor output to lowest
			PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
			SetMotors();
		}
		else if(!GetAccCalibrated())
		{
			/* _ACCELEROMETER CALIBRATION INSTRUCTIONS_
			 * Hold the quadcopter still for a few seconds in each of the following positions:
			 * Level, upside-down, left side down, right side down, front down, rear down.
			 * TODO Not implemented yet...
			 * */

			STM_EVAL_LEDOn(LED5);

			CalibrateAcc();

			// Set motor output to lowest
			PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
			SetMotors();
		}
		else if(!GetGyroCalibrated())
		{
			/* _GYROSCOPE CALIBRATION INSTRUCTIONS_
			 * Bla bla
			 * */

			STM_EVAL_LEDOn(LED7);

			CalibrateGyro();
			GetBodyAttitude(BodyAttitude, h);

			// TODO Init attitude

			// Set motor output to lowest
			PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
			SetMotors();
		}
		else if(flightMode == ATTITUDE)
		{
			STM_EVAL_LEDOff(LED7);
			STM_EVAL_LEDOff(LED8);
			STM_EVAL_LEDOff(LED10);
			STM_EVAL_LEDOn(LED9);

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();
			Heading = GetHeading();

			SetReferenceSignals();
			AltitudeControl();
			RollControl();
			PitchControl();
			YawControl();

			ControlAllocation();
			SetMotors();
		}
		else if(flightMode == MANUAL)
		{
			STM_EVAL_LEDOff(LED7);
			STM_EVAL_LEDOff(LED8);
			STM_EVAL_LEDOff(LED9);
			STM_EVAL_LEDOn(LED10);

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();

			// Set motor output to lowest
			ManualModeAllocation();
			SetMotors();
		}
		else if(flightMode == SHUTDOWN)
		{
			STM_EVAL_LEDOff(LED7);
			STM_EVAL_LEDOff(LED9);
			STM_EVAL_LEDOff(LED10);
			STM_EVAL_LEDOn(LED8);

			GetBodyVelocity(BodyVelocity, h);
			GetBodyAttitude(BodyAttitude, h);
			YawRate = GetYawRate();

			// Set motor output to lowest
			PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
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

	CtrlSignals.Thrust = (Pvz + Ivz + Dvz + g) * MASS; // /cosf(BodyAttitude[0])*cosf(BodyAttitude[1]) angle boost

	// Saturation
	if(CtrlSignals.Thrust < 0)
		CtrlSignals.Thrust = 0;
	else if(CtrlSignals.Thrust > MAX_THRUST)
		CtrlSignals.Thrust = MAX_THRUST;

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
	CtrlSignals.Roll = (Pr + Ir + Dr) * IXX;

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
	CtrlSignals.Pitch = (Pp + Ip + Dp) * IYY;

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
	CtrlSignals.Yaw = (Py + Iy + Dy) * IZZ;

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
	if(PWMInputTimes.Throttle >= GetRCmin() && PWMInputTimes.Throttle <= GetRCmax())
		refs[0] = 2*MAX_Z_VELOCITY*1000*(PWMInputTimes.Throttle-GetRCmid());
	else
		refs[0] = -MAX_Z_VELOCITY;

	// Set roll reference limits
	if (PWMInputTimes.Aileron >= GetRCmin() && PWMInputTimes.Aileron <= GetRCmax())
		refs[1] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.Aileron-GetRCmid());
	else
		refs[1] = GetRCmid();

	// Set pitch reference limits
	if (PWMInputTimes.Elevator >= GetRCmin() && PWMInputTimes.Elevator <= GetRCmax())
		refs[2] = 2*MAX_ROLLPITCH_ANGLE*1000*(PWMInputTimes.Elevator-GetRCmid());
	else
		refs[2] = GetRCmid();

	// Set yaw rate reference limits
	if (PWMInputTimes.Rudder >= GetRCmin() && PWMInputTimes.Rudder <= GetRCmax())
		refs[3] = 2*MAX_YAW_RATE*1000*(PWMInputTimes.Rudder-GetRCmid());
	else
		refs[3] = GetRCmid();
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
	PWMMotorTimes.M1 = (BQ*LENGTH_ARM*CtrlSignals.Thrust - M_SQRT2*BQ*CtrlSignals.Roll - M_SQRT2*BQ*CtrlSignals.Pitch - AT*LENGTH_ARM*CtrlSignals.Yaw - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	PWMMotorTimes.M2 = (BQ*LENGTH_ARM*CtrlSignals.Thrust + M_SQRT2*BQ*CtrlSignals.Roll - M_SQRT2*BQ*CtrlSignals.Pitch + AT*LENGTH_ARM*CtrlSignals.Yaw - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	PWMMotorTimes.M3 = (BQ*LENGTH_ARM*CtrlSignals.Thrust + M_SQRT2*BQ*CtrlSignals.Roll + M_SQRT2*BQ*CtrlSignals.Pitch - AT*LENGTH_ARM*CtrlSignals.Yaw - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);
	PWMMotorTimes.M4 = (BQ*LENGTH_ARM*CtrlSignals.Thrust - M_SQRT2*BQ*CtrlSignals.Roll + M_SQRT2*BQ*CtrlSignals.Pitch + AT*LENGTH_ARM*CtrlSignals.Yaw - 4*BQ*CT*LENGTH_ARM) / ((float)4*AT*BQ*LENGTH_ARM);

	if(PWMMotorTimes.M1 > MAX_ESC_VAL)
		PWMMotorTimes.M1 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M1 >= MIN_ESC_VAL)
		PWMMotorTimes.M1 = PWMMotorTimes.M1;
	else
		PWMMotorTimes.M1 = MIN_ESC_VAL;

	if(PWMMotorTimes.M2 > MAX_ESC_VAL)
			PWMMotorTimes.M2 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M2 >= MIN_ESC_VAL)
		PWMMotorTimes.M2 = PWMMotorTimes.M2;
	else
		PWMMotorTimes.M2 = MIN_ESC_VAL;

	if(PWMMotorTimes.M3 > MAX_ESC_VAL)
		PWMMotorTimes.M3 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M3 >= MIN_ESC_VAL)
		PWMMotorTimes.M3 = PWMMotorTimes.M3;
	else
		PWMMotorTimes.M3 = MIN_ESC_VAL;

	if(PWMMotorTimes.M4 > MAX_ESC_VAL)
		PWMMotorTimes.M4 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M4 >= MIN_ESC_VAL)
		PWMMotorTimes.M4 = PWMMotorTimes.M4;
	else
		PWMMotorTimes.M4 = MIN_ESC_VAL;
}

/* ManualModeAllocation
 * @brief  Manual mode allocation - allocates raw RC input.
 * @param  None
 * @retval None
 */
void ManualModeAllocation(void)
{
	PWMMotorTimes.M1 = 0.9*(PWMInputTimes.Throttle - 2*(PWMInputTimes.Aileron-GetRCmid()) - 2*(PWMInputTimes.Elevator-GetRCmid()) - 2*(PWMInputTimes.Rudder-GetRCmid()));
	PWMMotorTimes.M2 = 0.9*(PWMInputTimes.Throttle + 2*(PWMInputTimes.Aileron-GetRCmid()) - 2*(PWMInputTimes.Elevator-GetRCmid()) + 2*(PWMInputTimes.Rudder-GetRCmid()));
	PWMMotorTimes.M3 = 0.9*(PWMInputTimes.Throttle + 2*(PWMInputTimes.Aileron-GetRCmid()) + 2*(PWMInputTimes.Elevator-GetRCmid()) - 2*(PWMInputTimes.Rudder-GetRCmid()));
	PWMMotorTimes.M4 = 0.9*(PWMInputTimes.Throttle - 2*(PWMInputTimes.Aileron-GetRCmid()) + 2*(PWMInputTimes.Elevator-GetRCmid()) + 2*(PWMInputTimes.Rudder-GetRCmid()));

	if(PWMMotorTimes.M1 > MAX_ESC_VAL)
		PWMMotorTimes.M1 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M1 >= MIN_ESC_VAL)
		PWMMotorTimes.M1 = PWMMotorTimes.M1;
	else
		PWMMotorTimes.M1 = MIN_ESC_VAL;

	if(PWMMotorTimes.M2 > MAX_ESC_VAL)
			PWMMotorTimes.M2 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M2 >= MIN_ESC_VAL)
		PWMMotorTimes.M2 = PWMMotorTimes.M2;
	else
		PWMMotorTimes.M2 = MIN_ESC_VAL;

	if(PWMMotorTimes.M3 > MAX_ESC_VAL)
		PWMMotorTimes.M3 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M3 >= MIN_ESC_VAL)
		PWMMotorTimes.M3 = PWMMotorTimes.M3;
	else
		PWMMotorTimes.M3 = MIN_ESC_VAL;

	if(PWMMotorTimes.M4 > MAX_ESC_VAL)
		PWMMotorTimes.M4 = MAX_ESC_VAL;
	else if(PWMMotorTimes.M4 >= MIN_ESC_VAL)
		PWMMotorTimes.M4 = PWMMotorTimes.M4;
	else
		PWMMotorTimes.M4 = MIN_ESC_VAL;
}

/* @SetMotors
 * @brief	Sets the motor PWM, which is sent to the ESCs
 * @param	None.
 * @retval	None.
 */
void SetMotors()
{
	TIM4->CCR1 = GetPWM_CCR(PWMMotorTimes.M1);	// To motor 1 (PD12)
	TIM4->CCR2 = GetPWM_CCR(PWMMotorTimes.M2);	// To motor 2 (PD13)
	TIM4->CCR3 = GetPWM_CCR(PWMMotorTimes.M3);	// To motor 3 (PD14)
	TIM4->CCR4 = GetPWM_CCR(PWMMotorTimes.M4);	// To motor 4 (PD15)
}

/* @SetFlightMode
 * @brief	Sets the Flight Mode.
 * @param	None.
 * @retval	None.
 */
void SetFlightMode()
{
	if(CheckRCConnection())
	{
		if (PWMInputTimes.Gear >= GetRCmid())
			flightMode = ATTITUDE;
		else if(PWMInputTimes.Gear < GetRCmid() && PWMInputTimes.Gear >= 0.0)
			flightMode = MANUAL;
	}
	else
	{
		flightMode = SHUTDOWN;
	}
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

/* @TIM6_Setup
 * @brief	Sets up the Timer 7 timebase. Timer 7 is responsible for
 * 			generating system interrupts at well-defined intervals used
 * 			to execute code at discrete time intervals.
 * @param	None.
 * @retval	None.
 */
void TIM7_Setup(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure7;	// TIM7 init struct

	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/* TIM7 Time Base configuration */
	TIM_TimeBaseStructure7.TIM_Prescaler = SystemCoreClock/TIM7_FREQ - 1;	// Scaling of system clock freq
	TIM_TimeBaseStructure7.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure7.TIM_Period = TIM7_FREQ/TIM7_CTRLFREQ - 1;		// Counter reset value
	TIM_TimeBaseStructure7.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure7.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure7);

	/* TIM7 counter enable */
	TIM_Cmd(TIM7, ENABLE);
}

/* TIM7_SetupIRQ
 * @brief  Configures the Timer 7 IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM7_SetupIRQ(void)
{

	/* Interrupt config */
    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    nvicStructure.NVIC_IRQChannelSubPriority = 0x00;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}
