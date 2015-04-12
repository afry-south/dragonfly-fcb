/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

typedef struct
{
	float M1;
	float M2;
	float M3;
	float M4;
}PWMMotor_TimeTypeDef;

typedef struct
{
	float Thrust;
	float Roll;
	float Pitch;
	float Yaw;
}CtrlSignals_TypeDef;

typedef struct
{
	float ZVelocity;	// [m/s]
	float RollAngle;	// [rad]
	float PitchAngle;	// [rad]
	float YawRate;		// [rad/s]
}RefSignals_TypeDef;

typedef struct
{
	float K;		// PID gain parameter
	float Ti;		// PID integration time parameter
	float Td;		// PID derivative time parameter
	float B;		// Set-point weighting 0-1
	float N;		// Derivative action filter constant
	float P;		// Proportional control part
	float I;		// Integration control part
	float D;		// Derivative control part
	float PreState;	// Previous value of control variable state
}PIDController_TypeDef;

/* Define variables */

/* Control sample time settings */
#define TIM7_FREQ 		1000000				// Timer 3 (counter update) frequency [Hz]
#define TIM7_CTRLFREQ	50					// Control output sample frequency [Hz]
#define H 				1/TIM7_CTRLFREQ		// Control sample time [s]

/* Physical properties of aircraft */
#define LENGTH_ARM 	0.30		// Quadcopter arm length [m]
#define MASS		2.100		// Total mass of the quadcopter [kg]
#define IXX			0.030		// X-axis moment of inertia [kg/(m^2)]
#define IYY			0.030		// Y-axis moment of inertia [kg/(m^2)]
#define IZZ			0.060		// Z-axis moment of inertia [kg/(m^2)]

/* Data fitting variables to map physical outputs to PWM widths
 * Thrust T = AT*t_out + CT 		[Unit: N] [t_out unit in seconds]
 * Draq torque Q = BQ*t_out + DQ	[Unit: Nm]
 */
#define AT 			15520.7
#define CT 			-18.6312
#define BQ 			478.966
#define DQ 			-0.577590

/* Vertical control parameters */
#define K_VZ 				8.0
#define TI_VZ				0.0
#define TD_VZ				0.5
#define BETA_VZ				1.0			// Proportional set-point weighting
#define N_VZ				15.0		// Max derivative gain (often 10-20)
#define MAX_Z_VELOCITY		2.0			// Max vertical velocity (+/-) [m/s]
#define MAX_THRUST			49.60		// Maximal upward thrust from all four motors combined [N]

/* Roll/pitch control parameters */
#define K_RP				8.0			// Roll/pitch angle controller parameters
#define TI_RP				0.0
#define TD_RP				0.875
#define BETA_RP				1.0			// Proportional set-point weighting
#define N_RP				15.0		// Max derivative gain (often 10-20)
#define MAX_ROLLPITCH_ANGLE 10*PI/180	// Max roll/pitch angle (+/-) [rad] (NOTE! Not in deg!)
#define MAX_ROLLPITCH_MOM	MAX_THRUST*1.4142*LENGTH_ARM

/* Yaw control parameters */
#define K_YR 				2.0
#define TI_YR 				0.0
#define TD_YR				0.5
#define BETA_YR				1.0			// Proportional set-point weighting
#define N_YR				15.0		// Max derivative gain (often 10-20)
#define MAX_YAW_RATE		10*PI/180	// Max yaw angle rate [rad/s] (NOTE! Not deg/s)

/* ESC range */
#define MAX_ESC_VAL 0.002
#define MIN_ESC_VAL 0.001

/* Flight modes */
#define SHUTDOWN 	0
#define MANUAL		1
#define ATTITUDE	2
#define VELOCITY	3
#define	AUTONOMOUS	4

/* Flight performance */
#define SLOW_FLIGHT 	0
#define NORMAL_FLIGHT	1
#define FAST_FLIGHT		2

#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_it.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

/* Function prototype declarations */

uint16_t GetPWM_CCR(float dutycycle);
void UpdateControl(void);
void TIM7_Setup(void);
void TIM7_SetupIRQ(void);
void ControlAllocation(void);
void ManualModeAllocation(void);
void SetReferenceSignals(void);
void UpdateBodyAttitude(void);
void UpdateBodyVelocity(void);
void AltitudeControl(void);
void RollControl(void);
void PitchControl(void);
void YawControl(void);
void SetMotors(void);
void SetFlightMode(void);
void InitPIDControllers(void);

#endif /* __CONTROL_H */

/* **** END OF FILE ****/
