/* Control sample time settings */
#define TIM3_FREQ 		1000000	// Timer 3 (counter update) frequency [Hz]
#define TIM3_CTRLFREQ	100		// Control output sample frequency [Hz]

/* Data fitting variables to map physical outputs to PWM widths
 * Thrust T = At*t_out^2 + Ct 		[Unit: N]
 * Draq torque Q = Bq*t_out^2 + Dq	[Unit: Nm]
 */
#define L 		0.30		// Quadcopter arm length
#define At 		4187385		// T = A*t_out^2+C
#define Ct 		-4.187385	// (Thrust coeffs)
#define Bq 		128305		// Q = B*t_out^2+D
#define Dq 		-0.128305	// (drag torque coeffs, d not used?)

/* Vertical control parameters */
#define Kvz 				16.0
#define TIvz				0.0
#define TDvz				0.5
#define BETAvz				1.0		// Proportional set-point weighting
#define Nvz					20.0	// Max derivative gain (often 10-20)
#define MAX_Z_VELOCITY		5.0		// Max vertical velocity (+/-) [m/s]

/* Roll/pitch control parameters */
#define Krp					16.0	// Roll/pitch angle controller parameters
#define TIrp				0.0
#define TDrp				0.5
#define BETArp				1.0		// Proportional set-point weighting
#define Nrp					20.0	// Max derivative gain (often 10-20)
#define MAX_ROLLPITCH_ANGLE 0.35	// Max vertical velocity (+/-) [rad] (NOTE! Not in deg!)

/* Yaw control parameters */
#define Kyr 				4.0
#define TIyr 				0.0
#define TDyr				1.0
#define BETAyr				1.0		// Proportional set-point weighting
#define Nyr					20.0	// Max derivative gain (often 10-20)
#define MAX_YAW_RATE		0.78	// Max yaw angle rate [rad/s] (NOTE! Not deg/s)

#define M		2.500		// Total mass of the quadcopter [kg]
#define IXX		0.030		// X-axis moment of inertia [kg/(m^2)]
#define IYY		0.030		// Y-axis moment of inertia [kg/(m^2)]
#define IZZ		0.060		// Z-axis moment of inertia [kg/(m^2)]

#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_it.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

uint16_t GetPWM_CCR(float dutycycle);
void TIM3_IRQHandler(void);
void TIM3_Setup(void);
void TIM3_SetupIRQ(void);
void ControlAllocation(void);
void SetReferenceSignals(void);
void UpdateBodyAttitude(void);
void UpdateBodyVelocity(void);
void AltitudeControl(void);
void RollControl(void);
void PitchControl(void);
void YawControl(void);
