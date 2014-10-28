/* Control sample time settings */
#define TIM3_FREQ 		1000000	// Timer 3 (counter update) frequency [Hz]
#define TIM3_CTRLFREQ	50		// Control output sample frequency [Hz]

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
#define Kvz 				8.0
#define TIvz				0.0
#define TDvz				0.5
#define BETAvz				1.0			// Proportional set-point weighting
#define Nvz					15.0		// Max derivative gain (often 10-20)
#define MAX_Z_VELOCITY		2.0			// Max vertical velocity (+/-) [m/s]
#define MAX_THRUST			49.60		// Maximal upward thrust [N]

/* Roll/pitch control parameters */
#define Krp					8.0			// Roll/pitch angle controller parameters
#define TIrp				0.0
#define TDrp				0.875
#define BETArp				1.0			// Proportional set-point weighting
#define Nrp					15.0		// Max derivative gain (often 10-20)
#define MAX_ROLLPITCH_ANGLE 10*PI/180	// Max roll/pitch angle (+/-) [rad] (NOTE! Not in deg!)
#define MAX_ROLLPITCH_MOM	MAX_THRUST*1.4142*LENGTH_ARM

/* Yaw control parameters */
#define Kyr 				2.0
#define TIyr 				0.0
#define TDyr				0.5
#define BETAyr				1.0			// Proportional set-point weighting
#define Nyr					15.0		// Max derivative gain (often 10-20)
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
void SetMotors(void);
