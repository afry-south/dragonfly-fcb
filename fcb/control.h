/* Control sample time settings */
#define TIM3_FREQ 		1000000	// Timer 3 (counter update) frequency [Hz]
#define TIM3_CTRLFREQ	100		// Control output sample frequency [Hz]

/* Data fitting variables to map physical outputs to PWM widths
 * Thrust T = At*t_out^2 + Ct 		[Unit: N]
 * Draq torque Q = Bq*t_out^2 + Dq	[Unit: Nm]
 */
#define L 	0.30		// Quadcopter arm length
#define At 	4187385		// T = A*t_out^2+C
#define Ct 	-4.187385	// (Thrust coeffs)
#define Bq 	128305		// Q = B*t_out^2+D
#define Dq 	-0.128305	// (drag torque coeffs, d not used?)

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
void SetControlSignals(void);
