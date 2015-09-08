/*****************************************************************************
 * @file    motor_control.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-05-07
 * @brief   Flight Control program for the �F Dragonfly quadcopter
 *          Header file for motor PWM output
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

#include "flight_control.h"
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
	MOTORCTRL_ERROR = 0, MOTORCTRL_OK = !MOTORCTRL_ERROR
} MotorControlErrorStatus;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Defines for Motor TIM clock source */
#define TIM_MOTOR                               TIM4
#define MOTOR_TIM_CLK_ENABLE()                  __TIM4_CLK_ENABLE()
#define MOTOR_TIM_CLK_DISABLE()                 __TIM4_CLK_DISABLE()

/* Defines for Motor TIM channel pins */
#define MOTOR_TIM_CHANNEL_GPIO_PORT()           __GPIOD_CLK_ENABLE()
#define MOTOR_TIM_AF                            GPIO_AF2_TIM4
#define MOTOR_PIN_PORT                          GPIOD
#define MOTOR_GPIO_PIN_CHANNEL1                 GPIO_PIN_12
#define MOTOR_GPIO_PIN_CHANNEL2                 GPIO_PIN_13
#define MOTOR_GPIO_PIN_CHANNEL3                 GPIO_PIN_14
#define MOTOR_GPIO_PIN_CHANNEL4                 GPIO_PIN_15

/* Defines to enumerate motors */
#define MOTOR1_CHANNEL                          TIM_CHANNEL_1
#define MOTOR2_CHANNEL                          TIM_CHANNEL_2
#define MOTOR3_CHANNEL                          TIM_CHANNEL_3
#define MOTOR4_CHANNEL                          TIM_CHANNEL_4

/* Defines Motor TIM Timebase
 * 60000 ticks on a 24MHz clock yields a 400 MHz PWM frequency */
#define MOTOR_OUTPUT_COUNTER_CLOCK              24000000
#define MOTOR_OUTPUT_PERIOD                     60000

#define ESC_MAX_OUTPUT                          48000 // 2.0 ms pulse
#define ESC_MIN_OUTPUT                          24000 // 1.0 ms pulse

/* Data fitting variables to map physical outputs to motor control values
 * Thrust T(m) = AT*m + BT 		[Unit: N] [t_out unit in seconds]
 * Draq torque Q(m) = AQ*m + BQ	[Unit: Nm]
 */
#define R_PROP		0.1397		// Propeller radius [m]
#define AT 			0.0001768
#define BT 			0.0
#define AQ			0.000001748	// This was calculated based on aerodynamic rotor equations
// #define BQ			0.0

/* Physical control allocation constants */
#define THRUST_ALLOC_COEFF		1/(4*AT)
#define THRUST_ALLOC_OFFSET		-BT/AT
#define ROLLPITCH_ALLOC_COEFF	1*M_SQRT2/(4*AT*LENGTH_ARM)
#define	YAW_ALLOC_COEFF			1/(4*AQ)

/* Exported functions ------------------------------------------------------- */
void MotorControlConfig(void);
void SetMotors(uint16_t ctrlValMotor1, uint16_t ctrlValMotor2, uint16_t ctrlValMotor3, uint16_t ctrlValMotor4);
void MotorAllocationRaw(void);
void MotorAllocationPhysical(const float u1, const float u2, const float u3, const float u4);
void ShutdownMotors(void);

MotorControlErrorStatus StartMotorControlSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
MotorControlErrorStatus StopMotorControlSamplingTask(void);
void PrintMotorControlValues(void);

#endif /* __MOTOR_CONTROL_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
