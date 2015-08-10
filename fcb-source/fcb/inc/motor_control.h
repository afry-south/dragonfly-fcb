/*****************************************************************************
* @file    motor_control.h
* @author  ÅF Embedded Systems SydDragonfly
*          Daniel Stenberg
* @version v. 1.0.0
* @date    2015-05-07
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          Header file for motor PWM output
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

/* Includes */
#include "stm32f3xx.h"

/* Exported types ------------------------------------------------------------*/
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

/* Exported functions ------------------------------------------------------- */
void MotorControl_Config(void);
void SetMotor1(uint16_t ctrlVal);
void SetMotor2(uint16_t ctrlVal);
void SetMotor3(uint16_t ctrlVal);
void SetMotor4(uint16_t ctrlVal);

#endif /* __MOTOR_CONTROL_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
