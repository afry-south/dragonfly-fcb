/*****************************************************************************
* @file    fcb/motor_control.h
* @author  �F Embedded Systems SydDragonfly
*          Daniel Stenberg
* @version v. 1.0.0
* @date    2015-05-07
* @brief   Flight Control program for the �F Dragonfly quadcopter
*          Header file for motor PWM output
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_OUTPUT_H
#define __MOTOR_OUTPUT_H

/* Includes */
#include "stm32f3xx.h"

/* Defines for Motor TIM clock source */
#define TIM_MOTOR                               TIM4
#define MOTOR_TIM_CLK_ENABLE()                  __TIM4_CLK_ENABLE()

/* Defines for Motor TIM channel pins */
#define MOTOR_TIM_CHANNEL_GPIO_PORT()           __GPIOD_CLK_ENABLE()
#define MOTOR_TIM_AF                            GPIO_AF2_TIM4
#define MOTOR_GPIO_PIN_CHANNEL1                 GPIO_PIN_12
#define MOTOR_GPIO_PIN_CHANNEL2                 GPIO_PIN_13
#define MOTOR_GPIO_PIN_CHANNEL3                 GPIO_PIN_14
#define MOTOR_GPIO_PIN_CHANNEL4                 GPIO_PIN_15

/* Defines to enumerate motors */
#define MOTOR1_CHANNEL                          TIM_CHANNEL_1
#define MOTOR2_CHANNEL                          TIM_CHANNEL_2
#define MOTOR3_CHANNEL                          TIM_CHANNEL_3
#define MOTOR4_CHANNEL                          TIM_CHANNEL_4

/* Defines Motor TIM Timebase */
// 60000 ticks on a 24MHz clock yields a 400 MHz PWM frequency
#define MOTOR_OUTPUT_SAMPLECLOCK                24000000
#define MOTOR_OUTPUT_PERIOD                     60000

/* ESC range */
#define MAX_ESC_VAL                             2000
#define MID_ESC_CAL                             1500
#define MIN_ESC_VAL                             1000

/* Function prototypes */
void MotorControl_Config(void);
void SetMotor1(uint16_t ccrVal);
void SetMotor2(uint16_t ccrVal);
void SetMotor3(uint16_t ccrVal);
void SetMotor4(uint16_t ccrVal);

#endif /* __MOTOR_OUTPUT_H */
