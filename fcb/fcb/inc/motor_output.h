/*****************************************************************************
* @file    fcb/motor_output.h
* @author  ÅF Dragonfly - Daniel Stenberg, Embedded Systems
* @version v. 0.0.1
* @date    2014-09-29
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
*          Header file for motor PWM output
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_OUTPUT_H
#define __MOTOR_OUTPUT_H

/* Defines */
#define MOTOR_OUT_SAMPLECLOCK (int) 24000000
#define TIM4_Period (int) 60000

/* Function prototypes */
void TIM4_IOconfig(void);
void TIM4_Setup(void);
void TIM4_SetupOC(void);

#endif /* __MOTOR_OUTPUT_H */
