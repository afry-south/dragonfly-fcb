/******************************************************************************
 * @file    fcb/RCinput.h
 * @author  ÅF Dragonfly - Daniel Nilsson and Daniel Stenberg, Embedded Systems
 * @version v. 0.0.2
 * @date    2015-04-16
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          Header file for reading signals from the RC receiver
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RCINPUT_H
#define __RCINPUT_H

typedef struct
{
	float Throttle;
	float Aileron;
	float Elevator;
	float Rudder;
	float Gear;
	float Auxiliary;
}PWMRC_TimeTypeDef;

void UpdateThrottleChannel(void);
void UpdateAileronChannel(void);
void UpdateElevatorChannel(void);
void UpdateRudderChannel(void);
void UpdateGearChannel(void);
void UpdateAuxiliaryChannel(void);
void TIM2_Setup(void);
void TIM3_Setup(void);
void PWM_In_Setup(void);
void GetPWMInputTimes(PWMRC_TimeTypeDef *PWM_Time);
char CheckRCConnection(void);
float GetRCmin(void);
float GetRCmid(void);
float GetRCmax(void);

#endif /* __RCINPUT_H */
