/******************************************************************************
 * @file    rc_input.h
 * @author  ÅF Dragonfly - Daniel Nilsson and Daniel Stenberg, Embedded Systems
 * @version v. 0.0.2
 * @date    2015-04-16
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          Header file for reading signals from the RC receiver
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RCINPUT_H
#define __RCINPUT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* Exported constants --------------------------------------------------------*/
typedef enum
{
  PWM_LOW = 0,
  PWM_HIGH = !PWM_LOW
} PWM_State;

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  PWM_State ThrottleInputState;
  PWM_State AileronInputState;
  PWM_State ElevatorInputState;
  PWM_State RudderInputState;
  PWM_State GearInputState;
  PWM_State AuxiliaryInputState;
}PWM_Input_Channel_States_TypeDef;

typedef struct
{
  float Throttle;
  float Aileron;
  float Elevator;
  float Rudder;
  float Gear;
  float Auxiliary;
}PWMRC_TimeTypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Definitions for Primary Receiver ##########################################*/
/* Definitions for Primary Receiver TIM clock */
#define PRIMARY_RECEIVER_TIM                            TIM2
#define PRIMARY_RECEIVER_TIM_CLK_ENABLE()               __TIM2_CLK_ENABLE()

/* Definitions for Primary Receiver TIM pins */
#define PRIMARY_RECEIVER_TIM_CHANNEL_GPIO_PORT()        __GPIOD_CLK_ENABLE()
#define PRIMARY_RECEIVER_TIM_AF                         GPIO_AF2_TIM2
#define PRIMARY_RECEIVER_TIM_PIN_PORT                   GPIOD
#define PRIMARY_RECEIVER_PIN_CHANNEL1                   GPIO_PIN_3
#define PRIMARY_RECEIVER_PIN_CHANNEL2                   GPIO_PIN_4
#define PRIMARY_RECEIVER_PIN_CHANNEL3                   GPIO_PIN_6
#define PRIMARY_RECEIVER_PIN_CHANNEL4                   GPIO_PIN_7

/* Definitions for PRIMARY_RECEIVER_TIM NVIC */
#define PRIMARY_RECEIVER_TIM_IRQn                       TIM2_IRQn
#define PRIMARY_RECEIVER_TIM_IRQHandler                 TIM2_IRQHandler
#define PRIMARY_RECEIVER_TIM_IRQ_PREEMPT_PRIO           0
#define PRIMARY_RECEIVER_TIM_IRQ_SUB_PRIO               0

/* Definitions for Primary Receiver channels input */
#define PRIMARY_RECEIVER_THROTTLE_CHANNEL               TIM_CHANNEL_1
#define PRIMARY_RECEIVER_AILERON_CHANNEL                TIM_CHANNEL_2
#define PRIMARY_RECEIVER_ELEVATOR_CHANNEL               TIM_CHANNEL_3
#define PRIMARY_RECEIVER_RUDDER_CHANNEL                 TIM_CHANNEL_4

/* Definitions for Aux Receiver ##############################################*/
/* Definitions for Aux Receiver TIM clock */
#define AUX_RECEIVER_TIM                                TIM3
#define AUX_RECEIVER_TIM_CLK_ENABLE()                   __TIM3_CLK_ENABLE()

/* Definitions for Auxiliary Receiver channels input */
#define AUX_RECEIVER_GEAR_CHANNEL                       TIM_CHANNEL_1
#define AUX_RECEIVER_AUX1_CHANNEL                       TIM_CHANNEL_2

/* Definitions for Aux Receiver TIM pins */
#define AUX_RECEIVER_TIM_CHANNEL_GPIO_PORT()            __GPIOB_CLK_ENABLE()
#define AUX_RECEIVER_TIM_AF                             GPIO_AF2_TIM2
#define AUX_RECEIVER_TIM_PIN_PORT                       GPIOB
#define AUX_RECEIVER_PIN_CHANNEL1                       GPIO_PIN_4
#define AUX_RECEIVER_PIN_CHANNEL2                       GPIO_PIN_5

/* Definitions for AUX_RECEIVER_TIM NVIC */
#define AUX_RECEIVER_TIM_IRQn                           TIM3_IRQn
#define AUX_RECEIVER_TIM_IRQHandler                     TIM3_IRQHandler
#define AUX_RECEIVER_TIM_IRQ_PREEMPT_PRIO               0
#define AUX_RECEIVER_TIM_IRQ_SUB_PRIO                   0

/* Definitions for Aux Receiver channels input */
#define AUX_RECEIVER_GEAR_CHANNEL                       TIM_CHANNEL_1
#define AUX_RECEIVER_AUX1_CHANNEL                       TIM_CHANNEL_2

/* Common definitions for the receiver TIM timers ############################*/
/* Definions for receiver TIM timebase */
#define RECEIVER_TIM_COUNTER_CLOCK                      2400000

/* Exported functions ------------------------------------------------------- */
void PrimaryReceiverInput_Config(void);
void AuxReceiverInput_Config(void);

void UpdateThrottleChannel(void);
void UpdateAileronChannel(void);
void UpdateElevatorChannel(void);
void UpdateRudderChannel(void);
void UpdateGearChannel(void);
void UpdateAuxiliaryChannel(void);

void TIM3_Setup(void);
void PWM_In_Setup(void);
void GetPWMInputTimes(PWMRC_TimeTypeDef *PWM_Time);
char CheckRCConnection(void);
float GetRCmin(void);
float GetRCmid(void);
float GetRCmax(void);

#endif /* __RCINPUT_H */
