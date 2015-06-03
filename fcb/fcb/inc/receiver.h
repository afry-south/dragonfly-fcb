/******************************************************************************
 * @file    receiver.h
 * @author  ÅF Dragonfly
 * @version v. 0.0.2
 * @date    2015-04-16
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          Header file for reading signals from the RC receiver
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECEIVER_H
#define __RECEIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
typedef enum
{
  PULSE_LOW = 0,
  PULSE_HIGH = !PULSE_LOW
} Pulse_State;

typedef enum
{
  RECEIVER_ERROR = 0,
  RECEIVER_OK = !RECEIVER_ERROR
} ReceiverErrorStatus;

/* Exported types ------------------------------------------------------------*/

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
#define PRIMARY_RECEIVER_THROTTLE_ACTIVE_CHANNEL        HAL_TIM_ACTIVE_CHANNEL_1
#define PRIMARY_RECEIVER_AILERON_CHANNEL                TIM_CHANNEL_2
#define PRIMARY_RECEIVER_AILERON_ACTIVE_CHANNEL         HAL_TIM_ACTIVE_CHANNEL_2
#define PRIMARY_RECEIVER_ELEVATOR_CHANNEL               TIM_CHANNEL_3
#define PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL        HAL_TIM_ACTIVE_CHANNEL_3
#define PRIMARY_RECEIVER_RUDDER_CHANNEL                 TIM_CHANNEL_4
#define PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL          HAL_TIM_ACTIVE_CHANNEL_4

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
#define AUX_RECEIVER_GEAR_ACTIVE_CHANNEL                HAL_TIM_ACTIVE_CHANNEL_1
#define AUX_RECEIVER_AUX1_CHANNEL                       TIM_CHANNEL_2
#define AUX_RECEIVER_AUX1_ACTIVE_CHANNEL                HAL_TIM_ACTIVE_CHANNEL_2

/* Common definitions for the receiver TIM timers ############################*/
/* Defintions for receiver TIM timebase */
#define RECEIVER_TIM_COUNTER_CLOCK                      2400000

/* RC min max default count values */
#define RECEIVER_DEFAULT_MAX_COUNT                      4608    // Corresponds to 1.92 ms with 2,4Mhz counter clock
#define RECEIVER_DEFAULT_MIN_COUNT                      2592    // Corresponds to 1.08 ms with 2,4Mhz counter clock

/* Used for sanity check of IC count*/
#define RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT             6000    // Corresponds to 2.50 ms with 2,4Mhz counter clock
#define RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT             1200    // Corresponds to 0.50 ms with 2,4Mhz counter clock

/* Exported functions ------------------------------------------------------- */
ReceiverErrorStatus ReceiverInput_Config(void);
uint16_t GetThrottleReceiverChannel(void);
int16_t GetAileronReceiverChannel(void);
int16_t GetElevatorReceiverChannel(void);
int16_t GetRudderReceiverChannel(void);
int16_t GetGearReceiverChannel(void);
int16_t GetAux1ReceiverChannel(void);
void CalibrateReceiver(void);
ReceiverErrorStatus IsReceiverActive(void);

#endif /* __RECEIVER_H */
