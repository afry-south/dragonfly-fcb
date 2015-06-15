/******************************************************************************
 * @file    receiver.h
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-04-16
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          Header file for reading signals from the RC receiver
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RECEIVER_H
#define __RECEIVER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* Exported constants --------------------------------------------------------*/
/* Definitions for Primary Receiver ##########################################*/
/* Definitions for Primary Receiver TIM clock */
#define PRIMARY_RECEIVER_TIM                            TIM2
#define PRIMARY_RECEIVER_TIM_CLK_ENABLE()               __TIM2_CLK_ENABLE()
#define PRIMARY_RECEIVER_TIM_CLK_DISABLE()              __TIM2_CLK_DISABLE()

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
#define AUX_RECEIVER_TIM_CLK_DISABLE()                  __TIM3_CLK_DISABLE()

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
#define RECEIVER_TIM_COUNTER_CLOCK                      18000000        // 18 MHz counter update with 16-bit period
#define RECEIVER_COUNTER_PERIOD                         UINT16_MAX      // reset gives ~3.64 ms counter period

/* RC min max default count values
 * The Spektrum AR610 receiver resolution is only 2048, so this should be more than enough!
 * The values below are based on observed values of the Spektrum AR610 */
#define RECEIVER_PULSE_DEFAULT_MAX_COUNT                34560   // Corresponds to 1.92 ms with 18Mhz counter clock
#define RECEIVER_PULSE_DEFAULT_MIN_COUNT                19440   // Corresponds to 1.08 ms with 18Mhz counter clock

/* Used for sanity check of IC pulse count, +/-20% of default count considered valid */
#define RECEIVER_MAX_VALID_IC_PULSE_COUNT               RECEIVER_PULSE_DEFAULT_MAX_COUNT*12/10
#define RECEIVER_MIN_VALID_IC_PULSE_COUNT               RECEIVER_PULSE_DEFAULT_MIN_COUNT*8/10

/* Used for sanity check of IC period count - Period is ~22 ms, +/-10% considered valid */
#define RECEIVER_MAX_VALID_PERIOD_COUNT                 432000
#define RECEIVER_MIN_VALID_PERIOD_COUNT                 360000

/* Receiver calibration definitions - must at least be withing valid pulse range */
#define RECEIVER_MAX_CALIBRATION_DURATION               1200000 // [ms] Max receiver calibration duration
#define RECEIVER_CALIBRATION_MIN_PULSE_COUNT            1000    // Corresponds to ~22.0 s of calibration (assuming period is 22 ms)
#define RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE        16
#define RECEIVER_MAX_CALIBRATION_MAX_PULSE_COUNT        RECEIVER_PULSE_DEFAULT_MAX_COUNT*11/10
#define RECEIVER_MAX_CALIBRATION_MIN_PULSE_COUNT        RECEIVER_PULSE_DEFAULT_MIN_COUNT*9/10
#define RECEIVER_MIN_CALIBRATION_MAX_PULSE_COUNT        RECEIVER_PULSE_DEFAULT_MIN_COUNT*11/10
#define RECEIVER_MIN_CALIBRATION_MIN_PULSE_COUNT        RECEIVER_PULSE_DEFAULT_MIN_COUNT*9/10

#define IS_RECEIVER_CHANNEL_INACTIVE_PERIODS_COUNT      300     // Corresponds to ~1.092 s

/* Exported types ------------------------------------------------------------*/
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

typedef struct
{
  uint16_t ChannelMaxCount;
  uint16_t ChannelMinCount;
}Receiver_IC_ChannelCalibrationValues_TypeDef;

typedef struct
{
  Receiver_IC_ChannelCalibrationValues_TypeDef ThrottleChannel;
  Receiver_IC_ChannelCalibrationValues_TypeDef AileronChannel;
  Receiver_IC_ChannelCalibrationValues_TypeDef ElevatorChannel;
  Receiver_IC_ChannelCalibrationValues_TypeDef RudderChannel;
  Receiver_IC_ChannelCalibrationValues_TypeDef GearChannel;
  Receiver_IC_ChannelCalibrationValues_TypeDef Aux1Channel;
}Receiver_CalibrationValues_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
ReceiverErrorStatus ReceiverInput_Config(void);
ReceiverErrorStatus StartReceiverCalibration(void);
ReceiverErrorStatus StopReceiverCalibration(void);
ReceiverErrorStatus IsReceiverActive(void);

uint16_t GetThrottleReceiverChannel(void);
int16_t GetAileronReceiverChannel(void);
int16_t GetElevatorReceiverChannel(void);
int16_t GetRudderReceiverChannel(void);
int16_t GetGearReceiverChannel(void);
int16_t GetAux1ReceiverChannel(void);

uint16_t GetThrottleReceiverChannelPulseMicros(void);
uint16_t GetAileronReceiverChannelPulseMicros(void);
uint16_t GetElevatorReceiverChannelPulseMicros(void);
uint16_t GetRudderReceiverChannelPulseMicros(void);
uint16_t GetGearReceiverChannelPulseMicros(void);
uint16_t GetAux1ReceiverChannelPulseMicros(void);

uint16_t GetThrottleReceiverChannelPeriodMicros(void);
uint16_t GetAileronReceiverChannelPeriodMicros(void);
uint16_t GetElevatorReceiverChannelPeriodMicros(void);
uint16_t GetRudderReceiverChannelPeriodMicros(void);
uint16_t GetGearReceiverChannelPeriodMicros(void);
uint16_t GetAux1ReceiverChannelPeriodMicros(void);

#endif /* __RECEIVER_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
