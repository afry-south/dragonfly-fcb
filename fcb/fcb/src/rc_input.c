/******************************************************************************
 * @file    rc_input.c
 * @author  ÅF Dragonfly
 * @version v. 0.0.4
 * @date    2015-05-28
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          File contains functionality for reading signals from the RC receiver
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "rc_input.h"
#include "main.h"
#include "flight_control.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  Pulse_State ThrottleInputState;
  Pulse_State AileronInputState;
  Pulse_State ElevatorInputState;
  Pulse_State RudderInputState;
  Pulse_State GearInputState;
  Pulse_State Aux1InputState;
}PWM_Input_Channel_States_TypeDef;

typedef struct
{
  uint16_t RisingCount;
  uint16_t FallingCounter;
  uint16_t PreviousRisingCount;
  uint16_t PulseTimerCount;
  uint16_t PeriodCount;
}PWM_IC_Values_TypeDef;

typedef struct
{
  uint16_t ChannelMaxCount;
  uint16_t ChannelMinCount;
}PWM_IC_CalibrationValues_TypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef PrimaryReceiverTimHandle;
TIM_HandleTypeDef AuxReceiverTimHandle;
static TIM_IC_InitTypeDef PrimaryReceiverICConfig;
static TIM_IC_InitTypeDef AuxReceiverICConfig;

/* Struct contaning HIGH/LOW state for each input channel pulse */
static PWM_Input_Channel_States_TypeDef ReceiverInputStates;

/* Structs for each channel's count values */
static PWM_IC_Values_TypeDef ThrottleICValues;
static PWM_IC_Values_TypeDef AileronICValues;
static PWM_IC_Values_TypeDef ElevatorICValues;
static PWM_IC_Values_TypeDef RudderICValues;
static PWM_IC_Values_TypeDef GearICValues;
static PWM_IC_Values_TypeDef Aux1ICValues;

static PWM_IC_CalibrationValues_TypeDef ThrottleCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef AileronCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef ElevatorCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef RudderCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef GearCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef Aux1CalibrationValues;

/* Private function prototypes -----------------------------------------------*/
static void InitReceiverCalibrationValues(void);
static bool GetReceiverCalibrationValuesFromFlash(void);
static void SetDefaultReceiverCalibrationValues(void);
static void PrimaryReceiverInput_Config(void);
static void AuxReceiverInput_Config(void);

static void UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState, PWM_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel);
static void UpdateReceiverThrottleChannel(void);
static void UpdateReceiverAileronChannel(void);
static void UpdateReceiverElevatorChannel(void);
static void UpdateReceiverRudderChannel(void);
static void UpdateReceiverGearChannel(void);
static void UpdateReceiverAux1Channel(void);

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input PWM signals
 * @param  None
 * @retval None
 */
void ReceiverInput_Config(void)
{
  InitReceiverCalibrationValues();
  PrimaryReceiverInput_Config();
  AuxReceiverInput_Config();
}

/*
 * @brief  Returns a normalized receiver throttle value as an unsigned integer.
 * @param  None
 * @retval throttle value [0, 65535]
 */
uint16_t GetThrottleReceiverChannel(void)
{
  if(ThrottleICValues.PulseTimerCount < ThrottleCalibrationValues.ChannelMinCount)
    return 0;
  else if(ThrottleICValues.PulseTimerCount > ThrottleCalibrationValues.ChannelMaxCount)
    return UINT16_MAX;
  else
    return ((uint32_t) (ThrottleICValues.PulseTimerCount-ThrottleCalibrationValues.ChannelMinCount)*UINT16_MAX)/(ThrottleCalibrationValues.ChannelMaxCount-ThrottleCalibrationValues.ChannelMinCount);
}

/*
 * @brief  Returns a normalized receiver aileron value as a signed integer.
 * @param  None
 * @retval aileron value [-32768, 32767]
 */
int16_t GetAileronReceiverChannel(void)
{
  if(AileronICValues.PulseTimerCount < AileronCalibrationValues.ChannelMinCount)
    return INT16_MIN;
  else if(AileronICValues.PulseTimerCount > AileronCalibrationValues.ChannelMaxCount)
    return INT16_MAX;
  else if(AileronCalibrationValues.ChannelMaxCount > AileronCalibrationValues.ChannelMinCount)
    return INT16_MIN + (((uint32_t) (AileronICValues.PulseTimerCount-AileronCalibrationValues.ChannelMinCount)*UINT16_MAX)/(AileronCalibrationValues.ChannelMaxCount-AileronCalibrationValues.ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver elevator value as a signed integer.
 * @param  None
 * @retval elevator value [-32768, 32767]
 */
int16_t GetElevatorReceiverChannel(void)
{
  if(ElevatorICValues.PulseTimerCount < ElevatorCalibrationValues.ChannelMinCount)
    return INT16_MIN;
  else if(ElevatorICValues.PulseTimerCount > ElevatorCalibrationValues.ChannelMaxCount)
    return INT16_MAX;
  else if(ElevatorCalibrationValues.ChannelMaxCount > ElevatorCalibrationValues.ChannelMinCount)
    return INT16_MIN + (((uint32_t) (ElevatorICValues.PulseTimerCount-ElevatorCalibrationValues.ChannelMinCount)*UINT16_MAX)/(ElevatorCalibrationValues.ChannelMaxCount-ElevatorCalibrationValues.ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver rudder value as a signed integer.
 * @param  None
 * @retval rudder value [-32768, 32767]
 */
int16_t GetRudderReceiverChannel(void)
{
  if(RudderICValues.PulseTimerCount < RudderCalibrationValues.ChannelMinCount)
    return INT16_MIN;
  else if(RudderICValues.PulseTimerCount > RudderCalibrationValues.ChannelMaxCount)
    return INT16_MAX;
  else if(RudderCalibrationValues.ChannelMaxCount > RudderCalibrationValues.ChannelMinCount)
    return INT16_MIN + (((uint32_t) (RudderICValues.PulseTimerCount-RudderCalibrationValues.ChannelMinCount)*UINT16_MAX)/(RudderCalibrationValues.ChannelMaxCount-RudderCalibrationValues.ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver gear value as a signed integer.
 * @param  None
 * @retval gear value [-32768, 32767]
 */
int16_t GetGearReceiverChannel(void)
{
  if(GearICValues.PulseTimerCount < GearCalibrationValues.ChannelMinCount)
    return INT16_MIN;
  else if(GearICValues.PulseTimerCount > GearCalibrationValues.ChannelMaxCount)
    return INT16_MAX;
  else if(GearCalibrationValues.ChannelMaxCount > GearCalibrationValues.ChannelMinCount)
    return INT16_MIN + (((uint32_t) (GearICValues.PulseTimerCount-GearCalibrationValues.ChannelMinCount)*UINT16_MAX)/(GearCalibrationValues.ChannelMaxCount-GearCalibrationValues.ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver aux1 value as a signed integer.
 * @param  None
 * @retval aux1 value [-32768, 32767]
 */
int16_t GetAux1ReceiverChannel(void)
{
  if(Aux1ICValues.PulseTimerCount < Aux1CalibrationValues.ChannelMinCount)
    return INT16_MIN;
  else if(Aux1ICValues.PulseTimerCount > Aux1CalibrationValues.ChannelMaxCount)
    return INT16_MAX;
  else if(Aux1CalibrationValues.ChannelMaxCount > Aux1CalibrationValues.ChannelMinCount)
    return INT16_MIN + (((uint32_t) (Aux1ICValues.PulseTimerCount-Aux1CalibrationValues.ChannelMinCount)*UINT16_MAX)/(Aux1CalibrationValues.ChannelMaxCount-Aux1CalibrationValues.ChannelMinCount));
  else
    return 0;
}

/*
 * @brief       Identifies the max/min input levels of the receiver channels.
 * @param       None.
 * @retval      None.
 */
void CalibrateReceiver(void)
{
  // TODO
}

/*
 * @brief  Initializes receiver input calibration values (max and min timer IC counts)
 * @param  None
 * @retval None
 */
static void InitReceiverCalibrationValues(void)
{
  if(!GetReceiverCalibrationValuesFromFlash())
    SetDefaultReceiverCalibrationValues();
}

static void SetDefaultReceiverCalibrationValues(void)
{
  ThrottleCalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  ThrottleCalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
  AileronCalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  AileronCalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
  ElevatorCalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  ElevatorCalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
  RudderCalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  RudderCalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
  GearCalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  GearCalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
  Aux1CalibrationValues.ChannelMaxCount = RECEIVER_DEFAULT_MAX_COUNT;
  Aux1CalibrationValues.ChannelMinCount = RECEIVER_DEFAULT_MIN_COUNT;
}

/*
 * @brief       Gets calibration values stored in flash after previously performed receiver calibration
 * @param       None.
 * @retval      true if valid calibration values has been loaded, else false.
 */
static bool GetReceiverCalibrationValuesFromFlash(void)
{
  // TODO implement
  return false;
}

/*
 * @brief       Initializes reading from the receiver primary input channels, i.e.
 *              throttle aileron, elevator and rudder channels. The signals are
 *              encoded as PWM pulses of ~1-2 ms.
 * @param       None.
 * @retval      None.
 */
static void PrimaryReceiverInput_Config(void)
{
  /*##-1- Configure the Primary Receiver TIM peripheral ######################*/
  /* Set TIM instance */
  PrimaryReceiverTimHandle.Instance = PRIMARY_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver PWM input period is ~22 ms) */
  PrimaryReceiverTimHandle.Init.Period = UINT16_MAX;
  PrimaryReceiverTimHandle.Init.Prescaler = SystemCoreClock/RECEIVER_TIM_COUNTER_CLOCK - 1;
  PrimaryReceiverTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  PrimaryReceiverTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&PrimaryReceiverTimHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

  /*##-2- Configure the Input Capture channels ###############################*/
  /* Common configuration */
  PrimaryReceiverICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  PrimaryReceiverICConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of elevator channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of rudder channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
}

/*
 * @brief       Initializes reading from the receiver aux input channels, i.e.
 *              Gear and Aux1. The signals are encoded as PWM pulses of ~1-2 ms.
 * @param       None.
 * @retval      None.
 */
static void AuxReceiverInput_Config(void)
{
  /*##-1- Configure the Aux Receiver TIM peripheral ##########################*/
  /* Set TIM instance */
  AuxReceiverTimHandle.Instance = AUX_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver PWM input period is ~22 ms) */
  AuxReceiverTimHandle.Init.Period = UINT16_MAX;
  AuxReceiverTimHandle.Init.Prescaler = SystemCoreClock/RECEIVER_TIM_COUNTER_CLOCK - 1;
  AuxReceiverTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  AuxReceiverTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&AuxReceiverTimHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }

  /*##-2- Configure the Input Capture channels ###############################*/
  /* Common configuration */
  AuxReceiverICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  AuxReceiverICConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  AuxReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  AuxReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }
}

/*
 * @brief       Updates a receiver channel IC counts. The channel is specified by the function parameters.
 * @param       TimHandle: Reference to the TIM_HandleTypeDef struct used to read the channel's IC count
 * @param       TimIC: Reference to the TIM_IC_InitTypeDef struct used to configure the IC to count on rising/falling pulse flank
 * @param       channelInputState: The current channel pulse input state (PULSE_LOW or PULSE_HIGH)
 * @param
 * @param
 * @retval      None.
 */
static void UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState, PWM_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel)
{
  /* Detected rising PWM edge */
  if ((*channelInputState) == PULSE_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(TimHandle, receiverChannel);
      (*channelInputState) = PULSE_HIGH; // Set input state to high

      TimIC->ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      ChannelICValues->PreviousRisingCount = ChannelICValues->RisingCount;
      ChannelICValues->RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(ChannelICValues->RisingCount > ChannelICValues->PreviousRisingCount)
        ChannelICValues->PeriodCount = ChannelICValues->RisingCount - ChannelICValues->PreviousRisingCount;
      else
        ChannelICValues->PeriodCount = ChannelICValues->RisingCount + UINT16_MAX - ChannelICValues->PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if ((*channelInputState) == PULSE_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(TimHandle, receiverChannel);
      (*channelInputState) = PULSE_LOW; // Set input state to low

      TimIC->ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ChannelICValues->FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (ChannelICValues->FallingCounter > ChannelICValues->RisingCount)
        tempPulseTimerCount = ChannelICValues->FallingCounter - ChannelICValues->RisingCount;
      else
        tempPulseTimerCount = ChannelICValues->FallingCounter + UINT16_MAX - ChannelICValues->RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        ChannelICValues->PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(TimHandle, TimIC, receiverChannel) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief	Handles the receiver input pulse measurements from the throttle channel
 * 		and updates pulse and frequency values.
 * @param 	None.
 * @retval      None.
 */
static void UpdateReceiverThrottleChannel(void)
{
  UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverInputStates.ThrottleInputState, &ThrottleICValues, PRIMARY_RECEIVER_THROTTLE_CHANNEL);
}

/*
 * @brief       Handles the receiver input pulse measurements from the aileron channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateReceiverAileronChannel(void)
{
  UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverInputStates.AileronInputState, &AileronICValues, PRIMARY_RECEIVER_AILERON_CHANNEL);
}

/*
 * @brief       Handles the receiver input pulse measurements from the elevator channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateReceiverElevatorChannel(void)
{
  UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverInputStates.ElevatorInputState, &ElevatorICValues, PRIMARY_RECEIVER_ELEVATOR_CHANNEL);
}

/*
 * @brief       Handles the receiver input pulse measurements from the rudder channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateReceiverRudderChannel(void)
{
  UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverInputStates.RudderInputState, &RudderICValues, PRIMARY_RECEIVER_RUDDER_CHANNEL);
}

/*
 * @brief       Handles the receiver input pulse measurements from the gear channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateReceiverGearChannel(void)
{
  UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverInputStates.GearInputState, &GearICValues, AUX_RECEIVER_GEAR_CHANNEL);
}

/*
 * @brief       Handles the receiver input pulse measurements from the aux1 channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateReceiverAux1Channel(void)
{
  UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverInputStates.Aux1InputState, &Aux1ICValues, AUX_RECEIVER_AUX1_CHANNEL);
}

/*
 * @brief       Checks if the RC transmission between transmitter and receiver is active.
 *              In case of transmitter is turned off or aircraft is out of transmission range
 *              this function will return false.
 * @param	None.
 * @retval	true if transmission is active, else false.
 */
bool IsReceiverActive()
{
  // TODO: Check last time for pulse. The throttle channel typically keeps transmitting a pulse in case of
  // connection failure, but the other channels (all of them?) go silent with no pulses
  // Perhaps count amount of reset interrupts without a pulse flank being detected
  return true;
}

/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==PRIMARY_RECEIVER_TIM)
    {
      if(htim->Channel == PRIMARY_RECEIVER_THROTTLE_ACTIVE_CHANNEL)
        UpdateReceiverThrottleChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_AILERON_ACTIVE_CHANNEL)
        UpdateReceiverAileronChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL)
        UpdateReceiverElevatorChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL)
        UpdateReceiverRudderChannel();
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      if(htim->Channel == AUX_RECEIVER_GEAR_ACTIVE_CHANNEL)
        UpdateReceiverGearChannel();
      else if(htim->Channel == AUX_RECEIVER_AUX1_ACTIVE_CHANNEL)
        UpdateReceiverAux1Channel();
    }
}
