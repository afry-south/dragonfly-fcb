/******************************************************************************
 * @file    receiver.c
 * @author  ÅF Dragonfly
 * @version v. 0.0.4
 * @date    2015-05-28
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          File contains functionality for reading signals from the RC receiver
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "receiver.h"
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
}Receiver_Pulse_States_TypeDef;

typedef struct
{
  uint16_t RisingCount;
  uint16_t FallingCounter;
  uint16_t PreviousRisingCount;
  uint16_t PreviousRisingCountTimerPeriodCount;
  uint16_t PulseTimerCount;
  uint32_t PeriodCount;
}Receiver_IC_Values_TypeDef;

typedef struct
{
  uint16_t ChannelMaxCount;
  uint16_t ChannelMinCount;
}PWM_IC_CalibrationValues_TypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile TIM_HandleTypeDef PrimaryReceiverTimHandle;
volatile TIM_HandleTypeDef AuxReceiverTimHandle;
static volatile TIM_IC_InitTypeDef PrimaryReceiverICConfig;
static volatile TIM_IC_InitTypeDef AuxReceiverICConfig;

/* Struct contaning HIGH/LOW state for each input channel pulse */
static Receiver_Pulse_States_TypeDef ReceiverPulseStates;

/* Structs for each channel's count values */
static volatile Receiver_IC_Values_TypeDef ThrottleICValues;
static volatile Receiver_IC_Values_TypeDef AileronICValues;
static volatile Receiver_IC_Values_TypeDef ElevatorICValues;
static volatile Receiver_IC_Values_TypeDef RudderICValues;
static volatile Receiver_IC_Values_TypeDef GearICValues;
static volatile Receiver_IC_Values_TypeDef Aux1ICValues;

static PWM_IC_CalibrationValues_TypeDef ThrottleCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef AileronCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef ElevatorCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef RudderCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef GearCalibrationValues;
static PWM_IC_CalibrationValues_TypeDef Aux1CalibrationValues;

static volatile uint16_t PrimaryReceiverTimerPeriodCount;
static volatile uint16_t AuxReceiverTimerPeriodCount;

/* Private function prototypes -----------------------------------------------*/
static void InitReceiverCalibrationValues(void);
static ReceiverErrorStatus GetReceiverCalibrationValuesFromFlash(void);
static void SetDefaultReceiverCalibrationValues(void);
static ReceiverErrorStatus PrimaryReceiverInput_Config(void);
static ReceiverErrorStatus AuxReceiverInput_Config(void);

static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState,
    Receiver_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel, const volatile uint16_t* ReceiverTimerPeriodCount);
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void);
static ReceiverErrorStatus UpdateReceiverAileronChannel(void);
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void);
static ReceiverErrorStatus UpdateReceiverRudderChannel(void);
static ReceiverErrorStatus UpdateReceiverGearChannel(void);
static ReceiverErrorStatus UpdateReceiverAux1Channel(void);

static int16_t GetSignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, PWM_IC_CalibrationValues_TypeDef* ChannelCalibrationValues);
static uint16_t GetUnsignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, PWM_IC_CalibrationValues_TypeDef* ChannelCalibrationValues);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input PWM signals
 * @param  None
 * @retval None
 */
ReceiverErrorStatus ReceiverInput_Config(void)
{
  InitReceiverCalibrationValues();

  if(!PrimaryReceiverInput_Config())
    return RECEIVER_ERROR;

  if(!AuxReceiverInput_Config())
    return RECEIVER_ERROR;

  return RECEIVER_OK;
}

/*
 * @brief  Returns a normalized receiver throttle value as an unsigned integer.
 * @param  None
 * @retval throttle value [0, 65535]
 */
uint16_t GetThrottleReceiverChannel(void)
{
  return GetUnsignedReceiverChannel(&ThrottleICValues, &ThrottleCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver aileron value as a signed integer.
 * @param  None
 * @retval aileron value [-32768, 32767]
 */
int16_t GetAileronReceiverChannel(void)
{
  return GetSignedReceiverChannel(&AileronICValues, &AileronCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver elevator value as a signed integer.
 * @param  None
 * @retval elevator value [-32768, 32767]
 */
int16_t GetElevatorReceiverChannel(void)
{
  return GetSignedReceiverChannel(&ElevatorICValues, &ElevatorCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver rudder value as a signed integer.
 * @param  None
 * @retval rudder value [-32768, 32767]
 */
int16_t GetRudderReceiverChannel(void)
{
  return GetSignedReceiverChannel(&RudderICValues, &RudderCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver gear value as a signed integer.
 * @param  None
 * @retval gear value [-32768, 32767]
 */
int16_t GetGearReceiverChannel(void)
{
  return GetSignedReceiverChannel(&GearICValues, &GearCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver aux1 value as a signed integer.
 * @param  None
 * @retval aux1 value [-32768, 32767]
 */
int16_t GetAux1ReceiverChannel(void)
{
  return GetSignedReceiverChannel(&Aux1ICValues, &Aux1CalibrationValues);
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
 * @brief       Checks if the RC transmission between transmitter and receiver is active.
 *              In case of transmitter is turned off or aircraft is out of transmission range
 *              this function will return false.
 * @param       None.
 * @retval      true if transmission is active, else false.
 */
ReceiverErrorStatus IsReceiverActive(void)
{
  // TODO: Check last time for pulse. The throttle channel typically keeps transmitting a pulse in case of
  // connection failure, but the other channels (all of them?) go silent with no pulses
  // Perhaps count amount of reset interrupts without a pulse flank being detected

  // Use IS_RECEIVER_INACTIVE_PERIODS_COUNT
  uint32_t periodsSinceLastAileronPulse;

  if(PrimaryReceiverTimerPeriodCount >= PrimaryAileronICValues.PreviousRisingCountTimerPeriodCount)
    periodsSinceLastAileronPulse = PrimaryReceiverTimerPeriodCount - PrimaryAileronICValues.PreviousRisingCountTimerPeriodCount;
  else
    periodsSinceLastAileronPulse = PrimaryReceiverTimerPeriodCount + UINT16_MAX - PrimaryAileronICValues.PreviousRisingCountTimerPeriodCount;

  if(AileronICValues.PreviousRisingCountTimerPeriodCount)

  return RECEIVER_OK;
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

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==PRIMARY_RECEIVER_TIM)
    {
      PrimaryReceiverTimerPeriodCount++;
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      AuxReceiverTimerPeriodCount++;
    }
}

/* Private functions ---------------------------------------------------------*/

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

/*
 * @brief  Sets receiver calibration values to default
 * @param  None
 * @retval None
 */
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
 * @brief  Returns a normalized receiver channel value as a signed integer.
 * @param  None
 * @retval channel value [-32768, 32767]
 */
static int16_t GetSignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, PWM_IC_CalibrationValues_TypeDef* ChannelCalibrationValues)
{
  if(ChannelICValues->PulseTimerCount < ChannelCalibrationValues->ChannelMinCount)
    return INT16_MIN;
  else if(ChannelICValues->PulseTimerCount > ChannelCalibrationValues->ChannelMaxCount)
    return INT16_MAX;
  else if(ChannelCalibrationValues->ChannelMaxCount > ChannelCalibrationValues->ChannelMinCount)
    return INT16_MIN + (((uint32_t) (ChannelICValues->PulseTimerCount-ChannelCalibrationValues->ChannelMinCount)*UINT16_MAX)/(ChannelCalibrationValues->ChannelMaxCount-ChannelCalibrationValues->ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver channel value as an unsigned integer.
 * @param  None
 * @retval channel value [0, 65535]
 */
static uint16_t GetUnsignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, PWM_IC_CalibrationValues_TypeDef* ChannelCalibrationValues)
{
  if(ChannelICValues->PulseTimerCount < ChannelCalibrationValues->ChannelMinCount)
    return 0;
  else if(ChannelICValues->PulseTimerCount > ChannelCalibrationValues->ChannelMaxCount)
    return UINT16_MAX;
  else if(ChannelCalibrationValues->ChannelMaxCount > ChannelCalibrationValues->ChannelMinCount)
    return ((uint32_t) (ChannelICValues->PulseTimerCount-ChannelCalibrationValues->ChannelMinCount)*UINT16_MAX)/(ChannelCalibrationValues->ChannelMaxCount-ChannelCalibrationValues->ChannelMinCount);
  else
    return 0;
}
/*
 * @brief       Gets calibration values stored in flash after previously performed receiver calibration
 * @param       None.
 * @retval      true if valid calibration values has been loaded, else false.
 */
static ReceiverErrorStatus GetReceiverCalibrationValuesFromFlash(void)
{
  // TODO implement
  return RECEIVER_ERROR;
}

/*
 * @brief       Initializes reading from the receiver primary input channels, i.e.
 *              throttle aileron, elevator and rudder channels. The signals are
 *              encoded as PWM pulses of ~1-2 ms.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus PrimaryReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

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
      errorStatus = RECEIVER_ERROR;
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
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of elevator channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of rudder channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-4- Start the Time Base update interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&PrimaryReceiverTimHandle) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief       Initializes reading from the receiver aux input channels, i.e.
 *              Gear and Aux1. The signals are encoded as PWM pulses of ~1-2 ms.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus AuxReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

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
      errorStatus = RECEIVER_ERROR;
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
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  AuxReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-4- Start the Time Base update interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&AuxReceiverTimHandle) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief       Updates a receiver channel IC counts. The channel is specified by the function parameters.
 * @param       TimHandle: Reference to the TIM_HandleTypeDef struct used to read the channel's IC count
 * @param       TimIC: Reference to the TIM_IC_InitTypeDef struct used to configure the IC to count on rising/falling pulse flank
 * @param       channelInputState: The current channel pulse input state (PULSE_LOW or PULSE_HIGH)
 * @param       ChannelICValues: Reference to the channels IC value struct
 * @param       receiverChannel: TIM Channel
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState,
    Receiver_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel, const volatile uint16_t* ReceiverTimerPeriodCount)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

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

      /* Calculate the period between pulses by computing the difference between current and previous rising edge timer counts.
       * Since the counter typically resets more often than a new pulse is triggered for the receiver, one must also use the
       * number of timer period resets since the last pulse. */
      if((*ReceiverTimerPeriodCount) > ChannelICValues->PreviousRisingCountTimerPeriodCount)
        {
          ChannelICValues->PeriodCount = ChannelICValues->RisingCount + UINT16_MAX - ChannelICValues->PreviousRisingCount
              + UINT16_MAX*((*ReceiverTimerPeriodCount)-ChannelICValues->PreviousRisingCountTimerPeriodCount-1);
        }
      else
        {
          if(ChannelICValues->RisingCount > ChannelICValues->PreviousRisingCount)
            ChannelICValues->PeriodCount = ChannelICValues->RisingCount - ChannelICValues->PreviousRisingCount;
          else
            ChannelICValues->PeriodCount = ChannelICValues->RisingCount + UINT16_MAX - ChannelICValues->PreviousRisingCount;
        }

      ChannelICValues->PreviousRisingCountTimerPeriodCount = (*ReceiverTimerPeriodCount);
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
      else
        errorStatus = RECEIVER_ERROR;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(TimHandle, TimIC, receiverChannel) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief	Handles the receiver input pulse measurements from the throttle channel
 * 		and updates pulse and frequency values.
 * @param 	None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ThrottleInputState,
      &ThrottleICValues, PRIMARY_RECEIVER_THROTTLE_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief       Handles the receiver input pulse measurements from the aileron channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverAileronChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.AileronInputState,
      &AileronICValues, PRIMARY_RECEIVER_AILERON_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief       Handles the receiver input pulse measurements from the elevator channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ElevatorInputState,
      &ElevatorICValues, PRIMARY_RECEIVER_ELEVATOR_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief       Handles the receiver input pulse measurements from the rudder channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverRudderChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.RudderInputState,
      &RudderICValues, PRIMARY_RECEIVER_RUDDER_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief       Handles the receiver input pulse measurements from the gear channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverGearChannel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.GearInputState,
      &GearICValues, AUX_RECEIVER_GEAR_CHANNEL, &AuxReceiverTimerPeriodCount);
}

/*
 * @brief       Handles the receiver input pulse measurements from the aux1 channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static ReceiverErrorStatus UpdateReceiverAux1Channel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.Aux1InputState,
      &Aux1ICValues, AUX_RECEIVER_AUX1_CHANNEL, &AuxReceiverTimerPeriodCount);
}
