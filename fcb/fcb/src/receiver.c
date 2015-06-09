/******************************************************************************
 * @file    receiver.c
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-05-28
 * @brief   File contains functionality for signal readubg from the Dragonfly
 *          RC receiver. The receiver model is Spektrum AR610, which uses DSMX
 *          frequency modulation technology. It outputs 6 channels: throttle,
 *          aileron, elevator, rudder, gear and aux1.
 *
 *          Since a timer IC on STM32 only has up to 4 channels, two timers are
 *          needed to collect the receiver pulses. The pulses sent from the
 *          receiver are typically ~1-2 ms width with a period of ~22 ms.
 *
 *          It is the pulse width that encodes the received transmitter stick
 *          action and is therefore the most relevant quantity when reading the
 *          signals from the receiver. As example, the aileron channel outputs
 *          ~1 ms when the transmitter control stick is held in one direction and
 *          ~2 ms when it is held in the opposite direction.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "receiver.h"
#include "main.h"

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
  uint32_t PeriodCount;
  uint16_t RisingCount;
  uint16_t FallingCounter;
  uint16_t PreviousRisingCount;
  uint16_t PreviousRisingCountTimerPeriodCount;
  uint16_t PulseTimerCount;
  ReceiverErrorStatus IsActive;
}Receiver_IC_Values_TypeDef;

typedef struct
{
  uint16_t ChannelMaxCount;
  uint16_t ChannelMinCount;
}Receiver_IC_PulseCalibrationValues_TypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer time base handlers for each timer */
TIM_HandleTypeDef PrimaryReceiverTimHandle;
TIM_HandleTypeDef AuxReceiverTimHandle;

/* Timer IC init declarations for each timer */
static TIM_IC_InitTypeDef PrimaryReceiverICConfig;
static TIM_IC_InitTypeDef AuxReceiverICConfig;

/* Struct contaning HIGH/LOW state for each input channel pulse */
static Receiver_Pulse_States_TypeDef ReceiverPulseStates;

/* Structs for each channel's timer count values */
static Receiver_IC_Values_TypeDef ThrottleICValues;
static Receiver_IC_Values_TypeDef AileronICValues;
static Receiver_IC_Values_TypeDef ElevatorICValues;
static Receiver_IC_Values_TypeDef RudderICValues;
static Receiver_IC_Values_TypeDef GearICValues;
static Receiver_IC_Values_TypeDef Aux1ICValues;

/* Structs for each channel's calibration values */
static Receiver_IC_PulseCalibrationValues_TypeDef ThrottleCalibrationValues;
static Receiver_IC_PulseCalibrationValues_TypeDef AileronCalibrationValues;
static Receiver_IC_PulseCalibrationValues_TypeDef ElevatorCalibrationValues;
static Receiver_IC_PulseCalibrationValues_TypeDef RudderCalibrationValues;
static Receiver_IC_PulseCalibrationValues_TypeDef GearCalibrationValues;
static Receiver_IC_PulseCalibrationValues_TypeDef Aux1CalibrationValues;

/* Timer reset counters for each timer */
static volatile uint16_t PrimaryReceiverTimerPeriodCount;
static volatile uint16_t AuxReceiverTimerPeriodCount;

/* Private function prototypes -----------------------------------------------*/
static ReceiverErrorStatus InitReceiverCalibrationValues(void);
static ReceiverErrorStatus GetReceiverCalibrationValuesFromFlash(void);
static void SetDefaultReceiverCalibrationValues(void);
static ReceiverErrorStatus PrimaryReceiverInput_Config(void);
static ReceiverErrorStatus AuxReceiverInput_Config(void);

static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState,
    Receiver_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel, volatile const uint16_t* ReceiverTimerPeriodCount);
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void);
static ReceiverErrorStatus UpdateReceiverAileronChannel(void);
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void);
static ReceiverErrorStatus UpdateReceiverRudderChannel(void);
static ReceiverErrorStatus UpdateReceiverGearChannel(void);
static ReceiverErrorStatus UpdateReceiverAux1Channel(void);

static int16_t GetSignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, Receiver_IC_PulseCalibrationValues_TypeDef* ChannelCalibrationValues);
static uint16_t GetUnsignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, Receiver_IC_PulseCalibrationValues_TypeDef* ChannelCalibrationValues);
static uint16_t GetReceiverChannelPulseMicros(Receiver_IC_Values_TypeDef* ChannelICValues);
static uint16_t GetReceiverChannelPeriodMicros(Receiver_IC_Values_TypeDef* ChannelICValues);

static ReceiverErrorStatus IsReceiverChannelActive(Receiver_IC_Values_TypeDef* ChannelICValues, const uint16_t ReceiverTimerPeriodCount);
static ReceiverErrorStatus IsReceiverPulseValid(uint16_t pulseTimerCount, uint16_t currentPeriodCount, uint16_t previousPeriodCount);
static ReceiverErrorStatus IsReceiverPeriodValid(uint32_t periodTimerCount);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input signals
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
 * @brief  Returns a normalized receiver throttle pulse value as an unsigned integer.
 * @param  None
 * @retval throttle value [0, 65535]
 */
uint16_t GetThrottleReceiverChannel(void)
{
  return GetUnsignedReceiverChannel(&ThrottleICValues, &ThrottleCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver aileron pulse value as a signed integer.
 * @param  None
 * @retval aileron value [-32768, 32767]
 */
int16_t GetAileronReceiverChannel(void)
{
  return GetSignedReceiverChannel(&AileronICValues, &AileronCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver elevator pulse value as a signed integer.
 * @param  None
 * @retval elevator value [-32768, 32767]
 */
int16_t GetElevatorReceiverChannel(void)
{
  return GetSignedReceiverChannel(&ElevatorICValues, &ElevatorCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver rudder pulse value as a signed integer.
 * @param  None
 * @retval rudder value [-32768, 32767]
 */
int16_t GetRudderReceiverChannel(void)
{
  return GetSignedReceiverChannel(&RudderICValues, &RudderCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver gear pulse value as a signed integer.
 * @param  None
 * @retval gear value [-32768, 32767]
 */
int16_t GetGearReceiverChannel(void)
{
  return GetSignedReceiverChannel(&GearICValues, &GearCalibrationValues);
}

/*
 * @brief  Returns a normalized receiver aux1 pulse value as a signed integer.
 * @param  None
 * @retval aux1 value [-32768, 32767]
 */
int16_t GetAux1ReceiverChannel(void)
{
  return GetSignedReceiverChannel(&Aux1ICValues, &Aux1CalibrationValues);
}

/*
 * @brief  Returns the last throttle pulse value in microseconds
 * @param  None
 * @retval throttle pulse value in microseconds
 */
uint16_t GetThrottleReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&ThrottleICValues);
}

/*
 * @brief  Returns the last aileron pulse value in microseconds
 * @param  None
 * @retval aileron pulse value in microseconds
 */
uint16_t GetAileronReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&AileronICValues);
}

/*
 * @brief  Returns the last elevator pulse value in microseconds
 * @param  None
 * @retval elevator pulse value in microseconds
 */
uint16_t GetElevatorReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&ElevatorICValues);
}

/*
 * @brief  Returns the last rudder pulse value in microseconds
 * @param  None
 * @retval rudder pulse value in microseconds
 */
uint16_t GetRudderReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&RudderICValues);
}

/*
 * @brief  Returns the last gear pulse value in microseconds
 * @param  None
 * @retval gear pulse value in microseconds
 */
uint16_t GetGearReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&GearICValues);
}

/*
 * @brief  Returns the last aux1 pulse value in microseconds
 * @param  None
 * @retval aux1 pulse value in microseconds
 */
uint16_t GetAux1ReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&Aux1ICValues);
}

/*
 * @brief  Returns the last throttle period value in microseconds
 * @param  None
 * @retval throttle period value in microseconds
 */
uint16_t GetThrottleReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&ThrottleICValues);
}

/*
 * @brief  Returns the last aileron period value in microseconds
 * @param  None
 * @retval aileron period value in microseconds
 */
uint16_t GetAileronReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&AileronICValues);
}

/*
 * @brief  Returns the last elevator period value in microseconds
 * @param  None
 * @retval elevator period value in microseconds
 */
uint16_t GetElevatorReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&ElevatorICValues);
}

/*
 * @brief  Returns the last rudder period value in microseconds
 * @param  None
 * @retval rudder period value in microseconds
 */
uint16_t GetRudderReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&RudderICValues);
}

/*
 * @brief  Returns the last gear period value in microseconds
 * @param  None
 * @retval gear period value in microseconds
 */
uint16_t GetGearReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&GearICValues);
}

/*
 * @brief  Returns the last aux1 period value in microseconds
 * @param  None
 * @retval aux1 period value in microseconds
 */
uint16_t GetAux1ReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&Aux1ICValues);
}


/*
 * @brief    Identifies the max/min input levels of the receiver channels.
 * @param    None.
 * @retval   None.
 */
ReceiverErrorStatus CalibrateReceiver(void)
{
  // TODO
  // CalibrateReceiverChannel(&ThrottleCalibrationValues);
  // CalibrateReceiverChannel(&AileronCalibrationValues);
  // CalibrateReceiverChannel(&ElevatorCalibrationValues);
  // CalibrateReceiverChannel(&RudderCalibrationValues);
  // CalibrateReceiverChannel(&GearCalibrationValues);
  // CalibrateReceiverChannel(&Aux1CalibrationValues);

  return RECEIVER_ERROR;
}

/*
 * @brief    Checks if the RC transmission between transmitter and receiver is active.
 * @param    None.
 * @retval   RECEIVER_OK if transmission is active, else RECEIVER_ERROR.
 */
ReceiverErrorStatus IsReceiverActive(void)
{
  ReceiverErrorStatus aileronChannelActive;
  ReceiverErrorStatus elevatorChannelActive;
  ReceiverErrorStatus rudderChannelActive;

  /* When transmission stops, the throttle channel on the Spektrum AR610 receiver keeps sending pulses on its
   * channel based on its last received throttle command. But the other channels go silents, so they can be
   * used to check if the transmission is down. */
  aileronChannelActive = IsReceiverChannelActive(&AileronICValues, PrimaryReceiverTimerPeriodCount);
  elevatorChannelActive = IsReceiverChannelActive(&ElevatorICValues, PrimaryReceiverTimerPeriodCount);
  rudderChannelActive = IsReceiverChannelActive(&RudderICValues, PrimaryReceiverTimerPeriodCount);

  return (aileronChannelActive && elevatorChannelActive && rudderChannelActive);
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
 * @retval RECEIVER_OK if calibration values loaded, RECEIVER_ERROR if default values used
 */
static ReceiverErrorStatus InitReceiverCalibrationValues(void)
{
  if(!GetReceiverCalibrationValuesFromFlash())
    {
      SetDefaultReceiverCalibrationValues();
      return RECEIVER_ERROR;
    }
  return RECEIVER_OK;
}

/*
 * @brief  Sets receiver calibration values to default
 * @param  None
 * @retval None
 */
static void SetDefaultReceiverCalibrationValues(void)
{
  ThrottleCalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  ThrottleCalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  AileronCalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  AileronCalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  ElevatorCalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  ElevatorCalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  RudderCalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  RudderCalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  GearCalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  GearCalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  Aux1CalibrationValues.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  Aux1CalibrationValues.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;
}

/*
 * @brief  Returns a normalized receiver channel value as a signed integer.
 * @param  ChannelICValues : Reference to channel's IC values struct
 * @param  ChannelCalibrationValues : Reference to channel's calibration values struct
 * @retval channel value [-32768, 32767]
 */
static int16_t GetSignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, Receiver_IC_PulseCalibrationValues_TypeDef* ChannelCalibrationValues)
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
 * @param  ChannelICValues : Reference to channel's IC values struct
 * @param  ChannelCalibrationValues : Reference to channel's calibration values struct
 * @retval channel value [0, 65535]
 */
static uint16_t GetUnsignedReceiverChannel(Receiver_IC_Values_TypeDef* ChannelICValues, Receiver_IC_PulseCalibrationValues_TypeDef* ChannelCalibrationValues)
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
 * @brief  Returns the receiver channel pulse count in microseconds
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @retval Channel pulse timer count in microseconds
 */
static uint16_t GetReceiverChannelPulseMicros(Receiver_IC_Values_TypeDef* ChannelICValues)
{
  return (uint16_t)(((uint32_t)ChannelICValues->PulseTimerCount * 1000000) / RECEIVER_TIM_COUNTER_CLOCK);
}

/*
 * @brief  Returns the receiver channel period count in microseconds
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @retval Channel period timer count in microseconds
 */
static uint16_t GetReceiverChannelPeriodMicros(Receiver_IC_Values_TypeDef* ChannelICValues)
{
  /* The period count will never be more than RECEIVER_MAX_VALID_PERIOD_COUNT, take care so that 32-bit overflow does not occur below */
  return (uint16_t)(((uint32_t)ChannelICValues->PeriodCount * 100000) / RECEIVER_TIM_COUNTER_CLOCK * 10);
}

/*
 * @brief  Gets calibration values stored in flash after previously performed receiver calibration
 * @param  None.
 * @retval true if valid calibration values has been loaded, else false.
 */
static ReceiverErrorStatus GetReceiverCalibrationValuesFromFlash(void)
{
  // TODO implement
  return RECEIVER_ERROR;
}

/*
 * @brief  Initializes reading from the receiver primary input channels, i.e. throttle aileron,
 *     elevator and rudder channels. The signals are encoded as pulses of ~1-2 ms.
 * @param  None.
 * @retval RECEIVER_OK if configured without errors, else RECEIVER_ERROR
 */
static ReceiverErrorStatus PrimaryReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /*##-1- Configure the Primary Receiver TIM peripheral ######################*/
  /* Set TIM instance */
  PrimaryReceiverTimHandle.Instance = PRIMARY_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver input period is ~22 ms) */
  PrimaryReceiverTimHandle.Init.Period = RECEIVER_COUNTER_PERIOD;
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
 * @brief    Initializes reading from the receiver aux input channels, i.e.
 *       Gear and Aux1. The signals are encoded as pulses of ~1-2 ms.
 * @param    None.
 * @retval   RECEIVER_OK if configured without errors, else RECEIVER_ERROR
 */
static ReceiverErrorStatus AuxReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /*##-1- Configure the Aux Receiver TIM peripheral ##########################*/
  /* Set TIM instance */
  AuxReceiverTimHandle.Instance = AUX_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver input period is ~22 ms) */
  AuxReceiverTimHandle.Init.Period = RECEIVER_COUNTER_PERIOD;
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
 * @brief  Updates a receiver channel IC counts. The channel is specified by the function parameters.
 * @param  TimHandle : Reference to the TIM_HandleTypeDef struct used to read the channel's IC count
 * @param  TimIC : Reference to the TIM_IC_InitTypeDef struct used to configure the IC to count on rising/falling pulse flank
 * @param  channelInputState : The current channel pulse input state (PULSE_LOW or PULSE_HIGH)
 * @param  ChannelICValues : Reference to the channels IC value struct
 * @param  receiverChannel : TIM Channel
 * @param  ReceiverTimerPeriodCount : Reference to the timer period count variable (primary or aux)
 * @retval None.
 */
static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState,
    Receiver_IC_Values_TypeDef* ChannelICValues, const uint32_t receiverChannel, volatile const uint16_t* ReceiverTimerPeriodCount)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /* Detected rising pulse edge */
  if ((*channelInputState) == PULSE_LOW)
    {
      uint32_t tempPeriodTimerCount;

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
        tempPeriodTimerCount = ChannelICValues->RisingCount + UINT16_MAX - ChannelICValues->PreviousRisingCount
        + UINT16_MAX*((*ReceiverTimerPeriodCount)-ChannelICValues->PreviousRisingCountTimerPeriodCount-1);
      else
        tempPeriodTimerCount = ChannelICValues->RisingCount - ChannelICValues->PreviousRisingCount; // Short period value, likely incorrect

      if(IsReceiverPeriodValid(tempPeriodTimerCount))
        ChannelICValues->PeriodCount = tempPeriodTimerCount;
      else
        errorStatus = RECEIVER_ERROR;

      ChannelICValues->PreviousRisingCountTimerPeriodCount = (*ReceiverTimerPeriodCount);
    }
  /* Detected falling pulse edge */
  else if ((*channelInputState) == PULSE_HIGH)
    {
      uint16_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(TimHandle, receiverChannel);
      (*channelInputState) = PULSE_LOW; // Set input state to low

      TimIC->ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ChannelICValues->FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      tempPulseTimerCount = ChannelICValues->FallingCounter - ChannelICValues->RisingCount;

      /* Sanity check of pulse count before updating it */
      if(IsReceiverPulseValid(tempPulseTimerCount, (*ReceiverTimerPeriodCount), ChannelICValues->PreviousRisingCountTimerPeriodCount))
        {
          ChannelICValues->PulseTimerCount = tempPulseTimerCount;
          ChannelICValues->IsActive = RECEIVER_OK; // Set channel to active
        }
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
 * @brief  Handles the receiver input pulse measurements from the throttle channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ThrottleInputState,
      &ThrottleICValues, PRIMARY_RECEIVER_THROTTLE_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief  Handles the receiver input pulse measurements from the aileron channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverAileronChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.AileronInputState,
      &AileronICValues, PRIMARY_RECEIVER_AILERON_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief  Handles the receiver input pulse measurements from the elevator channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ElevatorInputState,
      &ElevatorICValues, PRIMARY_RECEIVER_ELEVATOR_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief  Handles the receiver input pulse measurements from the rudder channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverRudderChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.RudderInputState,
      &RudderICValues, PRIMARY_RECEIVER_RUDDER_CHANNEL, &PrimaryReceiverTimerPeriodCount);
}

/*
 * @brief  Handles the receiver input pulse measurements from the gear channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverGearChannel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.GearInputState,
      &GearICValues, AUX_RECEIVER_GEAR_CHANNEL, &AuxReceiverTimerPeriodCount);
}

/*
 * @brief  Handles the receiver input pulse measurements from the aux1 channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverAux1Channel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.Aux1InputState,
      &Aux1ICValues, AUX_RECEIVER_AUX1_CHANNEL, &AuxReceiverTimerPeriodCount);
}

/*
 * @brief  Checks if the RC transmission between transmitter and receiver is active for a specified channel.
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @param  ReceiverTimerPeriodCount : The period count value for the receiver counter
 * @retval RECEIVER_OK if transmission is active, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverChannelActive(Receiver_IC_Values_TypeDef* ChannelICValues, const uint16_t ReceiverTimerPeriodCount)
{
  uint32_t periodsSinceLastChannelPulse;

  /* Check how many timer resets have been performed since the last rising pulse edge */
  periodsSinceLastChannelPulse = ReceiverTimerPeriodCount - ChannelICValues->PreviousRisingCountTimerPeriodCount;

  /* Set channel as inactive if too many periods have passed since last channel pulse update */
  if(periodsSinceLastChannelPulse > IS_RECEIVER_CHANNEL_INACTIVE_PERIODS_COUNT)
    ChannelICValues->IsActive = RECEIVER_ERROR;

  return ChannelICValues->IsActive;
}

/*
 * @brief  Checks if a receiver channel pulse count is within a valid range
 * @param  pulseTimerCount : pulse timer count value
 * @param  currentPeriodCount : The current period count
 * @param  previousPeriodCount : The previous period count
 * @retval RECEIVER_OK if receiver pulse count is valid, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverPulseValid(uint16_t pulseTimerCount, uint16_t currentPeriodCount, uint16_t previousPeriodCount)
{
  /* Pulse count considered valid if it is within defined bounds and doesn't strech over more than 2 timer periods */
  return (pulseTimerCount <= RECEIVER_MAX_VALID_IC_PULSE_COUNT && pulseTimerCount >= RECEIVER_MIN_VALID_IC_PULSE_COUNT
      && currentPeriodCount - previousPeriodCount <= 1);
}

/*
 * @brief  Checks if a receiver channel period count is within a valid range
 * @param  periodTimerCount : period timer count value
 * @retval RECEIVER_OK if receiver period count is valid, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverPeriodValid(uint32_t periodTimerCount)
{
  /* Period count considered valid if it is within defined bounds */
  return (periodTimerCount <= RECEIVER_MAX_VALID_PERIOD_COUNT && periodTimerCount >= RECEIVER_MIN_VALID_PERIOD_COUNT);
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
