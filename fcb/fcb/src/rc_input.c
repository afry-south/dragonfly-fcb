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
  PWM_State ThrottleInputState;
  PWM_State AileronInputState;
  PWM_State ElevatorInputState;
  PWM_State RudderInputState;
  PWM_State GearInputState;
  PWM_State Aux1InputState;
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
static void PrimaryReceiverInput_Config(void);
static void AuxReceiverInput_Config(void);
static void UpdateThrottleChannel(void);
static void UpdateAileronChannel(void);
static void UpdateElevatorChannel(void);
static void UpdateRudderChannel(void);
static void UpdateGearChannel(void);
static void UpdateAux1Channel(void);

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input PWM signals
 * @param  None
 * @retval None
 */
void ReceiverInput_Config(void)
{
  // TODO: Get these from persistent memory (last stored calibration)
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

  PrimaryReceiverInput_Config();
  AuxReceiverInput_Config();
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
  PrimaryReceiverTimHandle.Init.Period = 0xFFFF;
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
  AuxReceiverTimHandle.Init.Period = 0xFFFF;
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
 * @brief	Handles the RC input pulse measurements from the throttle channel
 * 			and updates pulse and frequency values.
 * @param 	None.
 * @retval      None.
 */
static void UpdateThrottleChannel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.ThrottleInputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL);
      ReceiverInputStates.ThrottleInputState = PWM_HIGH; // Set input state to high

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      ThrottleICValues.PreviousRisingCount = ThrottleICValues.RisingCount;
      ThrottleICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(ThrottleICValues.RisingCount > ThrottleICValues.PreviousRisingCount)
        ThrottleICValues.PeriodCount = ThrottleICValues.RisingCount - ThrottleICValues.PreviousRisingCount;
      else
        ThrottleICValues.PeriodCount = ThrottleICValues.RisingCount + 0xFFFF - ThrottleICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.ThrottleInputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL);
      ReceiverInputStates.ThrottleInputState = PWM_LOW; // Set input state to low

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ThrottleICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (ThrottleICValues.FallingCounter > ThrottleICValues.RisingCount)
        tempPulseTimerCount = ThrottleICValues.FallingCounter - ThrottleICValues.RisingCount;
      else
        tempPulseTimerCount = ThrottleICValues.FallingCounter + 0xFFFF - ThrottleICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        ThrottleICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief       Handles the RC input pulse measurements from the aileron channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateAileronChannel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.AileronInputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_AILERON_CHANNEL);
      ReceiverInputStates.AileronInputState = PWM_HIGH; // Set input state to high

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      AileronICValues.PreviousRisingCount = AileronICValues.RisingCount;
      AileronICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(AileronICValues.RisingCount > AileronICValues.PreviousRisingCount)
        AileronICValues.PeriodCount = AileronICValues.RisingCount - AileronICValues.PreviousRisingCount;
      else
        AileronICValues.PeriodCount = AileronICValues.RisingCount + 0xFFFF - AileronICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.AileronInputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_AILERON_CHANNEL);
      ReceiverInputStates.AileronInputState = PWM_LOW; // Set input state to low

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      AileronICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (AileronICValues.FallingCounter > AileronICValues.RisingCount)
        tempPulseTimerCount = AileronICValues.FallingCounter - AileronICValues.RisingCount;
      else
        tempPulseTimerCount = AileronICValues.FallingCounter + 0xFFFF - AileronICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        AileronICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief       Handles the RC input pulse measurements from the elevator channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateElevatorChannel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.ElevatorInputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_ELEVATOR_CHANNEL);
      ReceiverInputStates.ElevatorInputState = PWM_HIGH; // Set input state to high

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      ElevatorICValues.PreviousRisingCount = ElevatorICValues.RisingCount;
      ElevatorICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(ElevatorICValues.RisingCount > ElevatorICValues.PreviousRisingCount)
        ElevatorICValues.PeriodCount = ElevatorICValues.RisingCount - ElevatorICValues.PreviousRisingCount;
      else
        ElevatorICValues.PeriodCount = ElevatorICValues.RisingCount + 0xFFFF - ElevatorICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.ElevatorInputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_ELEVATOR_CHANNEL);
      ReceiverInputStates.ElevatorInputState = PWM_LOW; // Set input state to low

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ElevatorICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (ElevatorICValues.FallingCounter > ElevatorICValues.RisingCount)
        tempPulseTimerCount = ElevatorICValues.FallingCounter - ElevatorICValues.RisingCount;
      else
        tempPulseTimerCount = ElevatorICValues.FallingCounter + 0xFFFF - ElevatorICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        ElevatorICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief       Handles the RC input pulse measurements from the rudder channel
 *                      and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateRudderChannel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.RudderInputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_RUDDER_CHANNEL);
      ReceiverInputStates.RudderInputState = PWM_HIGH; // Set input state to high

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      RudderICValues.PreviousRisingCount = RudderICValues.RisingCount;
      RudderICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(RudderICValues.RisingCount > RudderICValues.PreviousRisingCount)
        RudderICValues.PeriodCount = RudderICValues.RisingCount - RudderICValues.PreviousRisingCount;
      else
        RudderICValues.PeriodCount = RudderICValues.RisingCount + 0xFFFF - RudderICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.RudderInputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_RUDDER_CHANNEL);
      ReceiverInputStates.RudderInputState = PWM_LOW; // Set input state to low

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      RudderICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (RudderICValues.FallingCounter > RudderICValues.RisingCount)
        tempPulseTimerCount = RudderICValues.FallingCounter - RudderICValues.RisingCount;
      else
        tempPulseTimerCount = RudderICValues.FallingCounter + 0xFFFF - RudderICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        RudderICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief       Handles the RC input pulse measurements from the gear channel
 *                      and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateGearChannel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.GearInputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&AuxReceiverTimHandle, AUX_RECEIVER_GEAR_CHANNEL);
      ReceiverInputStates.GearInputState = PWM_HIGH; // Set input state to high

      AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      GearICValues.PreviousRisingCount = GearICValues.RisingCount;
      GearICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(GearICValues.RisingCount > GearICValues.PreviousRisingCount)
        GearICValues.PeriodCount = GearICValues.RisingCount - GearICValues.PreviousRisingCount;
      else
        GearICValues.PeriodCount = GearICValues.RisingCount + 0xFFFF - GearICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.GearInputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&AuxReceiverTimHandle, AUX_RECEIVER_GEAR_CHANNEL);
      ReceiverInputStates.GearInputState = PWM_LOW; // Set input state to low

      AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      GearICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (GearICValues.FallingCounter > GearICValues.RisingCount)
        tempPulseTimerCount = GearICValues.FallingCounter - GearICValues.RisingCount;
      else
        tempPulseTimerCount = GearICValues.FallingCounter + 0xFFFF - GearICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        GearICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

/*
 * @brief       Handles the RC input pulse measurements from the aux1 channel
 *              and updates pulse and frequency values.
 * @param       None.
 * @retval      None.
 */
static void UpdateAux1Channel(void)
{
  /* Detected rising PWM edge */
  if (ReceiverInputStates.Aux1InputState == PWM_LOW)
    {
      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&AuxReceiverTimHandle, AUX_RECEIVER_AUX1_CHANNEL);
      ReceiverInputStates.Aux1InputState = PWM_HIGH; // Set input state to high

      AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      Aux1ICValues.PreviousRisingCount = Aux1ICValues.RisingCount;
      Aux1ICValues.RisingCount = icValue;

      /* Calculate the period of the 16-bit counter by computing the difference between current and previous rising edge timer counts */
      if(Aux1ICValues.RisingCount > Aux1ICValues.PreviousRisingCount)
        Aux1ICValues.PeriodCount = Aux1ICValues.RisingCount - Aux1ICValues.PreviousRisingCount;
      else
        Aux1ICValues.PeriodCount = Aux1ICValues.RisingCount + 0xFFFF - Aux1ICValues.PreviousRisingCount;
    }
  /* Detected falling PWM edge */
  else if (ReceiverInputStates.Aux1InputState == PWM_HIGH)
    {
      uint32_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&AuxReceiverTimHandle, AUX_RECEIVER_AUX1_CHANNEL);
      ReceiverInputStates.Aux1InputState = PWM_LOW; // Set input state to low

      AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      Aux1ICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (Aux1ICValues.FallingCounter > Aux1ICValues.RisingCount)
        tempPulseTimerCount = Aux1ICValues.FallingCounter - Aux1ICValues.RisingCount;
      else
        tempPulseTimerCount = Aux1ICValues.FallingCounter + 0xFFFF - Aux1ICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseTimerCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseTimerCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        Aux1ICValues.PulseTimerCount = tempPulseTimerCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
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
  // TODO: Check last time pulse
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
        UpdateThrottleChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_AILERON_ACTIVE_CHANNEL)
        UpdateAileronChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL)
        UpdateElevatorChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL)
        UpdateRudderChannel();
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      if(htim->Channel == AUX_RECEIVER_GEAR_ACTIVE_CHANNEL)
        UpdateGearChannel();
      else if(htim->Channel == AUX_RECEIVER_AUX1_ACTIVE_CHANNEL)
        UpdateAux1Channel();
    }
}
