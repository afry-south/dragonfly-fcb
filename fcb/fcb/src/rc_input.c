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
  PWM_State AuxiliaryInputState;
}PWM_Input_Channel_States_TypeDef;

typedef struct
{
  uint16_t RisingCount;
  uint16_t FallingCounter;
  uint16_t PreviousRisingCount;
  uint16_t PulseCount;
  uint16_t PeriodCount;
}PWM_IC_Values;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef PrimaryReceiverTimHandle;
TIM_HandleTypeDef AuxReceiverTimHandle;
TIM_IC_InitTypeDef PrimaryReceiverICConfig;
TIM_IC_InitTypeDef AuxReceiverICConfig;

PWM_Input_Channel_States_TypeDef ReceiverInputStates;

PWM_IC_Values ThrottleICValues;
PWM_IC_Values AileronICValues;
PWM_IC_Values ElevatorICValues;
PWM_IC_Values RudderICValues;
PWM_IC_Values GearICValues;
PWM_IC_Values Aux1ICValues;

uint16_t PWM_IN_DOWN_PRE_2 = 0;
uint16_t RCTimeOut = 50;
uint16_t RCTimeOutCount = 0;

// TODO DELETE LATER
/* RC input limits in microseconds */
float RCmin = 1080, RCmid = 1500, RCmax = 1920;

volatile PWMRC_TimeTypeDef PWMTimes;
PWMRC_TimeTypeDef PWMTimesTemp;
// END TODO

/* Private function prototypes -----------------------------------------------*/
static void PrimaryReceiverInput_Config(void);
static void AuxReceiverInput_Config(void);

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input PWM signals
 * @param  None
 * @retval None
 */
void ReceiverInput_Config(void)
{
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

void UpdateThrottleChannel(void)
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
      uint16_t tempPulseCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL);
      ReceiverInputStates.ThrottleInputState = PWM_HIGH; // Set input state to low

      PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ThrottleICValues.FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      if (ThrottleICValues.FallingCounter > ThrottleICValues.RisingCount)
        tempPulseCount = ThrottleICValues.FallingCounter - ThrottleICValues.RisingCount;
      else
        tempPulseCount = ThrottleICValues.FallingCounter + 0xFFFF - ThrottleICValues.RisingCount;

      /* Sanity check of pulse count before updating it */
      if(tempPulseCount <= RECEIVER_MAX_ALLOWED_IC_PULSE_COUNT && tempPulseCount >= RECEIVER_MIN_ALLOWED_IC_PULSE_COUNT)
        ThrottleICValues.PulseCount = tempPulseCount;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }
}

void UpdateAileronChannel(void)
{
#ifdef TODO
  if (pulseState[1] == 0) // Rising edge
    {
      TIM2_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      PWM_IN_UP2[1] = PWM_IN_UP[1];
      PWM_IN_UP[1] = TIM_GetCapture2(TIM2);

      if(PWM_IN_UP[1] >= PWM_IN_UP2[1])
        PWM_IN_PeriodTicks[1] = PWM_IN_UP[1] - PWM_IN_UP2[1];
      else
        PWM_IN_PeriodTicks[1] = PWM_IN_UP[1] + 0xFFFF - PWM_IN_UP2[1];

      //printf("Channel 2, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_2_IN_UP, PWM_2_IN_PeriodTicks);
      GPIO_SetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 high
    }
  else      //Falling edge
    {
      TIM2_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[1] = TIM_GetCapture2(TIM2);

      if (PWM_IN_DOWN[1] >= PWM_IN_UP[1])
        PWM_IN_DutyCycleTicks[1] = PWM_IN_DOWN[1] - PWM_IN_UP[1];
      else
        PWM_IN_DutyCycleTicks[1] = PWM_IN_DOWN[1] + 0xFFFF - PWM_IN_UP[1];

      //Sanity check
      PWMTimesTemp.Aileron = (float)((float)PWM_IN_DutyCycleTicks[1])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Aileron >= 0.0 && PWMTimesTemp.Aileron <= 0.0025)
        PWMTimes.Aileron = PWMTimesTemp.Aileron;

      //printf("Channel 2, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_2_IN_DOWN, PWM_2_IN_DutyCycleTicks);
      GPIO_ResetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 low
    }
  pulseState[1] = !pulseState[1];
  TIM_ICInit(TIM2, &TIM2_CH2_ICInitStructure);      // Reverse polarity
#endif
}

void UpdateElevatorChannel(void)
{
#ifdef TODO
  if (pulseState[2] == 0) // Rising edge
    {
      TIM2_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      PWM_IN_UP2[2] = PWM_IN_UP[2];
      PWM_IN_UP[2] = TIM_GetCapture3(TIM2);

      if(PWM_IN_UP[2] >= PWM_IN_UP2[2])
        PWM_IN_PeriodTicks[2] = PWM_IN_UP[2] - PWM_IN_UP2[2];
      else
        PWM_IN_PeriodTicks[2] = PWM_IN_UP[2] + 0xFFFF - PWM_IN_UP2[2];
    }
  else      //Falling edge
    {
      TIM2_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[2] = TIM_GetCapture3(TIM2);

      if (PWM_IN_DOWN[2] >= PWM_IN_UP[2])
        PWM_IN_DutyCycleTicks[2] = PWM_IN_DOWN[2] - PWM_IN_UP[2];
      else
        PWM_IN_DutyCycleTicks[2] = PWM_IN_DOWN[2] + 0xFFFF - PWM_IN_UP[2];

      //Sanity check
      PWMTimesTemp.Elevator = (float)((float)PWM_IN_DutyCycleTicks[2])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Elevator >= 0.0 && PWMTimesTemp.Elevator <= 0.0025)
        PWMTimes.Elevator = PWMTimesTemp.Elevator;
    }
  pulseState[2] = !pulseState[2];
  TIM_ICInit(TIM2, &TIM2_CH3_ICInitStructure); // Reverse polarity
#endif
}

void UpdateRudderChannel(void)
{
#ifdef TODO
  if (pulseState[3] == 0) // Rising edge
    {
      TIM2_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      PWM_IN_UP2[3] = PWM_IN_UP[3];
      PWM_IN_UP[3] = TIM_GetCapture4(TIM2);

      if(PWM_IN_UP[3] >= PWM_IN_UP2[3])
        PWM_IN_PeriodTicks[3] = PWM_IN_UP[3] - PWM_IN_UP2[3];
      else
        PWM_IN_PeriodTicks[3] = PWM_IN_UP[3] + 0xFFFF - PWM_IN_UP2[3];
    }
  else      //Falling edge
    {
      TIM2_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[3] = TIM_GetCapture4(TIM2);

      if (PWM_IN_DOWN[3] >= PWM_IN_UP[3])
        PWM_IN_DutyCycleTicks[3] = PWM_IN_DOWN[3] - PWM_IN_UP[3];
      else
        PWM_IN_DutyCycleTicks[3] = PWM_IN_DOWN[3] + 0xFFFF - PWM_IN_UP[3];

      //Sanity check
      PWMTimesTemp.Rudder = (float)((float)PWM_IN_DutyCycleTicks[3])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Rudder >= 0.0 && PWMTimesTemp.Rudder <= 0.0025)
        PWMTimes.Rudder = PWMTimesTemp.Rudder;
    }
  pulseState[3] = !pulseState[3];
  TIM_ICInit(TIM2, &TIM2_CH4_ICInitStructure);      // Reverse polarity
#endif
}

void UpdateGearChannel(void)
{
#ifdef TODO
  if (pulseState[4] == 0) // Rising edge
    {
      TIM3_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      PWM_IN_UP2[4] = PWM_IN_UP[4];
      PWM_IN_UP[4] = TIM_GetCapture1(TIM3);

      if(PWM_IN_UP[4] >= PWM_IN_UP2[4])
        PWM_IN_PeriodTicks[4] = PWM_IN_UP[4] - PWM_IN_UP2[4];
      else
        PWM_IN_PeriodTicks[4] = PWM_IN_UP[4] + 0xFFFF - PWM_IN_UP2[4];

      GPIO_SetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 high
    }
  else      //Falling edge
    {
      TIM3_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[4] = TIM_GetCapture1(TIM3);

      if (PWM_IN_DOWN[4] >= PWM_IN_UP[4])
        PWM_IN_DutyCycleTicks[4] = PWM_IN_DOWN[4] - PWM_IN_UP[4];
      else
        PWM_IN_DutyCycleTicks[4] = PWM_IN_DOWN[4] + 0xFFFF - PWM_IN_UP[4];

      //Sanity check
      PWMTimesTemp.Gear = (float)((float)PWM_IN_DutyCycleTicks[4])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Gear >= 0.0 && PWMTimesTemp.Gear <= 0.0025)
        PWMTimes.Gear = PWMTimesTemp.Gear;

      GPIO_ResetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 low
    }
  pulseState[4] = !pulseState[4];
  TIM_ICInit(TIM3, &TIM3_CH1_ICInitStructure);      // Reverse polarity
#endif
}

void UpdateAuxiliaryChannel(void)
{
#ifdef TODO
  if (pulseState[5] == 0) // Rising edge
    {
      TIM3_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
      PWM_IN_UP2[5] = PWM_IN_UP[5];
      PWM_IN_UP[5] = TIM_GetCapture2(TIM3);

      if(PWM_IN_UP[5] >= PWM_IN_UP2[5])
        PWM_IN_PeriodTicks[5] = PWM_IN_UP[5] - PWM_IN_UP2[5];
      else
        PWM_IN_PeriodTicks[5] = PWM_IN_UP[5] + 0xFFFF - PWM_IN_UP2[5];

      GPIO_SetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 high
    }
  else      //Falling edge
    {
      TIM3_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[5] = TIM_GetCapture2(TIM3);

      if (PWM_IN_DOWN[5] >= PWM_IN_UP[5])
        PWM_IN_DutyCycleTicks[5] = PWM_IN_DOWN[5] - PWM_IN_UP[5];
      else
        PWM_IN_DutyCycleTicks[5] = PWM_IN_DOWN[5] + 0xFFFF - PWM_IN_UP[5];

      //Sanity check
      PWMTimesTemp.Auxiliary = (float)((float)PWM_IN_DutyCycleTicks[5])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Auxiliary >= 0.0 && PWMTimesTemp.Auxiliary <= 0.0025)
        PWMTimes.Auxiliary = PWMTimesTemp.Auxiliary;

      GPIO_ResetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 low
    }
  pulseState[5] = !pulseState[5];
  TIM_ICInit(TIM3, &TIM3_CH2_ICInitStructure);      // Reverse polarity
#endif
}

/* @CheckRCConnection
 * @brief	Checks that the RC transmitter and receiver are connected.
 * @param	None.
 * @retval	None.
 */
char CheckRCConnection()
{
#ifdef TODO
  char retVal = 0;

  if(PWM_IN_DOWN[1] != PWM_IN_DOWN_PRE_2)
    {
      RCTimeOutCount = 0;
      retVal = 1;
    }
  else if(RCTimeOutCount < RCTimeOut && PWM_IN_DOWN[1] == PWM_IN_DOWN_PRE_2)
    {
      RCTimeOutCount++;
      retVal = 1;
    }
  else
    {
      PWMTimes.Throttle = RCmin;
      PWMTimes.Aileron = RCmid;
      PWMTimes.Elevator = RCmid;
      PWMTimes.Rudder = RCmid;
      PWMTimes.Gear = RCmin;
      PWMTimes.Auxiliary = RCmin;
      retVal = 0;
    }

  PWM_IN_DOWN_PRE_2 = PWM_IN_DOWN[1];

  return retVal;
#endif
  return 0;
}

/* @GetPWMInputTimes
 * @brief	Gets the receiver PWM times
 */
void GetPWMInputTimes(PWMRC_TimeTypeDef *PWM_Time)
{
  memcpy((void*)PWM_Time ,(const void*)(&PWMTimes), sizeof(PWMRC_TimeTypeDef));
}

/* @GetRCmin
 * @brief	Gets the minimum receiver value
 */
float GetRCmin(void)
{
  return RCmin;
}

/* @GetRCmid
 * @brief	Gets the midpoint receiver value
 */
float GetRCmid(void)
{
  return RCmid;
}

/* @GetRCmax
 * @brief	Gets the maximum receiver value
 */
float GetRCmax(void)
{
  return RCmax;
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
        {
#ifdef TODO
          /* Get the Input Capture value */
//          uint32_t aileronInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, PRIMARY_RECEIVER_AILERON_CHANNEL);
          // UpdateAileronChannel
#endif
        }
      else if(htim->Channel == PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL)
        {
#ifdef TODO
          /* Get the Input Capture value */
//          uint32_t elevatorInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, PRIMARY_RECEIVER_ELEVATOR_CHANNEL);
          // UpdatedElevatorChannel();
#endif
        }
      else if(htim->Channel == PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL)
        {
#ifdef TODO
          /* Get the Input Capture value */
//          uint32_t rudderInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, PRIMARY_RECEIVER_RUDDER_CHANNEL);
          // UpdateRudderChannel();
#endif
        }
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      if(htim->Channel == AUX_RECEIVER_GEAR_ACTIVE_CHANNEL)
        {
#ifdef TODO
          /* Get the Input Capture value */
//          uint32_t gearInputCaptureValue = HAL_TIM_ReadCapturedValue(htim, AUX_RECEIVER_GEAR_CHANNEL);
          // UpdateGearChannel();
#endif
        }
      else if(htim->Channel == AUX_RECEIVER_AUX1_ACTIVE_CHANNEL)
        {
#ifdef TODO
          /* Get the Input Capture value */
//          uint32_t aux1InputCaptureValue = HAL_TIM_ReadCapturedValue(htim, AUX_RECEIVER_AUX1_CHANNEL);
          // UpdateAux1Channel();
#endif
        }
    }
}
