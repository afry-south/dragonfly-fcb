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
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef PrimaryReceiverTimHandle;
TIM_HandleTypeDef AuxReceiverTimHandle;

uint16_t PWM_IN_DOWN[6] = {0, 0, 0, 0, 0, 0}; 	// Current falling edge (for PWM input 1, 2, 3, 4)
uint16_t PWM_IN_UP[6] = {0, 0, 0, 0, 0, 0};		// Current rising edge (for PWM input 1, 2, 3, 4)
uint16_t PWM_IN_UP2[6] = {0, 0, 0, 0, 0, 0};		// Previous rising edge (for PWM input 1, 2, 3, 4)
uint8_t pulseState[6] = {0, 0, 0, 0, 0, 0};			// 0 = rising, 1 = falling detection (for PWM input 1, 2, 3, 4)
uint16_t PWM_IN_DutyCycleTicks[6] = {0, 0, 0, 0, 0, 0};	// Number of PWM duty cycle ticks (for PWM input 1, 2, 3, 4)
uint16_t PWM_IN_PeriodTicks[6] = {0, 0, 0, 0, 0, 0};		// Number of PWM period ticks (for PWM input 1, 2, 3, 4)

uint16_t PWM_IN_DOWN_PRE_2 = 0;
uint16_t RCTimeOut = 50;
uint16_t RCTimeOutCount = 0;

/* RC input limits in microseconds */
float RCmin = 1080, RCmid = 1500, RCmax = 1920;

volatile PWMRC_TimeTypeDef PWMTimes;
PWMRC_TimeTypeDef PWMTimesTemp;

/*
 * @brief       Initializes reading from the receiver primary input channels, i.e.
 *              throttle aileron, elevator and rudder channels. The signals are
 *              encoded as PWM pulses of ~1-2 ms.
 * @param       None.
 * @retval      None.
 */
void PrimaryReceiverInput_Config(void)
{
  /* Timer Input Capture Configuration Structure declaration */
  TIM_IC_InitTypeDef sConfig;

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
  sConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &sConfig, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &sConfig, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of elevator channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &sConfig, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of rudder channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &sConfig, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
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
void AuxReceiverInput_Config(void)
{
  /* Timer Input Capture Configuration Structure declaration */
  TIM_IC_InitTypeDef sConfig;

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
  sConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &sConfig, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &sConfig, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
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
#ifdef TODO
  if (pulseState[0] == 0) // Rising edge
    {
      TIM2_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

      PWM_IN_UP2[0] = PWM_IN_UP[0];
      PWM_IN_UP[0] = TIM_GetCapture1(TIM2);

      if(PWM_IN_UP[0] >= PWM_IN_UP2[0])
        PWM_IN_PeriodTicks[0] = PWM_IN_UP[0] - PWM_IN_UP2[0];
      else
        PWM_IN_PeriodTicks[0] = PWM_IN_UP[0] + 0xFFFF - PWM_IN_UP2[0];

      //printf("Channel 1, Rising edge, Timer ticks = %d \nPeriod ticks = %d", PWM_1_IN_UP, PWM_1_IN_PeriodTicks);
      GPIO_SetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 high
    }
  else      //Falling edge
    {
      TIM2_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
      PWM_IN_DOWN[0] = TIM_GetCapture1(TIM2);

      if (PWM_IN_DOWN[0] >= PWM_IN_UP[0])
        PWM_IN_DutyCycleTicks[0] = PWM_IN_DOWN[0] - PWM_IN_UP[0];
      else
        PWM_IN_DutyCycleTicks[0] = PWM_IN_DOWN[0] + 0xFFFF - PWM_IN_UP[0];

      //Sanity check
      PWMTimesTemp.Throttle = (float)((float)PWM_IN_DutyCycleTicks[0])/((float)PWM_INPUT_SAMPLE_CLOCK);
      if(PWMTimesTemp.Throttle >= 0.0 && PWMTimesTemp.Throttle <= 0.0025)
        PWMTimes.Throttle = PWMTimesTemp.Throttle;

      //printf("Channel 1, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_1_IN_DOWN, PWM_1_IN_DutyCycleTicks);
      GPIO_ResetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 low
    }
  pulseState[0] = !pulseState[0];
  TIM_ICInit(TIM2, &TIM2_CH1_ICInitStructure);      // Reverse polarity
#endif
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

/* @TIM3_Setup
 * @brief	Timer 3 setup
 * @param	None.
 * @retval	None.
 */
void TIM3_Setup(void)
{
#ifdef TODO
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3;	// TIM3 timebase struct

  //TIM3 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

  // TIM3 Time Base configuration
  TIM_TimeBaseStructure3.TIM_Prescaler = SystemCoreClock/PWM_INPUT_SAMPLE_CLOCK -1;	// 72 MHz to 2.4 MHz
  TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure3.TIM_Period = 0xFFFF;	// Can take receiver input down to less than 40 Hz (approx. 45 Hz is used for RC)
  TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

  // TIM3 counter enable
  TIM_Cmd(TIM3, ENABLE);
#endif
}

/* @PWM_In_Setup
 * @brief  Sets up timer 2, GPIO and NVIC for PWM input
 * @param  None
 * @retval None
 */
void PWM_In_Setup(void)
{
#ifdef TODO
//  /* Debug/test pins (TODO: delete later) */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOD, &GPIO_InitStructure);

//
//  /* Connect pins to TIM3 AF2 */
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_2);
//
//  /* Setup TIM3 interrupts */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  // TIM3 Channel 1-2 IC init structs, not used until interrupt handler
//  TIM3_CH1_ICInitStructure.TIM_Channel = TIM_Channel_1;
//  TIM3_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//  TIM3_CH1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM3_CH1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM3_CH1_ICInitStructure.TIM_ICFilter = 0;
//  TIM_ICInit(TIM3, &TIM3_CH1_ICInitStructure);
//
//  TIM3_CH2_ICInitStructure.TIM_Channel = TIM_Channel_2;
//  TIM3_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//  TIM3_CH2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//  TIM3_CH2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//  TIM3_CH2_ICInitStructure.TIM_ICFilter = 0;
//  TIM_ICInit(TIM3, &TIM3_CH2_ICInitStructure);
//
//  //Enable TIM2 CC1-4 interrupt
//  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
//  //Clear TIM2 CC1-4 Flag
//  TIM_ClearFlag(TIM2, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 );
//
//  //Enable TIM3 CC1-2 interrupt
//  TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
//  //Clear TIM3 CC1-2 Flag
//  TIM_ClearFlag(TIM3, TIM_FLAG_CC1 | TIM_FLAG_CC2);
#endif
}
