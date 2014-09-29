/**
******************************************************************************
* @file    fcb/main.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-26
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "sensors.c"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;	// TIM2
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure4; // TIM3
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3; // TIM4

TIM_OCInitTypeDef TIM_OCInitStructure;

TIM_ICInitTypeDef TIM_CH1_ICInitStructure;
TIM_ICInitTypeDef TIM_CH2_ICInitStructure;
TIM_ICInitTypeDef TIM_CH3_ICInitStructure;
TIM_ICInitTypeDef TIM_CH4_ICInitStructure;

volatile uint16_t PWM_IN_DOWN[4] = {0, 0, 0, 0}; 	// Current falling edge
volatile uint16_t PWM_IN_UP[4] = {0, 0, 0, 0};		// Current rising edge
volatile uint16_t PWM_IN_UP2[4] = {0, 0, 0, 0};		// Previous rising edge
volatile char pulseState[4] = {0, 0, 0, 0};			// 0 = rising, 1 = falling detection

volatile uint16_t PWM_1_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_2_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_3_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_4_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_1_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm1 in signal
volatile uint16_t PWM_2_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm2 in signal
volatile uint16_t PWM_3_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm3 in signal
volatile uint16_t PWM_4_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm4 in signal

/* Computes prescaler and timer period to adjust output signal frequency to 400 Hz */
uint16_t TIM4_Prescaler = 0;	// Timer 4 prescaler (recalculated in TIM4_Setup())
uint16_t TIM4_Period = 60000;	// TIM4 clock period (set to obtain 400 MHz PWM output)

// Sensor readings arrays
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};

/* Private functions -----------------------------------------------*/
static void TIM4_IOconfig(void);
static void TIM4_Setup(void);
static void TIM4_SetupOC(void);

static void TIM3_Setup(void);
static void TIM3_SetupIRQ(void);

static void PWM_In_Setup(void);
static void TIM2_Setup(void);

static uint16_t getPWM_CCR(float dutycycle);

/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
	/* At this stage the microcontroller clock setting is already configured,
	 * this is done through SystemInit() function which is called from startup
	 * file (startup_stm32f30x.s) before to branch to application main.
	 * To reconfigure the default setting of SystemInit() function, refer to
	 * system_stm32f30x.c file
	 */

	/* TIM GPIO configuration */
	TIM4_IOconfig();

	/* Setup Timer 4 (used for PWM output)*/
  	TIM4_Setup();
	/* Setup Timer 4 OC registers (for PWM output) */
	TIM4_SetupOC();

	/* Setup Timer 3 (used for program periodic execution) */
	TIM3_Setup();
	/* Setup Timer 3 for interrupt generation */
	TIM3_SetupIRQ();

	/* Setup Timer 2 (used for PWM input) */
	TIM2_Setup();
	/* Setup PWM input (GPIO, NVIC settings) */
	PWM_In_Setup();

	/* Setup sensors */
	GyroConfig();
	CompassConfig();

	/* Infinite loop keeps the program alive.
	 * The actual program tasks are performed by
	 * the TIM3_IRQHandler interrupt handler
	 */
	while (1);
}

/* @TIM3_IRQHandler
 * @brief	Timer 3 interrupt handler.
 * 			Continually performs program duties with regular intervals.
 * @param	None.
 * @retval	None.
 */
void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		/* Read Gyro Angular rate data */
		GyroReadAngRate(GyroBuffer);
		/* Read Compass data */
		CompassReadMag(MagBuffer);
		CompassReadAcc(AccBuffer);

		// Set motor output PWM
		TIM4->CCR1 = getPWM_CCR((float)(PWM_1_IN_DutyCycleTicks/PWM_1_IN_PeriodTicks));
		TIM4->CCR2 = getPWM_CCR((float)(PWM_2_IN_DutyCycleTicks/PWM_2_IN_PeriodTicks));
		TIM4->CCR3 = getPWM_CCR((float)(PWM_3_IN_DutyCycleTicks/PWM_3_IN_PeriodTicks));
		TIM4->CCR4 = getPWM_CCR((float)(PWM_4_IN_DutyCycleTicks/PWM_4_IN_PeriodTicks));
	}
}

/* @TIM2_IRQHandler
 * @brief	Timer 2 interrupt handler.
 * @param	None.
 * @retval	None.
 */
void TIM2_IRQHandler()
{
	/* If interrupt concerns CH1 */
	if (TIM2->SR & TIM_IT_CC1)
	{
		TIM2->SR &= (~TIM_IT_CC1);

		if (pulseState[0] == 0)	// Rising edge
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_IN_UP2[0] = PWM_IN_UP[0];
			PWM_IN_UP[0] = TIM_GetCapture1(TIM2);

			if(PWM_IN_UP[0] >= PWM_IN_UP2[0])
				PWM_1_IN_PeriodTicks = PWM_IN_UP[0] - PWM_IN_UP2[0];
			else
				PWM_1_IN_PeriodTicks = PWM_IN_UP[0] + 0xFFFF - PWM_IN_UP2[0];

			//printf("Channel 1, Rising edge, Timer ticks = %d \nPeriod ticks = %d", PWM_1_IN_UP, PWM_1_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 high
		}
		else	//Falling edge
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[0] = TIM_GetCapture1(TIM2);

			if (PWM_IN_DOWN[0] >= PWM_IN_UP[0])
				PWM_1_IN_DutyCycleTicks = PWM_IN_DOWN[0] - PWM_IN_UP[0];
			else
				PWM_1_IN_DutyCycleTicks = PWM_IN_DOWN[0] + 0xFFFF - PWM_IN_UP[0];

			//printf("Channel 1, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_1_IN_DOWN, PWM_1_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_8); //Debugging set GPIOD pin 8 low
		}
		pulseState[0] = !pulseState[0];
		TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);	// Reverse polarity
	}


	/* If interrupt concerns CH2 */
	if (TIM2->SR & TIM_IT_CC2)
	{
		TIM2->SR &= (~TIM_IT_CC2);

		if (pulseState[1] == 0)	// Rising edge
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[1] = PWM_IN_UP[1];
			PWM_IN_UP[1] = TIM_GetCapture2(TIM2);

			if(PWM_IN_UP[1] >= PWM_IN_UP2[1])
				PWM_2_IN_PeriodTicks = PWM_IN_UP[1] - PWM_IN_UP2[1];
			else
				PWM_2_IN_PeriodTicks = PWM_IN_UP[1] + 0xFFFF - PWM_IN_UP2[1];

			//printf("Channel 2, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_2_IN_UP, PWM_2_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 high
		}
		else	//Falling edge
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[1] = TIM_GetCapture2(TIM2);

			if (PWM_IN_DOWN[1] >= PWM_IN_UP[1])
				PWM_2_IN_DutyCycleTicks = PWM_IN_DOWN[1] - PWM_IN_UP[1];
			else
				PWM_2_IN_DutyCycleTicks = PWM_IN_DOWN[1] + 0xFFFF - PWM_IN_UP[1];

			//printf("Channel 2, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_2_IN_DOWN, PWM_2_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_9); //Debugging set GPIOD pin 9 low
		}
		pulseState[1] = !pulseState[1];
		TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);	// Reverse polarity
	}


	/* If interrupt concerns CH3 */
	if (TIM2->SR & TIM_IT_CC3)
	{
		TIM2->SR &= (~TIM_IT_CC3);

		if (pulseState[2] == 0)	// Rising edge
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[2] = PWM_IN_UP[2];
			PWM_IN_UP[2] = TIM_GetCapture3(TIM2);

			if(PWM_IN_UP[2] >= PWM_IN_UP2[2])
				PWM_3_IN_PeriodTicks = PWM_IN_UP[2] - PWM_IN_UP2[2];
			else
				PWM_3_IN_PeriodTicks = PWM_IN_UP[2] + 0xFFFF - PWM_IN_UP2[2];

			//printf("Channel 3, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_3_IN_UP, PWM_3_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 high
		}
		else	//Falling edge
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[2] = TIM_GetCapture3(TIM2);

			if (PWM_IN_DOWN[2] >= PWM_IN_UP[2])
				PWM_3_IN_DutyCycleTicks = PWM_IN_DOWN[2] - PWM_IN_UP[2];
			else
				PWM_3_IN_DutyCycleTicks = PWM_IN_DOWN[2] + 0xFFFF - PWM_IN_UP[2];

			//printf("Channel 3, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_3_IN_DOWN, PWM_3_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_10); //Debugging set GPIOD pin 10 low
		}
		pulseState[2] = !pulseState[2];
		TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure); // Reverse polarity
	}

	/* If interrupt concerns CH4 */
	if (TIM2->SR & TIM_IT_CC4)
	{
		TIM2->SR &= (~TIM_IT_CC4);

		if (pulseState[3] == 0)	// Rising edge
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			PWM_IN_UP2[3] = PWM_IN_UP[3];
			PWM_IN_UP[3] = TIM_GetCapture4(TIM2);

			if(PWM_IN_UP[3] >= PWM_IN_UP2[3])
				PWM_4_IN_PeriodTicks = PWM_IN_UP[3] - PWM_IN_UP2[3];
			else
				PWM_4_IN_PeriodTicks = PWM_IN_UP[3] + 0xFFFF - PWM_IN_UP2[3];

			//printf("Channel 4, Rising edge, Timer ticks = %d\nPeriod ticks = %d", PWM_4_IN_UP, PWM_4_IN_PeriodTicks);
			GPIO_SetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 high
		}
		else	//Falling edge
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			PWM_IN_DOWN[3] = TIM_GetCapture4(TIM2);

			if (PWM_IN_DOWN[3] >= PWM_IN_UP[3])
				PWM_4_IN_DutyCycleTicks = PWM_IN_DOWN[3] - PWM_IN_UP[3];
			else
				PWM_4_IN_DutyCycleTicks = PWM_IN_DOWN[3] + 0xFFFF - PWM_IN_UP[3];

			//printf("Channel 4, Falling edge, Timer ticks = %d \nDuty cycle ticks = %d", PWM_4_IN_DOWN, PWM_4_IN_DutyCycleTicks);
			GPIO_ResetBits(GPIOD, GPIO_Pin_11); //Debugging set GPIOD pin 11 low
		}
		pulseState[3] = !pulseState[3];
		TIM_ICInit(TIM2, &TIM_CH4_ICInitStructure);	// Reverse polarity
	}
}

/* @TIM2_Setup
 * @brief	Timer 2 setup
 * @param	None.
 * @retval	None.
 */
static void TIM2_Setup(void) {
	//TIM2 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	// TIM2 Time Base configuration
	TIM_TimeBaseStructure2.TIM_Prescaler = 30-1;	// 72 MHz to 2.4 MHz
	TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure2.TIM_Period = 0xFFFF;	// Can take receiver input down to less than 40 Hz (50 Hz is common?)
	TIM_TimeBaseStructure2.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure2.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);

	// TIM2 counter enable
	TIM_Cmd(TIM2, ENABLE);
}

/* @getPWM_CCR
 * @brief	Returns the CCR (Capture compare register) value for a specified PWM duty cycle.
 * @param	dutycycle: The dutycycle percentage in decimal form (i.e. 45.2% = 0.452)
 * @retval	CCR value (Nbr of PWM clock ticks with voltage) corresponding to the desired duty cycle.
 */
static uint16_t getPWM_CCR(float dutycycle)
{
	return (uint16_t) (dutycycle * TIM4_Period);
}

/* @TIM4_IOconfig
 * @brief	Initializes and configures output pins used to
 * 			physically transmit PWM motor control signals.
 * @param	None.
 * @retval	None.
 */
static void TIM4_IOconfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIO D clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_2);
}

/* @TIM4_Setup
 * @brief	Sets up the Timer 4 timebase. Timer 4 is responsible
 * 			for producing the PWM motor control signals at 400 Hz.
 * @param	None.
 * @retval	None.
 */
static void TIM4_Setup(void)
{
	TIM4_Prescaler = SystemCoreClock/24000000;	// SystemCoreClock (72 MHz on STM32F303) to 24 MHz (Prescaler = 3)

	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);

	/* TIM4 Time Base configuration */
	TIM_TimeBaseStructure4.TIM_Prescaler = TIM4_Prescaler-1;
	TIM_TimeBaseStructure4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure4.TIM_Period = TIM4_Period-1;
	TIM_TimeBaseStructure4.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure4.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure4);

	/* TIM4 counter enable */
	TIM_Cmd(TIM4, ENABLE);
}

/* @TIM3_Setup
 * @brief	Sets up the Timer 3 timebase. Timer 3 is responsible for
 * 			generating system interrupts at well-defined intervals used
 * 			to execute code at discrete time intervals.
 * @param	None.
 * @retval	None.
 */
static void TIM3_Setup(void)
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);

	/* TIM3 Time Base configuration */
	TIM_TimeBaseStructure3.TIM_Prescaler = SystemCoreClock/1000000 - 1;	// 72 MHz to 1 MHz
	TIM_TimeBaseStructure3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure3.TIM_Period = 10000 - 1;						// 1 Mhz to 100 Hz
	TIM_TimeBaseStructure3.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure3);

	/* TIM3 counter enable */
	TIM_Cmd(TIM3, ENABLE);
}

/* @TIM4_SetupOC
 * @brief  Configures the Timer 4 OC registers for PWM output
 * 		   Channel 1 outputs on pin PD12
 * 		   Channel 2 outputs on pin PD13
 * 		   Channel 3 outputs on pin PD14
 * 		   Channel 4 outputs on pin PD15
 * @param  None
 * @retval None
 */
static void TIM4_SetupOC(void)
{
	/* Channel 1, 2,3, 4 configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	// Initializes Timer 4 OC registers (Channels 1, 2, 3, 4)
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	/* TIM4 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

/* @PWM_In_Setup
 * @brief  Sets up timer 2, GPIO and NVIC for PWM input
 * @param  None
 * @retval None
 */
static void PWM_In_Setup(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* GPIOD clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* TIM2 GPIO pin configuration : CH1=PD3, C2=PD4, CH3=PD7, CH4=PD6 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect pins to TIM2 AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_2);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Timers 1-4 IC init structs, not used until interrupt handler
	TIM_CH1_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH1_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);

	TIM_CH2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH2_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);

	TIM_CH3_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH3_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure);

	TIM_CH4_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH4_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH4_ICInitStructure);

	//Enable TIM2
	TIM_Cmd(TIM2, ENABLE);

	//Enable CC1-4 interrupt
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	//Clear CC1 Flag
	TIM_ClearFlag(TIM2, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4 );
}


/* TIM3_SetupIRQ
 * @brief  Configures the Timer 3 IRQ Handler.
 * @param  None
 * @retval None
 */
static void TIM3_SetupIRQ(void)
{
	/* Interrupt config */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
    nvicStructure.NVIC_IRQChannelSubPriority = 0;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);
}


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/

/**
* @}
*/
 /*****END OF FILE****/
