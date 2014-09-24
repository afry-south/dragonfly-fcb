/**
******************************************************************************
* @file    PWM_Test/main.c
* @author  ÅF Dragonfly - Embedded
* @version V0.0.1
* @date    2014-09-19
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */


/* Private variables ---------------------------------------------------------*/

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;	// TIM2
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure4; // TIM3
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure3; // TIM4

TIM_OCInitTypeDef TIM_OCInitStructure;

TIM_ICInitTypeDef TIM_CH1_ICInitStructure;
TIM_ICInitTypeDef TIM_CH2_ICInitStructure;
TIM_ICInitTypeDef TIM_CH3_ICInitStructure;
TIM_ICInitTypeDef TIM_CH4_ICInitStructure;

volatile uint16_t PWM_1_IN_DOWN = 0;	// Current falling edge
volatile uint16_t PWM_2_IN_DOWN = 0;
volatile uint16_t PWM_3_IN_DOWN = 0;
volatile uint16_t PWM_4_IN_DOWN = 0;
volatile uint16_t PWM_1_IN_UP = 0;	// Current rising edge
volatile uint16_t PWM_2_IN_UP = 0;
volatile uint16_t PWM_3_IN_UP = 0;
volatile uint16_t PWM_4_IN_UP = 0;
volatile uint16_t PWM_1_IN_UP2 = 0; 	// Previous rising edge
volatile uint16_t PWM_2_IN_UP2 = 0;
volatile uint16_t PWM_3_IN_UP2 = 0;
volatile uint16_t PWM_4_IN_UP2 = 0;

volatile uint16_t PWM_1_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_2_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_3_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_4_IN_PeriodTicks = 0; 	//Number of timer ticks of the period in the pwm1 in signal
volatile uint16_t PWM_1_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm1 in signal
volatile uint16_t PWM_2_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm2 in signal
volatile uint16_t PWM_3_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm3 in signal
volatile uint16_t PWM_4_IN_DutyCycleTicks = 0; //Number of timer ticks of the duty cycle in the pwm4 in signal

uint32_t PWM_IN_Frequency = 0;		//Frequency of the pwm in signal

static volatile char pulseState1 = 0;
static volatile char pulseState2 = 0;
static volatile char pulseState3 = 0;
static volatile char pulseState4 = 0;

uint16_t PWM_PeriodTicks = 0;	// Number of (prescaled) clock ticks per PWM period
uint16_t PrescalerValue = 0;	// Scales the SystemCoreClock frequency

RCC_ClocksTypeDef RCC_Clocks;

uint32_t i = 0; // counter (delete later)

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

static void GyroConfig(void);
static void GyroReadAngRate(float* pfData);
static void CompassConfig(void);
static void CompassReadAcc(float* pfData);
static void CompassReadMag(float* pfData);

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

	/* Computes prescaler and timer period to adjust output signal frequency to 400 Hz */
  	PrescalerValue = SystemCoreClock/24000000;	// SystemCoreClock (72 MHz on STM32F303) to 24 MHz (Prescaler = 3)
  	PWM_PeriodTicks = 60000;					// 24 MHz to 400 Hz output signal frequency (yields a timer value of 60 000 set in the ARR register) */

	/* Setup Timer 4 */
  	TIM4_Setup();
	/* Setup Timer 4 OC registers (for PWM output) */
	TIM4_SetupOC();

	/* Setup Timer 3 */
	TIM3_Setup();
	/* Setup Timer 3 IRQ */
	TIM3_SetupIRQ();

	/* Setup Timer 2 */
	TIM2_Setup();

	/* Setup sensors */
	GyroConfig();
	CompassConfig();

	/* Setup PWM input */
	PWM_In_Setup();

	TIM4->CCR1 = getPWM_CCR(0.20);
	TIM4->CCR2 = getPWM_CCR(0.50);
	TIM4->CCR3 = getPWM_CCR(0.70);
	TIM4->CCR4 = getPWM_CCR(0.90);

	/* Infinite loop keeps the program alive.
	 * The actual program tasks are performed by
	 * the TIM3_IRQHandler interrupt handler
	 */
	while (1)
	{
	}

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
		i++;
		if(i == 1000) {
			/* Read Gyro Angular rate data */
			//GyroReadAngRate(GyroBuffer);
			/* Read Compass data */
			//CompassReadMag(MagBuffer);
			//CompassReadAcc(AccBuffer);

			// 40% dutycycle for 10 sec
//			TIM4->CCR1 = getPWM_CCR(0.40);
//			TIM4->CCR2 = getPWM_CCR(0.40);
//			TIM4->CCR3 = getPWM_CCR(0.40);
//			TIM4->CCR4 = getPWM_CCR(0.40);
		}
		else if(i == 2000) {
			/* Read Gyro Angular rate data */
			//GyroReadAngRate(GyroBuffer);
			/* Read Compass data */
			//CompassReadMag(MagBuffer);
			//CompassReadAcc(AccBuffer);

			// 50% dutycycle for 10 sec
//			TIM4->CCR1 = getPWM_CCR(0.50);
//			TIM4->CCR2 = getPWM_CCR(0.50);
//			TIM4->CCR3 = getPWM_CCR(0.50);
//			TIM4->CCR4 = getPWM_CCR(0.50);
//			i = 0;
		}
	}
}

/* @TIM2_IRQHandler
 * @brief	Timer 2 interrupt handler.
 * @param	None.
 * @retval	None.
 */
void TIM2_IRQHandler()
{
	//If interrupt concerning CH1
	if (TIM2->SR & TIM_IT_CC1)
	{
		TIM2->SR &= (~TIM_IT_CC1);
		// Rising edge
		if (pulseState1 == 0)
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_1_IN_UP2 = PWM_1_IN_UP;
			PWM_1_IN_UP = TIM_GetCapture1(TIM2);

			if(PWM_1_IN_UP >= PWM_1_IN_UP2)
				PWM_1_IN_PeriodTicks = PWM_1_IN_UP - PWM_1_IN_UP2;
			else
				PWM_1_IN_PeriodTicks = PWM_1_IN_UP + 0xFFFF - PWM_1_IN_UP2;
		}
		//Falling edge
		else
		{
			TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			PWM_1_IN_DOWN = TIM_GetCapture1(TIM2);

			if (PWM_1_IN_DOWN >= PWM_1_IN_UP)
				PWM_1_IN_DutyCycleTicks = PWM_1_IN_DOWN - PWM_1_IN_UP;
			else
				PWM_1_IN_DutyCycleTicks = PWM_1_IN_DOWN + 0xFFFF - PWM_1_IN_UP;
		}
		pulseState1 = !pulseState1;

		// Reverse polarity.
		TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);
	}


	//If interrupt concerning CH2
	if (TIM2->SR & TIM_IT_CC2)
	{
		TIM2->SR &= (~TIM_IT_CC2);
		// Rising edge
		if (pulseState2 == 0)
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_2_IN_UP2 = PWM_2_IN_UP;
			PWM_2_IN_UP = TIM_GetCapture2(TIM2);

			if (PWM_2_IN_UP >= PWM_2_IN_UP2)
				PWM_2_IN_PeriodTicks = PWM_2_IN_UP - PWM_2_IN_UP2;
			else
				PWM_2_IN_PeriodTicks = PWM_2_IN_UP + 0xFFFF - PWM_2_IN_UP2;
		}
		// Falling edge
		else
		{
			TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			PWM_2_IN_DOWN = TIM_GetCapture2(TIM2);
			if (PWM_2_IN_DOWN >= PWM_2_IN_UP)
				PWM_2_IN_DutyCycleTicks = PWM_2_IN_DOWN - PWM_2_IN_UP;
			else
				PWM_2_IN_DutyCycleTicks = PWM_2_IN_DOWN + 0xFFFF - PWM_2_IN_UP;
		}
		pulseState2 = !pulseState2;

		// Reverse polarity
		TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);
	}


	//If interrupt concerning CH3
	if (TIM2->SR & TIM_IT_CC3)
	{
		TIM2->SR &= (~TIM_IT_CC3);
		// Rising edge
		if (pulseState3 == 0)
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_3_IN_UP2 = PWM_3_IN_UP;
			PWM_3_IN_UP = TIM_GetCapture3(TIM2);

			if (PWM_3_IN_UP >= PWM_3_IN_UP2)
				PWM_3_IN_PeriodTicks = PWM_3_IN_UP - PWM_3_IN_UP2;
			else
				PWM_3_IN_PeriodTicks = PWM_3_IN_UP + 0xFFFF - PWM_3_IN_UP2;
		}
		// Falling edge
		else
		{
			TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			PWM_3_IN_DOWN = TIM_GetCapture3(TIM2);

			if (PWM_3_IN_DOWN >= PWM_3_IN_UP)
				PWM_3_IN_DutyCycleTicks = PWM_3_IN_DOWN - PWM_3_IN_UP;
			else
				PWM_3_IN_DutyCycleTicks = PWM_3_IN_DOWN + 0xFFFF - PWM_3_IN_UP;
		}
		pulseState3 = !pulseState3;

		// Reverse polarity
		TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure);
	}


	//If interrupt concerning CH4
	if (TIM2->SR & TIM_IT_CC4)
	{
		TIM2->SR &= (~TIM_IT_CC4);
		// Rising edge
		if (pulseState4 == 0)
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;

			PWM_4_IN_UP2 = PWM_4_IN_UP;
			PWM_4_IN_UP = TIM_GetCapture4(TIM2);

			if (PWM_4_IN_UP >= PWM_4_IN_UP2)
				PWM_4_IN_PeriodTicks = PWM_4_IN_UP - PWM_4_IN_UP2;
			else
				PWM_4_IN_PeriodTicks = PWM_4_IN_UP + 0xFFFF - PWM_4_IN_UP2;
		}
		// Falling edge
		else
		{
			TIM_CH4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

			PWM_4_IN_DOWN = TIM_GetCapture4(TIM2);
			if (PWM_4_IN_DOWN >= PWM_4_IN_UP)
				PWM_4_IN_DutyCycleTicks = PWM_4_IN_DOWN - PWM_4_IN_UP;
			else
				PWM_4_IN_DutyCycleTicks = PWM_4_IN_DOWN + 0xFFFF - PWM_4_IN_UP;
		}
		pulseState4 = !pulseState4;

		// Reverse polarity
		TIM_ICInit(TIM2, &TIM_CH4_ICInitStructure);
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
	return (uint16_t) (dutycycle * PWM_PeriodTicks);
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
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);

	/* TIM4 Time Base configuration */
	TIM_TimeBaseStructure4.TIM_Prescaler = PrescalerValue-1;
	TIM_TimeBaseStructure4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure4.TIM_Period = PWM_PeriodTicks-1;
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

static void PWM_In_Setup() {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	TIM_DeInit(TIM2);

	/* TIM2 clock setup */
	TIM2_Setup();

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
	pulseState1 = 0;
	TIM_CH1_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_CH1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH1_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH1_ICInitStructure);

	pulseState2 = 0;
	TIM_CH2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_CH2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH2_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH2_ICInitStructure);

	pulseState3 = 0;
	TIM_CH3_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_CH3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_CH3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_CH3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_CH3_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_CH3_ICInitStructure);

	pulseState4 = 0;
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

	RCC_GetClocksFreq(&RCC_Clocks);
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

/* GyroConfig
 * @brief  Configures the MEMS to gyroscope application
 * @param  None
 * @retval None
 */
static void GyroConfig(void)
{
	  L3GD20_InitTypeDef L3GD20_InitStructure;
	  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

	  /* Configure Mems L3GD20 */
	  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
	  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500;
	  L3GD20_Init(&L3GD20_InitStructure);

	  L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
	  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
static void GyroReadAngRate(float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);

  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity = L3G_Sensitivity_250dps;
    break;

  case 0x10:
    sensitivity = L3G_Sensitivity_500dps;
    break;

  case 0x20:
    sensitivity = L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i] = (float)RawData[i]/sensitivity;
  }
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
static void CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit rappresentation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
static void CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }

}

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
static void CompassReadMag(float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
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
