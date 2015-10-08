/**
 ******************************************************************************
 * @file    task_status.c
 * @brief   Task status module responsible for handling the timer needed for
 * 			vTaskGetRunTimeStats() function.
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOSConfig.h"
#include "stm32f3xx_hal.h"
#include "trace.h"
#include "task_status.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PERIOD	0xFFFF
#define INIT_ERROR	0
#define	START_ERROR	1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile unsigned long ulHighFrequencyTimerTicks;
static uint32_t TaskStatusTimerPeriodCount = 0;
static uint32_t uwPrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(uint8_t errorType);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  This function sets up TASK STATUS timer, defined in task_status.h
  * @param  None
  * @retval None
  */
void InitMonitoring(void){

	/* Compute the prescaler value to have TIM counter clock equal to 10 KHz */
	  uwPrescalerValue = (uint32_t) (SystemCoreClock / 10000) - 1;

	  /* Set TIMx instance */
	  TaskStatusTimHandle.Instance = TASK_STATUS_TIM;

	  /* Initialize TIMx peripheral */
	  TaskStatusTimHandle.Init.Period = PERIOD;//10000 - 1;
	  TaskStatusTimHandle.Init.Prescaler = uwPrescalerValue;
	  TaskStatusTimHandle.Init.ClockDivision = 0;
	  TaskStatusTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

	  if(HAL_TIM_Base_Init(&TaskStatusTimHandle) != HAL_OK)
	  {
	    Error_Handler(INIT_ERROR);
	  }

	   /* Start Channel1 */
	   if(HAL_TIM_Base_Start_IT(&TaskStatusTimHandle) != HAL_OK)
	   {
	     Error_Handler(START_ERROR);
	   }

}

/**
  * @brief  This function is called from the callback function in stm32f3xx_hal_callback.c
  * 		and it increases the period counter for the timer.
  * @param  None
  * @retval None
  */
void IncreaseTaskStatusTimerPeriodCount(void){
	TaskStatusTimerPeriodCount++;
}

/**
  * @brief  This function returns the period count. Used for debugging.
  * @param  None
  * @retval TaskStatusTimerPeriodCount: The number of periods elapsed.
  */
uint32_t GetTaskStatusTimerPeriodCount(void){
	return TaskStatusTimerPeriodCount;
}

/**
  * @brief  This function configures and resets the TIMx timer and the 32bit timer
  * 		for the task status feature.
  * @param  None
  * @retval None
  */
void configureTimerForRunTimeStats(void)
{
	//GetTimer
	TaskStatusTimHandle.State = HAL_TIM_STATE_RESET;
    ulHighFrequencyTimerTicks = 0UL;
}

/**
  * @brief  This function gets the 32bit timer value for the task status feature.
  * @param  None
  * @retval ulHighFrequencyTimerTicks: The 32bit timer value.
  */
unsigned getRunTimeCounterValue(void)
{
	ulHighFrequencyTimerTicks = __HAL_TIM_GetCounter(&TaskStatusTimHandle) + PERIOD*TaskStatusTimerPeriodCount;
    return ulHighFrequencyTimerTicks;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(uint8_t errorType)
{
	switch(errorType){

	case 0:
		trace_printf("TASK STATUS Error_Handler: INIT ERROR");
		break;

	case 1:
		trace_printf("TASK STATUS Error_Handler: START ERROR");
		break;
	}

	while(1)
	{
	}
}
