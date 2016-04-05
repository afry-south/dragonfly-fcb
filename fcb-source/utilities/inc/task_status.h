/**
 ******************************************************************************
 * @file    task_status.h
 * @brief   Header for task_status.c module
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TASK_STATUS_H
#define TASK_STATUS_H

/* Exported constants --------------------------------------------------------*/
#define TASK_STATUS_TIM	                    TIM15
#define TASK_STATUS_TIM_CLK_ENABLE()        __TIM15_CLK_ENABLE()
#define	TASK_STATUS_TIM_CLK_DISABLE()       __TIM15_CLK_DISABLE()
#define TASK_STATUS_TIM_IRQn	            TIM15_IRQn
#define TASK_STATUS_TIM_IRQHandler	        TIM15_IRQHandler
#define TASK_STATUS_TIM_IRQ_PREEMPT_PRIO    0
#define TASK_STATUS_TIM_IRQ_SUB_PRIO        0

/* Exported variables --------------------------------------------------------*/
TIM_HandleTypeDef    TaskStatusTimHandle;

/* Exported functions ------------------------------------------------------- */
void InitMonitoring(void);

unsigned getRunTimeCounterValue(void);

void configureTimerForRunTimeStats(void);

void IncreaseTaskStatusTimerPeriodCount(void);

uint32_t GetTaskStatusTimerPeriodCount(void);

#endif
