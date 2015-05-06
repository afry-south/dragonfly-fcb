#include "cmsis_os.h"

#include "stm32f3_discovery.h"

osThreadId LEDThread1Handle, LEDThread2Handle;

/**
  * @brief  Toggle LED3 and LED4 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument)
{
  uint32_t count = 0;
  (void) argument;

  for (;;)
  {
    count = osKernelSysTick() + 5000;

    /* Toggle LED3 every 200 ms for 5 s */
    while (count >= osKernelSysTick())
    {
      BSP_LED_Toggle(LED3);

      osDelay(200);
    }

    /* Turn off LED3 */
    BSP_LED_Off(LED3);

    /* Suspend Thread 1 */
    osThreadSuspend(NULL);

    count = osKernelSysTick() + 5000;

    /* Toggle LED3 every 400 ms for 5 s */
    while (count >= osKernelSysTick())
    {
      BSP_LED_Toggle(LED3);

      osDelay(400);
    }

    /* Resume Thread 2*/
    osThreadResume(LEDThread2Handle);

  }
}

/**
  * @brief  Toggle LED4 thread
  * @param  argument not used
  * @retval None
  */
static void LED_Thread2(void const *argument)
{
	uint32_t count = 0;
  (void) argument;

  for (;;)
  {
    count = osKernelSysTick() + 10000;

    /* Toggle LED4 every 500 ms for 10 s */
    while (count >= osKernelSysTick())
    {
      BSP_LED_Toggle(LED4);

      osDelay(500);
    }

    /* Turn off LED4 */
    BSP_LED_Off(LED4);

    /* Resume Thread 1 */
    osThreadResume(LEDThread1Handle);

    /* Suspend Thread 2 */
    osThreadSuspend(NULL);
  }
}


void run_dragon_threads(void) {
	  portTickType start = xTaskGetTickCount();
	  portTickType stop = 0;

	  volatile uint32_t my_count = 0xffffffff;

	  stop = xTaskGetTickCount();

	  /* Thread 1 definition */
	  osThreadDef(LED3, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	  /*  Thread 2 definition */
	  osThreadDef(LED4, LED_Thread2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	  /* Start thread 1 */
	  LEDThread1Handle = osThreadCreate(osThread(LED3), NULL);

	  /* Start thread 2 */
	  LEDThread2Handle = osThreadCreate(osThread(LED4), NULL);

	  /* Start scheduler    */
	  osKernelStart(NULL, NULL);

	  /* We should never get here as control is now taken by the scheduler */
	  for (;;);
}


