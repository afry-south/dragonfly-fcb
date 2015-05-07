#include "stm32f3_discovery.h"

//#define USE_CMSIS_API

#ifndef USE_CMSIS_API
#include "FreeRTOS.h"
#include "task.h"

xTaskHandle hLed1 = NULL;
xTaskHandle hLed2 = NULL;

#else
#include "cmsis_os.h" /* todo delete when translated to FreeRTOS */

osThreadId LEDThread1Handle, LEDThread2Handle;

#endif

/**
  * @brief  Toggle LED3 and LED4 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument)
{
  (void) argument;

#ifdef USE_CMSIS_API
  uint32_t count = 0;
#else
  portTickType count = 0;
#endif

  for (;;)
  {
#ifdef USE_CMSIS_API
    count = osKernelSysTick() + 5000;
#else
    count = xTaskGetTickCount() + 5000;
#endif


    /* Toggle LED3 every 200 ms for 5 s */

#ifdef USE_CMSIS_API
    while (count >= osKernelSysTick())
    {
#else
   	while (count >= xTaskGetTickCount())
   	{
#endif
      BSP_LED_Toggle(LED3);

#ifdef USE_CMSIS_API
      osDelay(200);
#else
      vTaskDelay(200 / portTICK_RATE_MS);          /* Minimum delay = 1 tick */
#endif
    }

    /* Turn off LED3 */
    BSP_LED_Off(LED3);

    /* Suspend Thread 1 (own thread) */

#ifdef USE_CMSIS_API
    osThreadSuspend(NULL);
#else
    vTaskSuspend(NULL);
#endif

#ifdef USE_CMSIS_API
    count = osKernelSysTick() + 5000;
#else
    count = xTaskGetTickCount() + 5000;
#endif

    /* Toggle LED3 every 400 ms for 5 s */
#ifdef USE_CMSIS_API
    while (count >= osKernelSysTick()) {
#else
   	while (count >= xTaskGetTickCount()) {
#endif
      BSP_LED_Toggle(LED3);

#ifdef USE_CMSIS_API
      osDelay(400);
#else
      vTaskDelay(400 / portTICK_RATE_MS); /* Minimum delay = 1 tick */
#endif
    }

    /* Resume Thread 2 & suspend Thread 1*/
#ifdef USE_CMSIS_API
   	osThreadResume(LEDThread2Handle);
#else
   	vTaskResume(hLed2);
/*    vTaskSuspend(NULL); */
#endif

  }
}

/**
  * @brief  Toggle LED4 thread
  * @param  argument not used
  * @retval None
  */
static void LED_Thread2(void const *argument)
{
#ifdef USE_CMSIS_API
	uint32_t count = 0;
#else
	portTickType count = 0;
#endif
  (void) argument;

  for (;;)
  {
#ifdef USE_CMSIS_API
	  count = osKernelSysTick() + 10000;
#else
	  count = xTaskGetTickCount() + 10000;
#endif

    /* Toggle LED4 every 500 ms for 10 s */

#ifdef USE_CMSIS_API
    while (count >= osKernelSysTick()) {
#else
    while (count >=	xTaskGetTickCount()) {
#endif
      BSP_LED_Toggle(LED4);

#ifdef USE_CMSIS_API
      osDelay(500);
#else
      vTaskDelay(500 / portTICK_RATE_MS);
#endif
    }

    /* Turn off LED4 */
    BSP_LED_Off(LED4);

    /* Resume Thread 1 */
#ifdef USE_CMSIS_API
   	osThreadResume(LEDThread1Handle);
#else
   	vTaskResume(hLed1);
#endif

    /* Suspend Thread 2 */
#ifdef USE_CMSIS_API
    osThreadSuspend(NULL);
#else
    vTaskSuspend(NULL);
#endif
  }
}


void run_dragon_threads(void) {
#ifndef USE_CMSIS_API
	  portBASE_TYPE retVal = 0;
#endif

#ifdef USE_CMSIS_API
	  /* Thread 1 definition */
	  osThreadDef(LED3, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	  /*  Thread 2 definition */
	  osThreadDef(LED4, LED_Thread2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	  /* Start thread 1 */
	  LEDThread1Handle = osThreadCreate(osThread(LED3), NULL);
	  /* Start thread 2 */
	  LEDThread2Handle = osThreadCreate(osThread(LED4), NULL);
#else
	  if (pdPASS != (retVal =
			  	 	 xTaskCreate((pdTASK_CODE)LED_Thread1,
			  	 			 (signed portCHAR*)"tLed1",
			  	 			 configMINIMAL_STACK_SIZE,
			  	 			 NULL /* parameter */,
			  	 			 3 /* priority */,
			  	 			 &hLed1))) {
		  // ERROR here.
	  }

	  if (pdPASS != (retVal =
			  	 	 xTaskCreate((pdTASK_CODE)LED_Thread2,
			  	 			 (signed portCHAR*)"tLed2",
			  	 			 configMINIMAL_STACK_SIZE,
			  	 			 NULL /* parameter */,
			  	 			 3 /* priority */,
			  	 			 &hLed2))) {
		  // ERROR here.
	  }

#endif


	  /* Start scheduler
	   *
	   * since we use heap1.c, we must create all tasks and queues before the OS kernel
	   * is started according to ST UM1722 manual section 1.6.
	   */
#ifdef USE_CMSIS_API
	  osKernelStart(NULL, NULL);
#else
	  vTaskStartScheduler();
#endif

	  /* We should never get here as control is now taken by the scheduler */
	  for (;;);
}


