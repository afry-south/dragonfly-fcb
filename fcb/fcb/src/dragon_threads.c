#include "stm32f3_discovery.h"

#include "FreeRTOS.h"
#include "task.h"

xTaskHandle hLed1 = NULL;
xTaskHandle hLed2 = NULL;

/**
  * @brief  Toggle LED3 and LED4 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument)
{
  (void) argument;

  portTickType count = 0;

  for (;;)
  {
    count = xTaskGetTickCount() + 5000;

    /* Toggle LED3 every 200 ms for 5 s */

   	while (count >= xTaskGetTickCount())
   	{
      BSP_LED_Toggle(LED3);

      vTaskDelay(200 / portTICK_RATE_MS);          /* Minimum delay = 1 tick */
    }

    /* Turn off LED3 */
    BSP_LED_Off(LED3);

    /* Suspend Thread 1 (own thread) */

    vTaskSuspend(NULL);

    count = xTaskGetTickCount() + 5000;

    /* Toggle LED3 every 400 ms for 5 s */
   	while (count >= xTaskGetTickCount()) {
      BSP_LED_Toggle(LED3);

      vTaskDelay(400 / portTICK_RATE_MS); /* Minimum delay = 1 tick */
    }

    /* Resume Thread 2*/
   	vTaskResume(hLed2);
  }
}

/**
  * @brief  Toggle LED4 thread
  * @param  argument not used
  * @retval None
  */
static void LED_Thread2(void const *argument)
{
	portTickType count = 0;
  (void) argument;

  for (;;)
  {
	  count = xTaskGetTickCount() + 10000;

    /* Toggle LED4 every 500 ms for 10 s */

    while (count >=	xTaskGetTickCount()) {
      BSP_LED_Toggle(LED4);
      vTaskDelay(500 / portTICK_RATE_MS);
    }

    /* Turn off LED4 */
    BSP_LED_Off(LED4);

    /* Resume Thread 1 */
   	vTaskResume(hLed1);

    /* Suspend Thread 2 */
    vTaskSuspend(NULL);
  }
}


void run_dragon_threads(void) {
	  portBASE_TYPE retVal = 0;

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

	  /* Start scheduler
	   *
	   * since we use heap1.c, we must create all tasks and queues before the OS kernel
	   * is started according to ST UM1722 manual section 1.6.
	   */
	  vTaskStartScheduler();

	  /* We should never get here as control is now taken by the scheduler */
	  for (;;);
}


