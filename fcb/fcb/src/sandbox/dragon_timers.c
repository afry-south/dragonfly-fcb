#include "stm32f3xx.h"
#include "stm32f3_discovery.h"
#include "stm32f3xx_hal_gpio.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

static GPIO_InitTypeDef  GPIO_InitStruct;


static void dragon_timer_action(xTimerHandle xTimer ); /* tmrTIMER_CALLBACK */
static xTimerHandle xDragonTimer = NULL;
enum { TIMER_INTERVAL_MS = 1 };
enum { DRAGON_TIMER_ID = 242 };

void dragon_timers(void) {
	xSemaphoreHandle sNeverExit = xSemaphoreCreateBinary();
	BSP_LED_On(LED3);

	/* configure pin PD9 for GPIO toggling */

	 __GPIOD_CLK_ENABLE();

	  /* -2- Configure PE.8 to PE.15 IOs in output push-pull mode to drive external LEDs */
	 GPIO_InitStruct.Pin = (GPIO_PIN_9);
	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	if (0 == (xDragonTimer = xTimerCreate((const signed char*)"tmrDragon",
			TIMER_INTERVAL_MS / portTICK_RATE_MS,
			pdTRUE, /* uxAutoReload */
			(void*)DRAGON_TIMER_ID, /* pvTimerID */
			dragon_timer_action))) {
		BSP_LED_On(LED10);
		return;
	}


	if (pdFAIL == xTimerStart(xDragonTimer, 0)) {
		BSP_LED_On(LED10);
		return;
	}

	/* Start scheduler
	 *
	 * since we use heap1.c, we must create all tasks and queues before the OS kernel
	 * is started according to ST UM1722 manual section 1.6.
	 */
	 vTaskStartScheduler();

	 /* We should never get here as control is now taken by the scheduler */
	 if (pdTRUE != xSemaphoreTake(sNeverExit, portMAX_DELAY)) {
		 // ERROR here
	 }
}


static void dragon_timer_action(xTimerHandle xTimer ) {
	static uint8_t flipflop = 0;

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);

	if (flipflop) {
		BSP_LED_Off(LED6);
		flipflop = 0;
	} else {
		BSP_LED_On(LED6);
		flipflop = 1;
	}
}
