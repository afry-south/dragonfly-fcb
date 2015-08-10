#include "FreeRTOS.h"
#include "task.h"
#include "fcb_error.h"

void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName ) {
	fcb_error();
}
