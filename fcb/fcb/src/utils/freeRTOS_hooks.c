#include "FreeRTOS.h"
#include "task.h"
#include "df_error.h"

void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName ) {
	df_error();
}
