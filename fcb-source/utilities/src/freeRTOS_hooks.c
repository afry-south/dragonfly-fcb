/******************************************************************************
 * @file    freeRTOS_hooks.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-12
 * @brief   Module contains implements functions provided by FreeRTOS
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "fcb_error.h"

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	fcb_error();
}
