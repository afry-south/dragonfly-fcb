/******************************************************************************
 * @file    freeRTOS_hooks.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-12
 * @brief   Module contains implements functions provided by FreeRTOS
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "fcb_error.h"

#include "usbd_cdc_if.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	HOOK_STRING_SIZE	64

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  FreeRTOS application/task stack overflow hook
 * @param  xTask : Task handle
 * @param  pcTaskName : Task name string
 * @retval None
 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	(void) xTask;
	char hookString[HOOK_STRING_SIZE];
	memset(hookString, 0x00, sizeof(hookString));
	USBComSendString("FreeRTOS stack overflow\n");
	strncpy(hookString, "Task: ", HOOK_STRING_SIZE);
	strncat(hookString, (char*)pcTaskName, HOOK_STRING_SIZE - strlen(hookString) - 1);
	strncat(hookString, "\r\n", HOOK_STRING_SIZE - strlen(hookString) - 1);
	USBComSendString(hookString);
	ErrorHandler();
}

/*
 * @brief  FreeRTOS application/task malloc failed hook
 * @param  xTask : Task handle
 * @param  pcTaskName : Task name string
 * @retval None
 */
void vApplicationMallocFailedHook(xTaskHandle xTask, signed char *pcTaskName) {
	(void) xTask;
	char hookString[HOOK_STRING_SIZE];
	memset(hookString, 0x00, sizeof(hookString));
	USBComSendString("FreeRTOS malloc failed\n");
	strncat(hookString, (char*)pcTaskName, HOOK_STRING_SIZE - strlen(hookString) - 1);
	strncat(hookString, "\r\n", HOOK_STRING_SIZE - strlen(hookString) - 1);
	USBComSendString(hookString);
	ErrorHandler();
}
