/******************************************************************************
 * @brief   Header file USB CDC class with Command Line Interface (CLI)
 *          functions.
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_CLI_H
#define __COM_CLI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define MAX_CLI_COMMAND_SIZE    256
#define MAX_CLI_OUTPUT_SIZE     256

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void RegisterCLICommands(void);
portBASE_TYPE CLIParser(uint8_t* cliIn, uint8_t* cliOut, uint16_t* len);
void CreateCLISemaphores(void);
uint32_t TakeCLIMutex(void);
void GiveCLIMutex(void);

#endif /* __USB_COM_CLI_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
