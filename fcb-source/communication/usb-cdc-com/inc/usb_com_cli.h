/******************************************************************************
 * @file    usb_com_cli.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-07-24
 * @brief   Header file USB CDC class with Command Line Interface (CLI)
 *          functions.
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_COM_CLI_H
#define __USB_COM_CLI_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define MAX_CLI_COMMAND_SIZE    256
#define MAX_CLI_OUTPUT_SIZE     256

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void RegisterCLICommands(void);

#endif /* __USB_COM_CLI_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
