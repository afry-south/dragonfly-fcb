/******************************************************************************
 * @file    usb_com_cli.h
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

enum ProtoMessageTypeEnum {
	RC_VALUES_MSG_ENUM = 1,
	MOTOR_VALUES_MSG_ENUM,
	SENSOR_SAMPLES_MSG_ENUM,
	FLIGHT_STATE_MSG_ENUM,
	PID_CTRLPARAMS_MSG_ENUM,
	CTRL_REFSIGNALS_MSG_ENUM,
	SIMULATED_STATES_ENUM
};

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
