/******************************************************************************
 * @file    usb_cdc_cli.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-07-24
 * @brief   File contains functionality to use the USB CDC class with a
 *          Command Line Interface (CLI). Each command is associated with
 *          number of command parameters and a function which executes command
 *          activities. The CLI used is based on the FreeRTOS Plus CLI API.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_cdc_cli.h"
#include "receiver.h"
#include "fifo_buffer.h"
#include "common.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_DATA_TRANSFER_DELAY         2000 // [ms]

/* Private function prototypes -----------------------------------------------*/

/*
 * Function implements the "echo" command.
 */
static portBASE_TYPE CLIEchoCommandFunction(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "echo-data" command.
 */
static portBASE_TYPE CLIEchoDataCommandFunction(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "start-receiver-calibration" command.
 */
static portBASE_TYPE CLIStartReceiverCalibrationCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "stop-receiver-calibration" command.
 */
static portBASE_TYPE CLIStopReceiverCalibrationCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "get-receiver" command.
 */
static portBASE_TYPE CLIGetReceiverCommandFunction(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "get-receiver-calibration" command.
 */
static portBASE_TYPE CLIGetReceiverCalibrationCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "start-receiver-sampling" command.
 */
static portBASE_TYPE CLIStartReceiverSamplingCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "start-receiver-sampling" command.
 */
static portBASE_TYPE CLIStopReceiverSamplingCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/* Private variables ---------------------------------------------------------*/

/* Structure that defines the "echo" command line command. */
static const CLI_Command_Definition_t echoCommand =
		{ (const int8_t * const ) "echo",
				(const int8_t * const ) "\r\necho <param>:\r\n Echoes one parameter\r\n",
				CLIEchoCommandFunction, /* The function to run. */
				1 /* Number of parameters expected */
		};

/* Structure that defines the "echo-data" command line command. */
static const CLI_Command_Definition_t echoDataCommand =
		{ (const int8_t * const ) "echo-data",
				(const int8_t * const ) "\r\necho-data <param:data size>:\r\n Echoes input data with size specified by command parameter\r\n",
				CLIEchoDataCommandFunction, /* The function to run. */
				1 /* Number of parameters expected */
		};

/* Structure that defines the "start-receiver-calibration" command line command. */
static const CLI_Command_Definition_t startReceiverCalibrationCommand =
		{ (const int8_t * const ) "start-receiver-calibration",
				(const int8_t * const ) "\r\nstart-receiver-calibration:\r\n Starts the receiver calibration procedure\r\n",
				CLIStartReceiverCalibrationCommandFunction, /* The function to run. */
				0 /* Number of parameters expected */
		};

/* Structure that defines the "stop-receiver-calibration" command line command. */
static const CLI_Command_Definition_t stopReceiverCalibrationCommand =
		{ (const int8_t * const ) "stop-receiver-calibration",
				(const int8_t * const ) "\r\nstop-receiver-calibration:\r\n Stops the receiver calibration procedure\r\n",
				CLIStopReceiverCalibrationCommandFunction, /* The function to run. */
				0 /* Number of parameters expected */
		};

/* Structure that defines the "get-receiver" command line command. */
static const CLI_Command_Definition_t getReceiverCommand =
		{ (const int8_t * const ) "get-receiver",
				(const int8_t * const ) "\r\nget-receiver:\r\n Prints the current receiver value\r\n",
				CLIGetReceiverCommandFunction, /* The function to run. */
				0 /* Number of parameters expected */
		};

/* Structure that defines the "get-receiver-calibration" command line command. */
static const CLI_Command_Definition_t getReceiverCalibrationCommand =
		{ (const int8_t * const ) "get-receiver-calibration",
				(const int8_t * const ) "\r\nget-receiver-calibration:\r\n Prints the current receiver calibration values\r\n",
				CLIGetReceiverCalibrationCommandFunction, /* The function to run. */
				0 /* Number of parameters expected */
		};

/* Structure that defines the "start-receiver-sampling" command line command. */
static const CLI_Command_Definition_t startReceiverSamplingCommand =
		{ (const int8_t * const ) "start-receiver-sampling",
				(const int8_t * const ) "\r\nstart-receiver-sampling <sampletime> <sampleduration>:\r\n Prints receiver values once every <sampletime> ms for <sampleduration> s\r\n",
				CLIStartReceiverSamplingCommandFunction, /* The function to run. */
				2 /* Number of parameters expected */
		};

/* Structure that defines the "stop-receiver-sampling" command line command. */
static const CLI_Command_Definition_t stopReceiverSamplingCommand =
		{ (const int8_t * const ) "stop-receiver-sampling",
				(const int8_t * const ) "\r\nstop-receiver-sampling:\r\n Stops the printing of receiver sample values\r\n",
				CLIStopReceiverSamplingCommandFunction, /* The function to run. */
				2 /* Number of parameters expected */
		};

extern volatile FIFOBuffer_TypeDef USBCOMRxFIFOBuffer;
extern xSemaphoreHandle USBCOMRxDataSem;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Registers CLI commands
 * @param  None
 * @retval None
 */
void RegisterCLICommands(void) {
	/* Register all the command line commands defined immediately above. */
	FreeRTOS_CLIRegisterCommand(&echoCommand);
	FreeRTOS_CLIRegisterCommand(&echoDataCommand);

	FreeRTOS_CLIRegisterCommand(&startReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&stopReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&getReceiverCommand);
	FreeRTOS_CLIRegisterCommand(&getReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&startReceiverSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&stopReceiverSamplingCommand);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Implements the CLI command to echo one parameter
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIEchoCommandFunction(int8_t* pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been
		 entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Parameter received:\r\n",
				xWriteBufferLen);
	} else {
		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		strncpy((char*) pcWriteBuffer, (const char*) pcParameter,
				xParameterStringLength);

		strncat((char*) pcWriteBuffer, "\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == echoCommand.cExpectedNumberOfParameters) {
		/* If this is the last parameter then there are no more strings to return after this one. */
		xReturn = pdFALSE;
		lParameterNumber = 0;
	} else {
		/* There are more parameters to return after this one. */
		xReturn = pdTRUE;
		lParameterNumber++;
	}

	return xReturn;
}

/**
 * @brief  Implements the CLI command to echo a specific amount of data (size specified by parameter)
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIEchoDataCommandFunction(int8_t* pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t *pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been
		 entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Received data:\r\n", xWriteBufferLen);
	} else if (lParameterNumber == 1) {
		/* Obtain the parameter string. */
		pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		// Convert pcParameter string to equivalent int to get the length of the following data
		int dataLength = atoi((char*) pcParameter);

		/* Check that data length is a positive integer and within buffer bounds */
		if (!IS_POS(dataLength))
			return pdFALSE;

		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		int previousRxBufferCount = -1; // Init to -1 so that while loop is iterated at least once

		/* Check that new data is being received */
		while (previousRxBufferCount != USBCOMRxFIFOBuffer.count) {
			if (dataLength <= USBCOMRxFIFOBuffer.count) {
				/* Read out the data */
				ErrorStatus bufferStatus = SUCCESS;
				uint16_t i = 0;
				uint8_t getByte;
				while (bufferStatus == SUCCESS && i < dataLength
						&& i <= xWriteBufferLen) {
					bufferStatus = FIFOBufferGetByte(&USBCOMRxFIFOBuffer,
							&getByte);

					if (bufferStatus == SUCCESS)
						pcWriteBuffer[i] = getByte;

					i++;
				}

				/* If there is data left in the buffer, it is likely the next command to be handled by the CLI.
				 * Give semaphore to wake up USB RX thread to start handling this. */
				if (USBCOMRxFIFOBuffer.count > 0)
					xSemaphoreGive(USBCOMRxDataSem);

				previousRxBufferCount = USBCOMRxFIFOBuffer.count; // Set this to exit while loop
			} else {
				previousRxBufferCount = USBCOMRxFIFOBuffer.count;
				/*/ Pend on USBComRxDataSem for MAX_DATA_TRANSFER_DELAY while waiting for data to enter RX buffer */
				if (pdPASS
						!= xSemaphoreTake(USBCOMRxDataSem,
								MAX_DATA_TRANSFER_DELAY)) {
					memset(pcWriteBuffer, 0x00, xWriteBufferLen);
					strncpy((char*) pcWriteBuffer,
							"Data transmission timed out.\r\n",
							xWriteBufferLen);
				}
			}
		}
	} else {
		/* Return new line */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		strncpy((char*) pcWriteBuffer, "\r\n", xWriteBufferLen);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == echoDataCommand.cExpectedNumberOfParameters + 1) // Added +1 to output \r\n
			{
		/* If this is the last of the parameters then there are no more strings to return after this one. */
		xReturn = pdFALSE;
		lParameterNumber = 0;
	} else {
		/* There are more parameters/data to return after this one. */
		xReturn = pdTRUE;
		lParameterNumber++;
	}

	return xReturn;
}

/**
 * @brief  Implements the CLI command to start receiver calibration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStartReceiverCalibrationCommandFunction(
		int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.   */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Start the receiver calibration procedure */
	if (StartReceiverCalibration())
		strncpy((char*) pcWriteBuffer,
				"RC receiver calibration started. Please saturate all RC transmitter control sticks and toggle switches.\r\n",
				xWriteBufferLen);
	else
		strncpy((char*) pcWriteBuffer,
				"RC receiver calibration already in progress.\r\n",
				xWriteBufferLen);

	/* Return false to indicate command activity finished */
	return pdFALSE;
}

/**
 * @brief  Implements the CLI command to stop receiver calibration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStopReceiverCalibrationCommandFunction(
		int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Stop the receiver calibration procedure */
	ReceiverErrorStatus rcCalStatus = StopReceiverCalibration();
	strncpy((char*) pcWriteBuffer, "RC receiver calibration stopped.\r\n",
			xWriteBufferLen);
	if (rcCalStatus)
		strncat((char*) pcWriteBuffer,
				"RC receiver calibration successful.\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
	else
		strncat((char*) pcWriteBuffer,
				"RC receiver calibration failed or has not been started.\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

	/* Return false to indicate command activity finished */
	return pdFALSE;
}

/**
 * @brief  Implements the CLI command to print receiver values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetReceiverCommandFunction(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {

	static uint8_t currentChannelPrint = 0;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Get the current receiver values */
	switch (currentChannelPrint) {
	case 0:
		strncpy((char*) pcWriteBuffer, "Receiver channel values:\r\nStatus: ",
				xWriteBufferLen);
		if (IsReceiverActive())
			strncat((char*) pcWriteBuffer, "ACTIVE\r\n",
					xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		else
			strncat((char*) pcWriteBuffer, "INACTIVE\r\n",
					xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		break;
	case 1:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Throttle (0-65535): %u\r\n", GetThrottleReceiverChannel());
		break;
	case 2:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Aileron (-32768-32767): %d\r\n", GetAileronReceiverChannel());
		break;
	case 3:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Elevator (-32768-32767): %d\r\n",
				GetElevatorReceiverChannel());
		break;
	case 4:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Rudder (-32768-32767): %d\r\n", GetRudderReceiverChannel());
		break;
	case 5:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Gear (-32768-32767): %d\r\n", GetGearReceiverChannel());
		break;
	case 6:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Aux1 (-32768-32767): %d\r\n", GetAux1ReceiverChannel());
		break;
	default:
		strncpy((char*) pcWriteBuffer, "\r\n", xWriteBufferLen);
		/* Reset receiver print iteration number*/
		currentChannelPrint = 0;
		/* Return false to indicate command activity finished */
		return pdFALSE;
	}

	currentChannelPrint++;
	/* Return true to indicate command activity not yet completed */
	return pdTRUE;
}

/**
 * @brief  Implements the CLI command to print receiver calibration values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetReceiverCalibrationCommandFunction(
		int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {

	static uint8_t currentChannelPrint = 0;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Get the current receiver values */
	switch (currentChannelPrint) {
	case 0:
		strncpy((char*) pcWriteBuffer,
				"Receiver channel calibration values:\r\nNOTE: Values are specified in timer ticks.\r\n",
				xWriteBufferLen);
		break;
	case 1:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Throttle Max: %u\r\n",
				GetThrottleReceiverCalibrationMaxValue());
		break;
	case 2:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Throttle Min: %u\r\n",
				GetThrottleReceiverCalibrationMinValue());
		break;
	case 3:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Aileron Max: %u\r\n",
				GetAileronReceiverCalibrationMaxValue());
		break;
	case 4:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Aileron Min: %u\r\n",
				GetAileronReceiverCalibrationMinValue());
		break;
	case 5:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Elevator Max: %u\r\n",
				GetElevatorReceiverCalibrationMaxValue());
		break;
	case 6:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Elevator Min: %u\r\n",
				GetElevatorReceiverCalibrationMinValue());
		break;
	case 7:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Rudder Max: %u\r\n",
				GetRudderReceiverCalibrationMaxValue());
		break;
	case 8:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Rudder Min: %u\r\n",
				GetRudderReceiverCalibrationMinValue());
		break;
	case 9:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Gear Max: %u\r\n",
				GetGearReceiverCalibrationMaxValue());
		break;
	case 10:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Gear Min: %u\r\n",
				GetGearReceiverCalibrationMinValue());
		break;
	case 11:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Aux1 Max: %u\r\n",
				GetAux1ReceiverCalibrationMaxValue());
		break;
	case 12:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen, "Aux1 Min: %u\r\n",
				GetAux1ReceiverCalibrationMinValue());
		break;
	default:
		strncpy((char*) pcWriteBuffer, "\r\n", xWriteBufferLen);
		/* Reset receiver print iteration number*/
		currentChannelPrint = 0;
		/* Return false to indicate command activity finished */
		return pdFALSE;
	}

	currentChannelPrint++;
	/* Return true to indicate command activity not yet completed */
	return pdTRUE;
}

/**
 * @brief  Starts receiver sampling for a specified sample time and duration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStartReceiverSamplingCommandFunction(
		int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been
		 entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer,
				"Starting print sampling of receiver values...\r\n",
				xWriteBufferLen);
	} else {
		uint16_t receiverSampleTime;
		uint16_t receiverSampleDuration;

		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncpy((char*) pcWriteBuffer, "Sample time (ms): ", xWriteBufferLen);

		size_t paramMaxSize; // TODO make function for this comparison?
		if ((unsigned long) xParameterStringLength
				> xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		receiverSampleTime = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncat((char*) pcWriteBuffer, "Duration (s): ",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		if ((unsigned long) xParameterStringLength
				> xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		receiverSampleDuration = atoi((char*) pcParameter); // TODO sanity check?

		StartReceiverSamplingTask(receiverSampleTime, receiverSampleDuration);
	}

	/* Update return value and parameter index */
	if (lParameterNumber
			== startReceiverSamplingCommand.cExpectedNumberOfParameters) {
		/* If this is the last parameter then there are no more strings to return after this one. */
		xReturn = pdFALSE;
		lParameterNumber = 0;
	} else {
		/* There are more parameters to return after this one. */
		xReturn = pdTRUE;
		lParameterNumber++;
	}

	return xReturn;
}

/**
 * @brief  Stops printing of receiver sample values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStopReceiverSamplingCommandFunction(
		int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer,
			"Stopping printing of receiver sample values...\r\n",
			xWriteBufferLen);

	/* Stop the receiver printing task */
	StopReceiverSamplingTask();

	return pdFALSE;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
