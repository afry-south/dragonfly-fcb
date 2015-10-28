/******************************************************************************
 * @file    usb_com_cli.c
 * @brief   File contains functionality to use the USB CDC class with a
 *          Command Line Interface (CLI). Each command is associated with
 *          number of command parameters and a function which executes command
 *          activities. The CLI used is based on the FreeRTOS Plus CLI API.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_com_cli.h"

#include "main.h"
#include "dragonfly_fcb.pb.h"
#include "receiver.h"
#include "motor_control.h"
#include "flight_control.h"
#include "fifo_buffer.h"
#include "common.h"
#include "fcb_gyroscope.h"
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensors.h"
#include "state_estimation.h"

#include <stdlib.h>
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
static portBASE_TYPE CLIEcho(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "echo-data" command.
 */
static portBASE_TYPE CLIEchoData(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "start-receiver-calibration" command.
 */
static portBASE_TYPE CLIStartReceiverCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "stop-receiver-calibration" command.
 */
static portBASE_TYPE CLIStopReceiverCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "reset-receiver-calibration" command.
 */
static portBASE_TYPE CLIResetReceiverCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "get-receiver" command.
 */
static portBASE_TYPE CLIGetReceiver(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "get-receiver-calibration" command.
 */
static portBASE_TYPE CLIGetReceiverCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "start-receiver-sampling" command.
 */
static portBASE_TYPE CLIStartReceiverSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "stop-receiver-sampling" command.
 */
static portBASE_TYPE CLIStopReceiverSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "get-sensors" command.
 */
static portBASE_TYPE CLIGetSensors(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "start-sensor-sampling" command.
 */
static portBASE_TYPE CLIStartSensorSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString);

/*
 * Function implements the "stop-sensor-sampling" command.
 */
static portBASE_TYPE CLIStopSensorSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);


/* Fcn implements "start-accmagmtr-calibration" CLI command. */
static portBASE_TYPE CLIStartAccMagMtrCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "get-motors" command.
 */
static portBASE_TYPE CLIGetMotorValues(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "start-motor-sampling" command.
 */
static portBASE_TYPE CLIStartMotorSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "stop-motor-sampling" command.
 */
static portBASE_TYPE CLIStopMotorSampling(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "about" command.
 */
static portBASE_TYPE CLIAbout(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "systime" command.
 */
static portBASE_TYPE CLISysTime(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/*
 * Function implements the "get-flight-mode" command.
 */
static portBASE_TYPE CLIGetFlightMode(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/*
 * Function implements the "get-ref-signals" command.
 */
static portBASE_TYPE CLIGetRefSignals(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/*
 * Function implements the "task-status" command.
 */
static portBASE_TYPE CLITaskStatus(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/*
 * Function implements the "get-states" command.
 */
static portBASE_TYPE CLIGetStateValues(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/*
 * Function implements the "start-state-sampling" command.
 */
static portBASE_TYPE CLIStartStateSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/*
 * Function implements the "stop-state-sampling" command.
 */
static portBASE_TYPE CLIStopStateSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString);

/* Private variables ---------------------------------------------------------*/

/* Structure that defines the "echo" command line command. */
static const CLI_Command_Definition_t echoCommand = { (const int8_t * const ) "echo",
		(const int8_t * const ) "\r\necho <param>:\r\n Echoes one parameter\r\n", CLIEcho, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "echo-data" command line command. */
static const CLI_Command_Definition_t echoDataCommand = { (const int8_t * const ) "echo-data",
		(const int8_t * const ) "\r\necho-data <param:data size>:\r\n Echoes input data with size specified by command parameter\r\n",
		CLIEchoData, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "start-receiver-calibration" command line command. */
static const CLI_Command_Definition_t startReceiverCalibrationCommand = { (const int8_t * const ) "start-receiver-calibration",
		(const int8_t * const ) "\r\nstart-receiver-calibration <encoding>:\r\n Starts receiver calibration procedure with values printing with <encoding>  (n=none, p=proto)\r\n",
		CLIStartReceiverCalibration, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "stop-receiver-calibration" command line command. */
static const CLI_Command_Definition_t stopReceiverCalibrationCommand = { (const int8_t * const ) "stop-receiver-calibration",
		(const int8_t * const ) "\r\nstop-receiver-calibration:\r\n Stops receiver calibration procedure\r\n",
		CLIStopReceiverCalibration, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "reset-receiver-calibration" command line command. */
static const CLI_Command_Definition_t resetReceiverCalibrationCommand = { (const int8_t * const ) "reset-receiver-calibration",
		(const int8_t * const ) "\r\nreset-receiver-calibration:\r\n Resets receiver calibration to default values\r\n",
		CLIResetReceiverCalibration, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "get-receiver" command line command. */
static const CLI_Command_Definition_t getReceiverCommand = { (const int8_t * const ) "get-receiver",
		(const int8_t * const ) "\r\nget-receiver <encoding>:\r\n Returns current receiver values with <encoding> (n=none, p=proto)\r\n",
		CLIGetReceiver, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "get-receiver-calibration" command line command. */
static const CLI_Command_Definition_t getReceiverCalibrationCommand = { (const int8_t * const ) "get-receiver-calibration",
		(const int8_t * const ) "\r\nget-receiver-calibration:\r\n Prints current receiver calibration values\r\n",
		CLIGetReceiverCalibration, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "start-receiver-sampling" command line command. */
static const CLI_Command_Definition_t startReceiverSamplingCommand = { (const int8_t * const ) "start-receiver-sampling",
		(const int8_t * const ) "\r\nstart-receiver-sampling <sampletime> <sampleduration> <encoding>:\r\n Prints receiver values once every <sampletime> ms for <sampleduration> s with <encoding> (n=none, p=proto)\r\n",
		CLIStartReceiverSampling, /* The function to run. */
		3 /* Number of parameters expected */
};

/* Structure that defines the "stop-receiver-sampling" command line command. */
static const CLI_Command_Definition_t stopReceiverSamplingCommand = { (const int8_t * const ) "stop-receiver-sampling",
		(const int8_t * const ) "\r\nstop-receiver-sampling:\r\n Stops printing of receiver sample values\r\n",
		CLIStopReceiverSampling, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "get-sensors" command line command. */
static const CLI_Command_Definition_t getSensorsCommand = { (const int8_t * const ) "get-sensors",
		(const int8_t * const ) "\r\nget-sensors: <encoding>\r\n Prints last read sensor values with <encoding> (n=none, p=proto)\r\n",
		CLIGetSensors, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "start-sensor-sampling" command line command. */
static const CLI_Command_Definition_t startSensorSamplingCommand = { (const int8_t * const ) "start-sensor-sampling",
		(const int8_t * const ) "\r\nstart-sensor-sampling <sampletime> <sampleduration> <encoding>:\r\n Prints sensor values once every <sampletime> ms for <sampleduration> s with <encoding> (n=none, p=proto, c=calibration)\r\n",
		CLIStartSensorSampling, /* The function to run. */
		3 /* Number of parameters expected */
};

/* Structure that defines the "stop-sensor-sampling" command line command. */
static const CLI_Command_Definition_t stopSensorSamplingCommand = { (const int8_t * const ) "stop-sensor-sampling",
		(const int8_t * const ) "\r\nstop-sensor-sampling:\r\n Stops printing of sensor sample values\r\n",
		CLIStopSensorSampling, /* The function to run. */
		0 /* Number of parameters expected */
};

static const CLI_Command_Definition_t startAccMagMtrCalibration = { (const int8_t * const ) "start-accmagmtr-calibration",
  (const int8_t * const) "\r\n start-accmagmtr-calibration:\r\n Starts collection of 6 magmtr values via 6 USER btn presses\r\n"
                         "NOTE: use with SciLab Gauss-Newton scripts (todo Issue2)\r\n",
  CLIStartAccMagMtrCalibration, /* the fcn to run */
  0 /* nbr of expected parameters */
};

/* Structure that defines the "get-motors" command line command. */
static const CLI_Command_Definition_t getMotorsCommand = { (const int8_t * const ) "get-motors",
		(const int8_t * const ) "\r\nget-motors <encoding>:\r\n Prints motor control values with <encoding> (n=none, p=proto)\r\n",
		CLIGetMotorValues, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "start-motor-sampling" command line command. */
static const CLI_Command_Definition_t startMotorSamplingCommand = { (const int8_t * const ) "start-motor-sampling",
		(const int8_t * const ) "\r\nstart-motor-sampling <sampletime> <sampleduration> <encoding>:\r\n Prints motor control values once every <sampletime> ms for <sampleduration> s with <encoding> (n=none, p=proto)\r\n",
		CLIStartMotorSampling, /* The function to run. */
		3 /* Number of parameters expected */
};

/* Structure that defines the "stop-motor-sampling" command line command. */
static const CLI_Command_Definition_t stopMotorSamplingCommand = { (const int8_t * const ) "stop-motor-sampling",
		(const int8_t * const ) "\r\nstop-motor-sampling:\r\n Stops printing of motor control sample values\r\n",
		CLIStopMotorSampling, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "about" command line command. */
static const CLI_Command_Definition_t aboutCommand = { (const int8_t * const ) "about",
		(const int8_t * const ) "\r\nabout:\r\n Prints system info\r\n",
		CLIAbout, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "systime" command line command. */
static const CLI_Command_Definition_t systimeCommand = { (const int8_t * const ) "systime",
		(const int8_t * const ) "\r\nsystime:\r\n Prints system time in ms\r\n",
		CLISysTime, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "get-flight-mode" command line command. */
static const CLI_Command_Definition_t getFlightModeCommand = { (const int8_t * const ) "get-flight-mode",
		(const int8_t * const ) "\r\nget-flight-mode:\r\n Prints the current flight mode\r\n",
		CLIGetFlightMode, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "get-ref-signals" command line command. */
static const CLI_Command_Definition_t getRefSignalsCommand = { (const int8_t * const ) "get-ref-signals",
		(const int8_t * const ) "\r\nget-ref-signals:\r\n Prints the current reference signal\r\n",
		CLIGetRefSignals, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "task-status" command line command. */
static const CLI_Command_Definition_t taskStatusCommand = { (const int8_t * const ) "task-status",
		(const int8_t * const ) "\r\ntask-status:\r\n Prints task status\r\n",
		CLITaskStatus, /* The function to run. */
		0 /* Number of parameters expected */
};

/* Structure that defines the "get-states" command line command. */
static const CLI_Command_Definition_t getStatesCommand = { (const int8_t * const ) "get-states",
		(const int8_t * const ) "\r\nget-states <encoding>:\r\n Prints state values with <encoding> (n=none, p=proto)\r\n",
		CLIGetStateValues, /* The function to run. */
		1 /* Number of parameters expected */
};

/* Structure that defines the "start-motor-sampling" command line command. */
static const CLI_Command_Definition_t startStateSamplingCommand = { (const int8_t * const ) "start-state-sampling",
		(const int8_t * const ) "\r\nstart-state-sampling <sampletime> <sampleduration> <encoding>:\r\n"
		" Prints state values once every <sampletime> ms for <sampleduration> s with <encoding> (n=none, p=proto, c=calibration)\r\n",
		CLIStartStateSampling, /* The function to run. */
		3 /* Number of parameters expected */
};

/* Structure that defines the "stop-motor-sampling" command line command. */
static const CLI_Command_Definition_t stopStateSamplingCommand = { (const int8_t * const ) "stop-state-sampling",
		(const int8_t * const ) "\r\nstop-state-sampling:\r\n Stops printing of state sample values\r\n",
		CLIStopStateSampling, /* The function to run. */
		0 /* Number of parameters expected */
};

extern volatile FIFOBuffer_TypeDef USBCOMRxFIFOBuffer;
extern xSemaphoreHandle USBCOMRxDataSem;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Registers CLI commands
 * @param  None
 * @retval None
 * @note   Too many CLI commands may cause problems with the "help" command if the UsbTxQueue is not large enough
 */
void RegisterCLICommands(void) {
	/* Register all the command line commands defined immediately above. */

	/* Sample CLI commands */
	FreeRTOS_CLIRegisterCommand(&echoCommand);
	FreeRTOS_CLIRegisterCommand(&echoDataCommand);

	/* RC receiver CLI commands*/
	FreeRTOS_CLIRegisterCommand(&startReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&stopReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&resetReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&getReceiverCommand);
	FreeRTOS_CLIRegisterCommand(&getReceiverCalibrationCommand);
	FreeRTOS_CLIRegisterCommand(&startReceiverSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&stopReceiverSamplingCommand);

	/* Sensor CLI commands */
	FreeRTOS_CLIRegisterCommand(&getSensorsCommand);
	FreeRTOS_CLIRegisterCommand(&startSensorSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&stopSensorSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&startAccMagMtrCalibration);

	/* Motors CLI commands */
	FreeRTOS_CLIRegisterCommand(&getMotorsCommand);
	FreeRTOS_CLIRegisterCommand(&startMotorSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&stopMotorSamplingCommand);

	/* System info CLI commands */
	FreeRTOS_CLIRegisterCommand(&aboutCommand);
	FreeRTOS_CLIRegisterCommand(&systimeCommand);
	FreeRTOS_CLIRegisterCommand(&taskStatusCommand);

	/* Flight control CLI commands */
	FreeRTOS_CLIRegisterCommand(&getFlightModeCommand);
	FreeRTOS_CLIRegisterCommand(&getRefSignalsCommand);
	FreeRTOS_CLIRegisterCommand(&getStatesCommand);
	FreeRTOS_CLIRegisterCommand(&startStateSamplingCommand);
	FreeRTOS_CLIRegisterCommand(&stopStateSamplingCommand);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Implements the CLI command to echo one parameter
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIEcho(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been
		 entered just a header string is returned. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		strncpy((char*) pcWriteBuffer, "Parameter received:\r\n", xWriteBufferLen);
	} else {
		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		strncpy((char*) pcWriteBuffer, (const char*) pcParameter, xParameterStringLength);
		strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
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
static portBASE_TYPE CLIEchoData(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
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
				while (bufferStatus == SUCCESS && i < dataLength && i <= xWriteBufferLen) {
					bufferStatus = FIFOBufferGetByte(&USBCOMRxFIFOBuffer, &getByte);

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
				if (pdPASS != xSemaphoreTake(USBCOMRxDataSem, MAX_DATA_TRANSFER_DELAY)) {
					memset(pcWriteBuffer, 0x00, xWriteBufferLen);
					strncpy((char*) pcWriteBuffer, "Data transmission timed out.\r\n", xWriteBufferLen);
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
static portBASE_TYPE CLIStartReceiverCalibration(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength;
	portBASE_TYPE lParameterNumber = 0;

	/* Empty pcWriteBuffer so no strange output is sent as command response */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	lParameterNumber++;

	/* Obtain the parameter string. */
	pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
			lParameterNumber, /* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
	);

	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	/* Get the current receiver values */
	if(pcParameter[0] == 'n')
		SetReceiverPrintSamplingSerialization(NO_SERIALIZATION);
	else if(pcParameter[0] == 'p')
		SetReceiverPrintSamplingSerialization(PROTOBUFFER_SERIALIZATION);

	/* Start the receiver calibration procedure */
	if (StartReceiverCalibration())
		strncpy((char*) pcWriteBuffer,
				"RC receiver calibration started. Please saturate all RC transmitter control sticks and toggle switches.\r\n",
				xWriteBufferLen);
	else
		strncpy((char*) pcWriteBuffer, "RC receiver calibration could not be started.\r\n", xWriteBufferLen);

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
static portBASE_TYPE CLIStopReceiverCalibration(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Stop the receiver calibration procedure */
	ReceiverErrorStatus rcCalStatus = StopReceiverCalibration();
	strncpy((char*) pcWriteBuffer, "RC receiver calibration stopped\r\n", xWriteBufferLen);
	if (rcCalStatus)
		strncat((char*) pcWriteBuffer, "RC receiver calibration successful\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
	else
		strncat((char*) pcWriteBuffer, "RC receiver calibration failed\r\n",
				xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

	/* Return false to indicate command activity finished */
	return pdFALSE;
}

/**
 * @brief  Implements the CLI command to reset receiver calibration to default values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIResetReceiverCalibration(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	/* Resets calibration values to default */
	ResetReceiverCalibrationValues();
	strncpy((char*) pcWriteBuffer, "RC receiver calibration set to default values.\r\n", xWriteBufferLen);

	/* Return false to indicate command activity finished */
	return pdFALSE;
}

/**
 * @brief  Implements the CLI command to print receiver values with or without serialization
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetReceiver(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength;
	portBASE_TYPE lParameterNumber = 0;

	/* Empty pcWriteBuffer so no strange output is sent as command response */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	lParameterNumber++;

	/* Obtain the parameter string. */
	pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
			lParameterNumber, /* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
	);

	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	/* Get the current receiver values */
	if(pcParameter[0] == 'n')
		PrintReceiverValues(NO_SERIALIZATION);
	else if(pcParameter[0] == 'p')
		PrintReceiverValues(PROTOBUFFER_SERIALIZATION);

	/* Return false to indicate command activity finished */
	return pdFALSE;
}

/**
 * @brief  Implements the CLI command to print receiver calibration values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetReceiverCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {

	static uint8_t currentChannelPrint = 0;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	/* Get the current receiver values */
	switch (currentChannelPrint) {
	case 0:
		strncpy((char*) pcWriteBuffer,
				"Receiver channel calibration values:\r\nNOTE: Values are specified in timer ticks.\r\n",
				xWriteBufferLen);
		break;
	case 1:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Throttle Max: %u\nThrottle Mid: %u\nThrottle Min: %u\nAileron Max: %u\nAileron Mid: %u\nAileron Min: %u\nElevator Max: %u\nElevator Mid: %u\nElevator Min: %u\nRudder Max: %u\nRudder Mid: %u\nRudder Min: %u\n",
				GetThrottleReceiverCalibrationMaxValue(), GetThrottleReceiverCalibrationMidValue(), GetThrottleReceiverCalibrationMinValue(),
				GetAileronReceiverCalibrationMaxValue(), GetAileronReceiverCalibrationMidValue(), GetAileronReceiverCalibrationMinValue(),
				GetElevatorReceiverCalibrationMaxValue(), GetElevatorReceiverCalibrationMidValue(), GetElevatorReceiverCalibrationMinValue(),
				GetRudderReceiverCalibrationMaxValue(), GetRudderReceiverCalibrationMidValue(), GetRudderReceiverCalibrationMinValue());
		break;
	case 2:
		snprintf((char*) pcWriteBuffer, xWriteBufferLen,
				"Gear Max: %u\nGear Mid: %u\nGear Min: %u\nAux1 Max: %u\nAux1 Mid: %u\nAux1 Min: %u\n\r\n",
				GetGearReceiverCalibrationMaxValue(), GetGearReceiverCalibrationMidValue(), GetGearReceiverCalibrationMinValue(),
				GetAux1ReceiverCalibrationMaxValue(), GetAux1ReceiverCalibrationMidValue(), GetAux1ReceiverCalibrationMinValue());
		break;
	default:
		// memset(pcWriteBuffer, 0x00, xWriteBufferLen);
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
static portBASE_TYPE CLIStartReceiverSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Starting print sampling of receiver values\r\n", xWriteBufferLen);
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

		size_t paramMaxSize;
		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		receiverSampleTime = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncat((char*) pcWriteBuffer, "Duration (s): ", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\n\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		receiverSampleDuration = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
				lParameterNumber, /* Return the next parameter. */
				&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Set serialization and start the receiver value sampling task */
		if (pcParameter[0] == 'n') {
			SetReceiverPrintSamplingSerialization(NO_SERIALIZATION);
		} else if (pcParameter[0] == 'p') {
			SetReceiverPrintSamplingSerialization(PROTOBUFFER_SERIALIZATION);
		}
		StartReceiverSamplingTask(receiverSampleTime, receiverSampleDuration);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == startReceiverSamplingCommand.cExpectedNumberOfParameters) {
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
static portBASE_TYPE CLIStopReceiverSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Stopping printing of receiver sample values...\r\n", xWriteBufferLen);

	/* Stop the receiver printing task */
	StopReceiverSamplingTask();

	return pdFALSE;
}

/**
 * @brief  Implements CLI command to print the last sampled sensor values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetSensors(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength;
	portBASE_TYPE lParameterNumber = 0;

	/* Empty pcWriteBuffer so no strange output is sent as command response */
	configASSERT(pcWriteBuffer);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	lParameterNumber++;

	/* Obtain the parameter string. */
	pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
			lParameterNumber, /* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
	);

	/* Get the current sensor values */
	if(pcParameter[0] == 'n')
		PrintSensorValues(NO_SERIALIZATION);
	else if(pcParameter[0] == 'p')
		PrintSensorValues(PROTOBUFFER_SERIALIZATION);

	return pdFALSE;
}

/**
 * @brief  Starts sensor sampling for a specified sample time and duration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStartSensorSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;
	SerializationType_TypeDef serialisationType = NO_SERIALIZATION;
	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Starting print sampling of sensor values...\r\n", xWriteBufferLen);
	} else {
		uint16_t sensorSampleTime;
		uint16_t sensorSampleDuration;

		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncpy((char*) pcWriteBuffer, "Sample time (ms): ", xWriteBufferLen);

		size_t paramMaxSize;
		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		sensorSampleTime = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncat((char*) pcWriteBuffer, "Duration (s): ", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\n\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		sensorSampleDuration = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
				lParameterNumber, /* Return the next parameter. */
				&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Set serialization and start the receiver value sampling task */

    switch (pcParameter[0]) {
      case 'n':
        serialisationType = NO_SERIALIZATION;
        break;
      case 'p':
        serialisationType = PROTOBUFFER_SERIALIZATION;
        break;
		 case 'c':
       serialisationType = CALIBRATION_SERIALIZATION;
       break;
    }

    SetSensorPrintSamplingSerialization(serialisationType);
		/* Start the sensor sample printing task */
		StartSensorSamplingTask(sensorSampleTime, sensorSampleDuration);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == startSensorSamplingCommand.cExpectedNumberOfParameters) {
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
 * @brief  Stops printing of sensor sample values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStopSensorSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Stopping printing of sensor sample values...\r\n", xWriteBufferLen);

	/* Stop the sensor sample printing task */
	StopSensorSamplingTask();

	return pdFALSE;
}

/**
 * Starts magnetometer calibration procedure.
 */
static portBASE_TYPE CLIStartAccMagMtrCalibration(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
  (void) pcCommandString;
  configASSERT(pcWriteBuffer);

  strncpy((char*) pcWriteBuffer, "Starting Acc/Mag meter parameters "
      "not applied to acc or mag values ...\r\n", xWriteBufferLen);

  StartAccMagMtrCalibration(ACCMAG_CALIBRATION_SAMPLES_N);
  return pdFALSE; /* false indicates CLI activity completed */
}

/**
 * @brief  Implements CLI command to print the last control signal values sent to the motors
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetMotorValues(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength;
	portBASE_TYPE lParameterNumber = 0;

	configASSERT(pcWriteBuffer);

	/* Empty pcWriteBuffer so no strange output is sent as command response */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	lParameterNumber++;

	/* Obtain the parameter string. */
	pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
			lParameterNumber, /* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
	);

	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	/* Get the current receiver values */
	if(pcParameter[0] == 'n')
		PrintMotorControlValues(NO_SERIALIZATION);
	else if(pcParameter[0] == 'p')
		PrintMotorControlValues(PROTOBUFFER_SERIALIZATION);

	return pdFALSE;
}

/**
 * @brief  Starts sensor sampling for a specified sample time and duration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStartMotorSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Starting print sampling of motor control signal values...\r\n", xWriteBufferLen);
	} else {
		uint16_t motorSampleTime;
		uint16_t motorSampleDuration;

		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncpy((char*) pcWriteBuffer, "Sample time (ms): ", xWriteBufferLen);

		size_t paramMaxSize;
		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		motorSampleTime = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncat((char*) pcWriteBuffer, "Duration (s): ", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\n\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		motorSampleDuration = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Set serialization and start the motor control sampling task */
		if (pcParameter[0] == 'n') {
			SetMotorPrintSamplingSerialization(NO_SERIALIZATION);
		} else if (pcParameter[0] == 'p') {
			SetMotorPrintSamplingSerialization(PROTOBUFFER_SERIALIZATION);
		}

		StartMotorControlSamplingTask(motorSampleTime, motorSampleDuration);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == startMotorSamplingCommand.cExpectedNumberOfParameters) {
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
 * @brief  Stops printing of motor control signal sample values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStopMotorSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Stopping printing of sensor sample values...\r\n", xWriteBufferLen);

	/* Stop the motor control sample printing task */
	StopMotorControlSamplingTask();

	return pdFALSE;
}

/**
 * @brief  Implements "about" command, prints system information
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIAbout(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer,
			"Dragonfly\nThe Dragonfly system is developed at AF Consult in Malmoe, Sweden. It is designed to control a quadrotor UAV.\n\nVersion:",
			xWriteBufferLen);
	strncat((char*) pcWriteBuffer, DF_FCB_VERSION, xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
	strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

	return pdFALSE;
}

/**
 * @brief  Implements "systime" command, prints system up time
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLISysTime(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1, "System time [ms]: %u\r\n",
			(unsigned int) HAL_GetTick());

	return pdFALSE;
}

/**
 * @brief  Implements "get-flight-mode" command, prints current flight control mode
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetFlightMode(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Flight mode: ", xWriteBufferLen);

	switch(GetFlightControlMode()) {
	case FLIGHT_CONTROL_IDLE:
		strncat((char*) pcWriteBuffer, "IDLE", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		break;
	case FLIGHT_CONTROL_RAW:
		strncat((char*) pcWriteBuffer, "RAW", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		break;
	case FLIGHT_CONTROL_PID:
		strncat((char*) pcWriteBuffer, "PID", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		break;
	default:
		strncat((char*) pcWriteBuffer, "UNKNOWN", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);
		break;
	}

	strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

	return pdFALSE;
}

/**
 * @brief  Implements "get-ref-signals" command, prints current reference signals
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetRefSignals(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Re: ", xWriteBufferLen);

	snprintf((char*) pcWriteBuffer, xWriteBufferLen,
			"Reference signals:\nZ velocity: %1.4f m/s\nRoll angle: %1.4f rad\nPitch angle: %1.4f rad\nYaw angular rate: %1.4f rad/s\n",
			GetZVelocityReferenceSignal(), GetRollAngleReferenceSignal(), GetPitchAngleReferenceSignal(), GetYawAngularRateReferenceSignal());

	return pdFALSE;
}

/**
 * @brief  Implements "task-status" command, prints task status
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLITaskStatus(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "\nTask\t\t\t Abs Time\t % Time \tStack (rem b)\n", xWriteBufferLen);
	size_t len = strlen((char*) pcWriteBuffer);
	len += vTaskGetRunTimeStats(pcWriteBuffer + len, xWriteBufferLen-len);
	strncat((char*) pcWriteBuffer, "\n", xWriteBufferLen - len -1);

	return pdFALSE;
}

/**
 * @brief  Implements CLI command to print the last control signal values sent to the motors
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIGetStateValues(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength;
	portBASE_TYPE lParameterNumber = 0;

	configASSERT(pcWriteBuffer);

	/* Empty pcWriteBuffer so no strange output is sent as command response */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	lParameterNumber++;

	/* Obtain the parameter string. */
	pcParameter = (int8_t *) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
			lParameterNumber, /* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
	);

	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	/* Get the current receiver values */
	if(pcParameter[0] == 'n')
		PrintStateValues(NO_SERIALIZATION);
	else if(pcParameter[0] == 'p')
		PrintStateValues(PROTOBUFFER_SERIALIZATION);

	return pdFALSE;
}

/**
 * @brief  Starts state sampling for a specified sample time and duration
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStartStateSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	int8_t* pcParameter;
	portBASE_TYPE xParameterStringLength, xReturn;
	static portBASE_TYPE lParameterNumber = 0;

	/* Check the write buffer is not NULL */
	configASSERT(pcWriteBuffer);

	if (lParameterNumber == 0) {
		/* The first time the function is called after the command has been entered just a header string is returned. */
		strncpy((char*) pcWriteBuffer, "Starting print sampling of state values...\r\n", xWriteBufferLen);
	} else {
		uint16_t stateSampleTime;
		uint16_t stateSampleDuration;

		/* Obtain the parameter string */
		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncpy((char*) pcWriteBuffer, "Sample time (ms): ", xWriteBufferLen);

		size_t paramMaxSize;
		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		stateSampleTime = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		strncat((char*) pcWriteBuffer, "Duration (s): ", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		if ((unsigned long) xParameterStringLength > xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1)
			paramMaxSize = xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1;
		else
			paramMaxSize = xParameterStringLength;

		strncat((char*) pcWriteBuffer, (char*) pcParameter, paramMaxSize);
		strncat((char*) pcWriteBuffer, "\n\r\n", xWriteBufferLen - strlen((char*) pcWriteBuffer) - 1);

		stateSampleDuration = atoi((char*) pcParameter);

		lParameterNumber++;

		pcParameter = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
		lParameterNumber, /* Return the next parameter. */
		&xParameterStringLength /* Store the parameter string length. */);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Set serialization and start the motor control sampling task */
		if (pcParameter[0] == 'n') {
			SetStatePrintSamplingSerialization(NO_SERIALIZATION);
		} else if (pcParameter[0] == 'p') {
			SetStatePrintSamplingSerialization(PROTOBUFFER_SERIALIZATION);
		} else if (pcParameter[0] == 'c') {
		  SetStatePrintSamplingSerialization(CALIBRATION_SERIALIZATION);
		}

		StartStateSamplingTask(stateSampleTime, stateSampleDuration);
	}

	/* Update return value and parameter index */
	if (lParameterNumber == startStateSamplingCommand.cExpectedNumberOfParameters) {
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
 * @brief  Stops printing of state sample values
 * @param  pcWriteBuffer : Reference to output buffer
 * @param  xWriteBufferLen : Size of output buffer
 * @param  pcCommandString : Command line string
 * @retval pdTRUE if more data follows, pdFALSE if command activity finished
 */
static portBASE_TYPE CLIStopStateSampling(int8_t* pcWriteBuffer, size_t xWriteBufferLen, const int8_t* pcCommandString) {
	/* Remove compile time warnings about unused parameters, and check the write buffer is not NULL */
	(void) pcCommandString;
	configASSERT(pcWriteBuffer);

	strncpy((char*) pcWriteBuffer, "Stopping printing of state sample values...\r\n", xWriteBufferLen);

	/* Stop the state sample printing task */
	StopStateSamplingTask();

	return pdFALSE;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
