/*****************************************************************************
 * @file    uart.c
 * @brief   Module contains UART initialization and handling functions.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uart.h"

#include "usb_com_cli.h"
#include "fifo_buffer.h"
#include "fcb_error.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define UART_RX_BUFFER_SIZE			512
#define UART_TX_BUFFER_SIZE			512

#define UART_RX_TASK_PRIO          	1
#define UART_TX_TASK_PRIO          	2

#define UART_COM_TX_QUEUE_ITEMS     16
#define UART_RX_MAX_SEM_COUNT		4

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
uint8_t rxByte = 0;

/* USB CDC Receive FIFO buffer */
uint8_t UartRxBufferArray[UART_RX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef UartRxFIFOBuffer;

/* USB CDC Transmit FIFO buffer */
uint8_t UartTxBufferArray[UART_TX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef UartTxFIFOBuffer;

/* USB RTOS variables */
xTaskHandle UartRxTaskHandle;
xTaskHandle UartTxTaskHandle;

xQueueHandle UartTxQueue;

xSemaphoreHandle UartRxDataSem;
xSemaphoreHandle UartTxBufferMutex;
xSemaphoreHandle UartTxMutex;

/* Private function prototypes -----------------------------------------------*/
static void InitUartCom(void);

static void UartRxTask(void const *argument);
static void UartTxTask(void const *argument);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes and configures UART.
 * @param  None.
 * @retval None.
 */
void UartConfig(void) {
	UART_HandleTypeDef UartHandle;

	/*##-1- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART configured as defined by header file definitions
	 * Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance        = UART;
	UartHandle.Init.BaudRate   = UART_BAUDRATE;
	UartHandle.Init.WordLength = UART_WORDLENGTH;
	UartHandle.Init.StopBits   = UART_STOPBITS;
	UartHandle.Init.Parity     = UART_PARITY;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	if(HAL_UART_DeInit(&UartHandle) != HAL_OK) {
		ErrorHandler();
	}
	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
		ErrorHandler();
	}
}

/*
 * @brief  Handles the UART Rx Callback
 * @param  None.
 * @retval None.
 */
void HandleUartRxCallback(void) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (FIFOBufferPutByte(&UartRxFIFOBuffer, rxByte)) {
		/* # Signal UART RX task that new data has arrived #### */
		xSemaphoreGiveFromISR(UartRxDataSem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/*
 * @brief  Create the UART communication tasks (for Rx and Tx)
 * @param  None.
 * @retval None.
 */
void CreateUARTComTasks(void) {
	/* UART Rx handler task creation
	 * Task function pointer: UartRXTask
	 * Task name: UART_RX
	 * Stack depth: 3*configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: UART_RX_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: UartRxTaskHandle
	 * */
	if (pdPASS != xTaskCreate((pdTASK_CODE )UartRxTask, (signed portCHAR*)"UART_RX", 3*configMINIMAL_STACK_SIZE, NULL,
					UART_RX_TASK_PRIO, &UartRxTaskHandle)) {
		ErrorHandler();
	}

	/* UART Tx handler task creation
	 * Task function pointer: UartTxTask
	 * Task name: UART_TX
	 * Stack depth: 3*configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: UART_TX_THREAD_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: UartTxTaskHandle
	 * */
	if (pdPASS != xTaskCreate((pdTASK_CODE )UartTxTask, (signed portCHAR*)"UART_TX", 1*configMINIMAL_STACK_SIZE,
					NULL, UART_TX_TASK_PRIO, &UartTxTaskHandle)) {
		ErrorHandler();
	}
}

/*
 * @brief
 * @param  None.
 * @retval None.
 */
void CreateUARTComQueues(void) {

}

/*
 * @brief
 * @param  None.
 * @retval None.
 */
void CreateUARTComSemaphores(void) {
	UartRxDataSem = xSemaphoreCreateCounting(UART_RX_MAX_SEM_COUNT, 0);
	if (UartRxDataSem == NULL) {
		ErrorHandler();
	}
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes the UART Fifo Buffers
 * @param  None.
 * @retval None.
 */
static void InitUartCom(void) {
	/* Create UART RX FIFO Buffer */
	FIFOBufferInit(&UartRxFIFOBuffer, UartRxBufferArray, sizeof(UartRxBufferArray));

	/* Create UART TX FIFO Buffer */
	FIFOBufferInit(&UartTxFIFOBuffer, UartTxBufferArray, sizeof(UartTxBufferArray));

	/* Start receiving data over UART, 1 byte at a time */
	HAL_UART_Receive_DMA(&UartHandle, &rxByte, 1);
}

/**
 * @brief  Task code handles the Uart Rx (receive) communication
 * @param  argument : Unused parameter
 * @retval None
 */
static void UartRxTask(void const *argument) {
	(void) argument;

	uint16_t i = 0;
	portBASE_TYPE xMoreDataToFollow;
	ErrorStatus bufferStatus = SUCCESS;
	uint8_t getByte;
	static uint8_t cliInBuffer[MAX_CLI_COMMAND_SIZE];
	static uint8_t cliOutBuffer[MAX_CLI_OUTPUT_SIZE];

	/* Init UART communication */
	InitUartCom();

	for (;;) {
		bufferStatus = SUCCESS;
		/* Wait forever for incoming data over Uart by pending on the Uart Rx semaphore */
		if (pdPASS == xSemaphoreTake(UartRxDataSem, portMAX_DELAY)) {
			// Read out the buffer until '\n'
			while (bufferStatus == SUCCESS && (i < MAX_CLI_COMMAND_SIZE || getByte != '\n')) {
				bufferStatus = FIFOBufferGetByte(&UartRxFIFOBuffer, &getByte);

				if (bufferStatus == SUCCESS) {
					cliInBuffer[i] = getByte;
				}

				i++;
			}

			/* End of command assumed found ('\n') */
			if (getByte == '\n') {
				TakeCLIMutex();
				do {
					/* Send the command string to the command interpreter. Any output generated
					 * by the command interpreter will be placed in the cliOutBuffer buffer. */
					xMoreDataToFollow = FreeRTOS_CLIProcessCommand((int8_t*) cliInBuffer, /* The command string.*/
					(int8_t*) cliOutBuffer, /* The output buffer. */
					MAX_CLI_OUTPUT_SIZE /* The size of the output buffer. */
					);

					// UARTComSendString((char*) cliOutBuffer); // TODO

				} while (xMoreDataToFollow != pdFALSE);
				GiveCLIMutex();

				i = 0;
				memset(cliInBuffer, 0x00, sizeof(cliInBuffer));
			}

			// If rxTempBuffer full without found command
			if (i >= MAX_CLI_COMMAND_SIZE) {
				i = 0;
				memset(cliInBuffer, 0x00, sizeof(cliInBuffer));
			}
		}
	}
}

/**
 * @brief  Task code handles the UART Tx communication
 * @param  argument : Unused parameter
 * @retval None
 */
static void UartTxTask(void const *argument) {
	(void) argument;

	for (;;) {
	}
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
