/*****************************************************************************
 * @brief   Module contains UART initialization and handling functions.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "uart.h"

#include "com_cli.h"
#include "fifo_buffer.h"
#include "fcb_error.h"
#include "communication.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define UART_RX_BUFFER_SIZE         512
#define UART_TX_BUFFER_SIZE         512

#define UART_RX_TASK_PRIO           1
#define UART_TX_TASK_PRIO           2

#define UART_COM_TX_QUEUE_ITEMS     16
#define UART_RX_MAX_SEM_COUNT       32

#define UART_COM_MAX_DELAY                 1000 // [ms]

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
uint8_t rxByte = 0;

/* UART Receive FIFO buffer */
uint8_t UartRxBufferArray[UART_RX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef UartRxFIFOBuffer;

/* UART Transmit FIFO buffer */
uint8_t UartTxBufferArray[UART_TX_BUFFER_SIZE];
volatile FIFOBuffer_TypeDef UartTxFIFOBuffer;

/* UART RTOS variables */
xTaskHandle UartRxTaskHandle;
xTaskHandle UartTxTaskHandle;

xQueueHandle UartTxQueue;

xSemaphoreHandle UartRxDataSem;
xSemaphoreHandle UartTxBufferMutex;
xSemaphoreHandle UartTxBinarySem;

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
void HandleUartRxCallback(UART_HandleTypeDef* UartHandle) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    (void) UartHandle; // To avoid warnings

    if (FIFOBufferPutByte(&UartRxFIFOBuffer, rxByte)) {
        /* # Signal UART RX task that new data has arrived #### */
        xSemaphoreGiveFromISR(UartRxDataSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * @brief  Handles the UART Tx Callback
 * @param  None.
 * @retval None.
 */
void HandleUartTxCallback(UART_HandleTypeDef* UartHandle) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    (void) UartHandle; // To avoid warnings

    xSemaphoreGiveFromISR(UartTxBinarySem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * @brief  Handles the UART Rx Callback
 * @param  None.
 * @retval None.
 */
void HandleUartErrorCallback(UART_HandleTypeDef* UartHandle) {
    if(HAL_UART_DeInit(UartHandle) != HAL_OK) {
        ErrorHandler();
    }
    if(HAL_UART_Init(UartHandle) != HAL_OK) {
        ErrorHandler();
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
 * @brief  Create UART Tx Queue
 * @param  None.
 * @retval None.
 */
void CreateUARTComQueues(void) {
    /* # Create queue for outgoing data ####################################### */
    UartTxQueue = xQueueCreate(UART_COM_TX_QUEUE_ITEMS, sizeof(ComTxQueueItem_TypeDef));

    /* We want this queue to be viewable in a RTOS kernel aware debugger, so register it. */
    vQueueAddToRegistry(UartTxQueue, (signed portCHAR*) "UartTxQueue");
}

/*
 * @brief  Create UART Semaphores
 * @param  None.
 * @retval None.
 */
void CreateUARTComSemaphores(void) {
    UartRxDataSem = xSemaphoreCreateCounting(UART_RX_MAX_SEM_COUNT, 0);
    if (UartRxDataSem == NULL) {
        ErrorHandler();
    }

    UartTxBufferMutex = xSemaphoreCreateMutex();
    if (UartTxBufferMutex == NULL) {
        ErrorHandler();
    }

    /* Create binary semaphore to synchronize UART output/TX. The semaphore is taken when device is
     * transmitting (UART DMA busy) and given when transfer has completed. */
    UartTxBinarySem = xSemaphoreCreateBinary();
    if (UartTxBinarySem == NULL) {
        ErrorHandler();
    }
}

/**
 * @brief  Send data over UART interface. The data is entered into an output FIFO buffer which is then entered is
 *         then referred to into a queue. Both the FIFO buffer and the queue are protected by semaphore which needs to pended on.
 * @param  sendData : Reference to the data to be sent
 * @param  sendDataSize : Size of data to be sent
 * @param  maxMutexWaitTicks : Max ticks to wait for the FIFO buffer mutex
 * @param  maxQueueWaitTicks : Max ticks to wait for the queue
 * @retval Result of the operation: UART_OK if all operations are OK else UART_FAIL
 */
UartStatus UartSendData(const uint8_t* sendData, const uint16_t sendDataSize) {
    ComTxQueueItem_TypeDef UartTxQueueItem;
    UartStatus result = UART_OK;

    if (xSemaphoreTake(UartTxBufferMutex, UART_COM_MAX_DELAY) == pdPASS) {
        // Mutex obtained - access the shared resource
        if (FIFOBufferPutData(&UartTxFIFOBuffer, sendData, sendDataSize)) {
            UartTxQueueItem.bufferType = FIFO_BUFFER;
            UartTxQueueItem.bufferPtr = (void*) &UartTxFIFOBuffer;
            UartTxQueueItem.BufferMutex = &UartTxBufferMutex;
            UartTxQueueItem.dataSize = sendDataSize;

            // The Tx Queue needs to be accessed in critical region since we don't want items from the same FIFO entering it in the wrong order!
            if (xQueueSend(UartTxQueue, &UartTxQueueItem, UART_COM_MAX_DELAY) != pdPASS) {
                // Queue full, delete data from FIFO
                FIFOBufferDeleteLastEnteredBytes(&UartTxFIFOBuffer, sendDataSize);
                result = UART_FAIL;
            }
        } else {
            result = UART_FAIL;
        }

        xSemaphoreGive(UartTxBufferMutex); // We have finished accessing the shared resource. Release the mutex.
    } else {
        result = UART_FAIL;
    }

    return result;
}

/**
 * @brief  Send a string over the UART interface.
 * @param  sendString : Reference to the string to be sent
 * @param  maxMutexWaitTicks : Max ticks to wait for the FIFO buffer mutex
 * @param  maxQueueWaitTicks : Max ticks to wait for the queue
 * @retval Result of the operation: UART_OK if all operations are OK else UART_FAIL
 */
UartStatus UartSendString(const char* sendString) {
    return UartSendData((uint8_t*) sendString, strlen(sendString));
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
            /* Read out the buffer until '\n' */
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
                    xMoreDataToFollow = FreeRTOS_CLIProcessCommand( (int8_t*) cliInBuffer, /* The command string.*/
                                                                    (int8_t*) cliOutBuffer, /* The output buffer. */
                                                                    MAX_CLI_OUTPUT_SIZE); /* The size of the output buffer. */

                    UartSendString((char*) cliOutBuffer);

                } while (xMoreDataToFollow != pdFALSE);
                GiveCLIMutex();

                i = 0;
                memset(cliInBuffer, 0x00, sizeof(cliInBuffer));
            }

            /* If rxTempBuffer full without found command */
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

    static ComTxQueueItem_TypeDef UartTxQueueItem;
    uint8_t* txDataPtr;
    uint16_t tmpSize;

    for (;;) {
        /* Wait forever for incoming data over UART by pending on the UART Tx queue */
        if (pdPASS == xQueueReceive(UartTxQueue, &UartTxQueueItem, portMAX_DELAY)) {
            if (xSemaphoreTake(UartTxBinarySem, UART_COM_MAX_DELAY) == pdPASS) { // Pend on UART Binary Semaphore while previous transmission is in progress
                /* Take the buffer mutex (if it has one) */
                if ((*UartTxQueueItem.BufferMutex == NULL || xSemaphoreTake(*UartTxQueueItem.BufferMutex, portMAX_DELAY) == pdPASS)) {
                    switch (UartTxQueueItem.bufferType) {
                    case ARRAY_BUFFER:
                        // Tx buffer is just a good ol' array of data;
                        if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*) UartTxQueueItem.bufferPtr, UartTxQueueItem.dataSize)!= HAL_OK) {
                            ErrorHandler();
                        }
                        break;
                    case FIFO_BUFFER:
                        // Tx buffer is a FIFO ring buffer that wraps around its zero index
                        tmpSize = FIFOBufferGetData((volatile FIFOBuffer_TypeDef*) UartTxQueueItem.bufferPtr,
                                &txDataPtr, UartTxQueueItem.dataSize);

                        if(HAL_UART_Transmit_DMA(&UartHandle, txDataPtr, tmpSize)!= HAL_OK) {
                            ErrorHandler();
                        }

                        // If not all data received from buffer (due to FIFO wrap-around), get the rest
                        if (tmpSize < UartTxQueueItem.dataSize) {
                            tmpSize = FIFOBufferGetData((volatile FIFOBuffer_TypeDef*) UartTxQueueItem.bufferPtr,
                                    &txDataPtr, UartTxQueueItem.dataSize - tmpSize);

                            if(HAL_UART_Transmit_DMA(&UartHandle, txDataPtr, tmpSize)!= HAL_OK) {
                                ErrorHandler();
                            }
                        }
                        break;
                    default:
                        /* Unspecified buffer type, indicate error */
                        ErrorHandler();
                        break;
                    }

                    if (*UartTxQueueItem.BufferMutex != NULL) {
                        xSemaphoreGive(*UartTxQueueItem.BufferMutex); // Give buffer mutex
                    }
                }
            }
        }
    }
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
