/******************************************************************************
 * @file    fcb_sensors.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-07-24
 * @brief   Implementation of interface publicised in fcb_sensors.h
 ******************************************************************************/

#include "fcb_sensors.h"
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_gyroscope.h"
#include "fcb_barometer.h"
#include "fcb_error.h"
#include "fcb_retval.h"
#include "dragonfly_fcb.pb.h"
#include "pb_encode.h"
#include "usbd_cdc_if.h"

#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define FCB_SENSORS_DEBUG // Define to enable sensor debug functions

#define PROCESS_SENSORS_TASK_PRIO					configMAX_PRIORITIES-1 // Max priority

#define SENSOR_DRDY_TIMEOUT                         500 // [ms]
#define SENSOR_ERROR_TIMEOUT                        2000 // [ms]

#define	SENSOR_PRINT_MAX_STRING_SIZE				192
#define SENSOR_PRINT_SAMPLING_TASK_PRIO				1
#define SENSOR_PRINT_MINIMUM_SAMPLING_TIME			10 // [ms]

/* Private typedef -----------------------------------------------------------*/

/**
 * Messages sent to the SENSORS queue
 *
 * The messages are sent as uin8_t under the assumption that
 * less than 255 different kinds of messages will ever be
 * needed.
 *
 * @see FcbSensorEvent
 */
typedef struct FcbSensorMsg {
    uint8_t event;
} FcbSensorMsgType;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* static data declarations */

static xTaskHandle hSensorsTask;
static xQueueHandle qFcbSensors = NULL;

uint8_t sensorSampleRateDone = 0;

typedef struct FcbSensorDataRateCalc {
    uint32_t lastDrdyTime;      // [ms]
} FcbSensorDataRateCalcType;

static FcbSensorDataRateCalcType sensorDrdyCalc[FCB_SENSOR_NBR] = { { 0} };

/* Task handle for printing of sensor values task */
xTaskHandle SensorPrintSamplingTaskHandle = NULL;
static volatile uint16_t sensorPrintSampleTime;
static volatile uint16_t sensorPrintSampleDuration;

/* Private function prototypes -----------------------------------------------*/
static void _ProcessSensorValues(void*);
static void _FetchSensorAtTimeout(uint8_t event);

static void _DebugFlashLEDs(uint8_t event);
static void _SensorPrintSamplingTask(void const *argument);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initialises the sensor processing queue and thread
 * @param  None
 * @retval FCB_OK if thread started, else FCB_ERR
 */
int FcbSensorsConfig(void) {
    portBASE_TYPE rtosRetVal;
    int retVal = FCB_OK;

    if (0 == (qFcbSensors = xQueueCreate(FCB_SENSORS_QUEUE_SIZE, FCB_SENSORS_Q_MSG_SIZE))) {
        ErrorHandler();
        retVal = FCB_ERR_INIT;
    }

    if (pdPASS != (rtosRetVal = xTaskCreate((pdTASK_CODE )_ProcessSensorValues, (signed portCHAR*)"SENSORS",
                    4 * configMINIMAL_STACK_SIZE, NULL /* parameter */,PROCESS_SENSORS_TASK_PRIO /* priority */,
                    &hSensorsTask))) {
        ErrorHandler();
        retVal = FCB_ERR_INIT;
    }

    return retVal;
}

bool GetSensorDrdyCalcIndex(uint8_t event, uint32_t *index) {
    bool retVal = true;
	switch (event) {
    case FCB_SENSOR_GYRO_DATA_READY:
    	*index = GYRO_IDX;
        break;
    case FCB_SENSOR_ACC_DATA_READY:
    	*index = ACC_IDX;
        break;
    case FCB_SENSOR_MAGNETO_DATA_READY:
    	*index = MAG_IDX;
        break;
    default:
        retVal = false;
        break;
    }

	return retVal;
}

/*
 * @brief  Handles data ready interrupt from each sensor, adds sensor read request message to queue
 * @param  event : Sensor data ready event enum
 * @retval None
 */
void FcbSendSensorMessageFromISR(uint8_t event) {
    FcbSensorMsgType msg = { 0 };
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

#ifdef FCB_SENSORS_DEBUG
    _DebugFlashLEDs(event);
#endif

    uint32_t sensorDrdyCalcIndex = 0;
    if (!GetSensorDrdyCalcIndex(event, &sensorDrdyCalcIndex)) {
    	return;
    }

    msg.event = event;
    sensorDrdyCalc[sensorDrdyCalcIndex].lastDrdyTime = HAL_GetTick();

    if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
        if(pdFALSE != xQueueIsQueueFullFromISR(qFcbSensors)) {
            // If the queue is full, empty it
            xQueueReset(qFcbSensors);

            // Then add a message of each event type
            msg.event = FCB_SENSOR_GYRO_DATA_READY;
            if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
                ErrorHandler();
            }

            msg.event = FCB_SENSOR_ACC_DATA_READY;
            if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
                ErrorHandler();
            }

            msg.event = FCB_SENSOR_MAGNETO_DATA_READY;
            if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
                ErrorHandler();
            }
        } else {
            ErrorHandler();
        }
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/*
 * @brief  Adds sensor read request message to queue
 * @param  event : Sensor read data event enum
 * @retval None
 */
void FcbSendSensorMessage(uint8_t event) {
    FcbSensorMsgType msg = { 0 };
    uint8_t status;

    msg.event = event;

    if (pdTRUE != (status = xQueueSend(qFcbSensors, &msg, portMAX_DELAY))) {
        ErrorHandler();
    }
}

void FcbSensorsInitGpioPinForInterrupt(GPIO_TypeDef  *GPIOx, uint32_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/* Private functions ---------------------------------------------------------*/

static void _FetchSensorAtTimeout(uint8_t event) {
    uint32_t sensorDrdyCalcIndex;
    if (!GetSensorDrdyCalcIndex(event, &sensorDrdyCalcIndex)) {
    	// Not a valid event.
    	return;
    }

    // lastDrdyTime must be read before HAL_GetTick(), as lastDrdyTime is updated from a ISR.
	// Else lastDrdyTime could be increased after HAL_GetTick() and timeSinceDrdy < 0.
	uint32_t lastDrdyTime = sensorDrdyCalc[sensorDrdyCalcIndex].lastDrdyTime;
	uint32_t timeSinceDrdy = HAL_GetTick() - lastDrdyTime;
	if (timeSinceDrdy > SENSOR_ERROR_TIMEOUT) {
		ErrorHandler();
	} else if (timeSinceDrdy > SENSOR_DRDY_TIMEOUT) {
		FetchDataFromGyroscope();
	}
}

static void _ProcessSensorValues(void* val __attribute__ ((unused))) {
    /*
     * configures the sensors to start giving Data Ready interrupts
     * and then polls the queue in an infinite loop
     */
    FcbSensorMsgType msg;

    if (FCB_OK != InitialiseGyroscope()) {
        ErrorHandler();
    }

    if (FCB_OK != FcbInitialiseAccMagSensor()) {
        ErrorHandler();
    }

    if (FCB_OK != FcbInitialiseBarometer()) {
    	ErrorHandler();
    }

    while (1) {
        if (pdFALSE == xQueueReceive(qFcbSensors, &msg,  SENSOR_ERROR_TIMEOUT)) {
            /*
             * if no message was received, interrupts from the sensors
             * aren't arriving and this is a serious error.
             */
            ErrorHandler();
        }

        switch (msg.event) {
        case FCB_SENSOR_GYRO_DATA_READY:
            FetchDataFromGyroscope();
            break;
        case FCB_SENSOR_ACC_DATA_READY:
            FetchDataFromAccelerometer();
            break;
        case FCB_SENSOR_MAGNETO_DATA_READY:
            FetchDataFromMagnetometer();
            break;
        }

        /* Check for sensor data ready read timeouts */
        _FetchSensorAtTimeout(FCB_SENSOR_GYRO_DATA_READY);
        _FetchSensorAtTimeout(FCB_SENSOR_ACC_DATA_READY);
        _FetchSensorAtTimeout(FCB_SENSOR_MAGNETO_DATA_READY);

    }
}

static void _DebugFlashLEDs(uint8_t event) {
    static uint32_t acc_cbk_sensor_counter = 0;
    static uint32_t mag_cbk_sensor_counter = 0;
    static uint32_t gyro_cbk_sensor_counter = 0;

    if(event == FCB_SENSOR_ACC_DATA_READY) {
    	if ((acc_cbk_sensor_counter % 200) == 0) {
    		BSP_LED_Toggle(LED5);
    	}
    	acc_cbk_sensor_counter++;
    } else if(event == FCB_SENSOR_MAGNETO_DATA_READY) {
    	if ((mag_cbk_sensor_counter % 200) == 0) {
    		BSP_LED_Toggle(LED3);
    	}
    	mag_cbk_sensor_counter++;
    } else if(event == FCB_SENSOR_GYRO_DATA_READY) {
    	if ((gyro_cbk_sensor_counter % 200) == 0) {
    		BSP_LED_Toggle(LED4);
    	}
    	gyro_cbk_sensor_counter++;
    }
}

/* Debug Print functions ---------------------------------------------------------*/

/*
 * @brief  Creates a task to sample print sensor values over USB
 * @param  sampleTime : Sets how often samples should be printed
 * @param  sampleDuration : Sets for how long sampling should be performed
 * @retval FCB_OK if thread started, else FCB_ERR
 */
FcbRetValType StartSensorSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration) {
  if(sampleTime < SENSOR_PRINT_MINIMUM_SAMPLING_TIME)
    sensorPrintSampleTime = SENSOR_PRINT_MINIMUM_SAMPLING_TIME;
  else
    sensorPrintSampleTime = sampleTime;

  sensorPrintSampleDuration = sampleDuration;

  /* Sensor value print sampling handler thread creation
   * Task function pointer: SensorPrintSamplingTask
   * Task name: SENS_PRINT_SAMPL
   * Stack depth: 2*configMINIMAL_STACK_SIZE
   * Parameter: NULL
   * Priority: SENSOR_PRINT_SAMPLING_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
   * Handle: SensorPrintSamplingTaskHandle
   * */
  if (pdPASS != xTaskCreate((pdTASK_CODE )_SensorPrintSamplingTask, (signed portCHAR*)"SENS_PRINT_SAMPL",
      3*configMINIMAL_STACK_SIZE, NULL, SENSOR_PRINT_SAMPLING_TASK_PRIO, &SensorPrintSamplingTaskHandle)) {
    ErrorHandler();
    return FCB_ERR;
  }

  return FCB_OK;
}

/*
 * @brief  Stops sensor print sampling by deleting the task
 * @param  None
 * @retval FCB_OK if task deleted, FCB_ERR if not
 */
FcbRetValType StopSensorSamplingTask(void) {
  if(SensorPrintSamplingTaskHandle != NULL) {
    vTaskDelete(SensorPrintSamplingTaskHandle);
    SensorPrintSamplingTaskHandle = NULL;
    return FCB_OK;
  }
  return FCB_ERR;
}

/**
 * @brief  Prints the latest sensor values to the USB com port
 * @param  none
 * @retval none
 */
void PrintSensorValues(void) {
  char sensorString[SENSOR_PRINT_MAX_STRING_SIZE];
  float32_t accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;

  /* Get the latest sensor values */
  GetAcceleration(&accX, &accY, &accZ);
  GetGyroAngleDot(&gyroX, &gyroY, &gyroZ);
  GetMagVector(&magX, &magY, &magZ);

  snprintf((char*) sensorString, SENSOR_PRINT_MAX_STRING_SIZE,
          "AccXYZ: %f, %f, %f\r\nGyrXYZ: %f, %f, %f\r\nMagXYZ: %f, %f, %f\n\r\n",
          accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
  USBComSendString(sensorString);
}

/**
 * @brief  Task code handles sensor print sampling
 * @param  argument : Unused parameter
 * @retval None
 */
static void _SensorPrintSamplingTask(void const *argument) {
  (void) argument;

  portTickType xLastWakeTime;
  portTickType xSampleStartTime;

  /* Initialise the xLastWakeTime variable with the current time */
  xLastWakeTime = xTaskGetTickCount();
  xSampleStartTime = xLastWakeTime;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, sensorPrintSampleTime);

    PrintSensorValues();

    /* If sampling duration exceeded, delete task to stop sampling */
    if (xTaskGetTickCount() >= xSampleStartTime + sensorPrintSampleDuration * configTICK_RATE_HZ)
      StopSensorSamplingTask();
  }
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
