/******************************************************************************
 * @file    fcb_sensors.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-07-24
 * @brief   Implementation of interface publicised in fcb_sensors.h
 ******************************************************************************/

#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensors.h"
#include "fcb_gyroscope.h"
#include "fcb_error.h"
#include "fcb_retval.h"
#include "dragonfly_fcb.pb.h"
#include "pb_encode.h"
#include "usb_com_cli.h"

#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define FCB_SENSORS_DEBUG /* todo delete */

#define PROCESS_SENSORS_TASK_PRIO					configMAX_PRIORITIES-1 // Max priority

#define SENSOR_PRINT_SAMPLING_TASK_PRIO				1
#define SENSOR_PRINT_MINIMUM_SAMPLING_TIME			10 // [ms]

#define	SENSOR_PRINT_MAX_STRING_SIZE				192

#ifdef FCB_SENSORS_DEBUG
static uint32_t cbk_gyro_counter = 0;
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* static data declarations */

static xTaskHandle hSensorsTask;
static xQueueHandle qFcbSensors = NULL;

uint8_t sensorSampleRateDone = 0;


typedef struct FcbSensorDataRateCalc {
  uint32_t intervalStart;
  float32_t samplePeriod; /* result */
  uint8_t count;
} FcbSensorDataRateCalcType;

/* this is used to calculate sensor sample periods ...
 * The true rate may not be the nominal value and the true
 * rate is needed for filter constants in the Kalman Filter
 * in state estimation.
 */
static FcbSensorDataRateCalcType sensorDrdyCalc[FCB_SENSOR_NBR] = { { 0, 0, 0.0f} };

static FcbSensorCbk sClientCbk = NULL;

#ifdef FCB_SENSORS_DEBUG
static float32_t sensorSampleRates[FCB_SENSOR_NBR] = { 0.0f };
#endif

/* Task handle for printing of sensor values task */
xTaskHandle SensorPrintSamplingTaskHandle = NULL;
static volatile uint16_t sensorPrintSampleTime;
static volatile uint16_t sensorPrintSampleDuration;
static SerializationType sensorValuesPrintSerializationType;

/* Private function prototypes -----------------------------------------------*/
static void _ProcessSensorValues(void*);
static uint8_t _CountSensorDrdy(uint8_t msg); /* not used */
static uint8_t _CalculateDrdyDeltaT(void);
static void _SensorPrintSamplingTask(void const *argument);


/* Exported functions --------------------------------------------------------*/
/* global fcn definitions */
int FcbSensorsConfig(void) {
  portBASE_TYPE rtosRetVal;
  int retVal = FCB_OK;

  if (0 == (qFcbSensors = xQueueCreate(FCB_SENSORS_QUEUE_SIZE,
      FCB_SENSORS_Q_MSG_SIZE))) {
    ErrorHandler();
    goto Error;
  }


  if (pdPASS != (rtosRetVal =
      xTaskCreate((pdTASK_CODE)_ProcessSensorValues,
          (signed portCHAR*)"SENSORS",
          4 * configMINIMAL_STACK_SIZE,
          NULL /* parameter */,
          PROCESS_SENSORS_TASK_PRIO /* priority */,
          &hSensorsTask))) {
    ErrorHandler();
    goto Error;
  }

Exit:
  return retVal;

Error:
  /* clean up */
  retVal = FCB_ERR_INIT;
  goto Exit;
}

uint8_t FcbSensorRegisterClientCallback(FcbSensorCbk cbk) {
  if (NULL != sClientCbk) {
    return FCB_ERR;
  }

  sClientCbk = cbk;

  return FCB_OK;
}


void FcbSensorPush2Client(FcbSensorIndexType sensorType, uint8_t deltaTms, float32_t const * xyz) {
  float32_t deltaT = (float) (deltaTms) / 1000; /* from ms to s */
  if (NULL != sClientCbk) {
    sClientCbk(sensorType, deltaT, xyz);
  }
}


void FcbSendSensorMessageFromISR(uint8_t event) {
  FcbSensorMsgType msg = { 0 , 0 };
  portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

#ifdef FCB_SENSORS_DEBUG
  /* this function counts incoming DRDY signals from the sensors */

  if ((cbk_gyro_counter % 48) == 0) {
    BSP_LED_Toggle(LED5);
  }
  cbk_gyro_counter++;
#endif

  if (0 == sensorSampleRateDone) {
    sensorSampleRateDone = _CountSensorDrdy(event);
  }

  msg.event = event;
  msg.deltaTime = _CalculateDrdyDeltaT();

  if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
    ErrorHandler();
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void FcbSensorsInitGpioPinForInterrupt(GPIO_TypeDef  *GPIOx, uint32_t pin) {
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);
}

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
void PrintSensorValues(const SerializationType serializationType) {
  char sensorString[SENSOR_PRINT_MAX_STRING_SIZE];
  float32_t accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;

  /* Get the latest sensor values */
  GetAcceleration(&accX, &accY, &accZ);
  GetGyroAngleDot(&gyroX, &gyroY, &gyroZ);
  GetMagVector(&magX, &magY, &magZ);

  switch (serializationType) {
    case  NO_SERIALIZATION:
      snprintf((char*) sensorString, SENSOR_PRINT_MAX_STRING_SIZE,
               "Accelerometer [m/s^2]:\nAccX: %1.3f\nAccY: %1.3f\nAccZ: %1.3f\r\nGyroscope [rad/s]:\nGyroX: %1.3f\nGyroY: %1.3f\nGyroZ: %1.3f\r\nMagnetometer [G]:\nMagX: %1.3f\nMagY: %1.3f\nMagZ: %1.3f\n\r\n",
               accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
      USBComSendString(sensorString);
      break;
    case CALIBRATION_SERIALIZATION:
      snprintf((char*) sensorString, SENSOR_PRINT_MAX_STRING_SIZE,
               "AccXYZ: %f, %f, %f\r\nGyrXYZ: %f, %f, %f\r\nMagXYZ: %f, %f, %f\n\r\n",
               accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
      USBComSendString(sensorString);
      break;
    case PROTOBUFFER_SERIALIZATION:  {
      bool protoStatus;
      uint8_t serializedSensorData[SensorSamplesProto_size];
      SensorSamplesProto sensorSamplesProto;
      uint32_t strLen;

      /* Update the protobuffer type struct members */
      sensorSamplesProto.has_accX = true;
      sensorSamplesProto.accX = accX;
      sensorSamplesProto.has_accY = true;
      sensorSamplesProto.accY = accY;
      sensorSamplesProto.has_accZ = true;
      sensorSamplesProto.accZ = accZ;

      sensorSamplesProto.has_gyroAngRateXb = true;
      sensorSamplesProto.gyroAngRateXb = gyroX;
      sensorSamplesProto.has_gyroAngRateYb = true;
      sensorSamplesProto.gyroAngRateYb = gyroY;
      sensorSamplesProto.has_gyroAngRateZb = true;
      sensorSamplesProto.gyroAngRateZb = gyroZ;

      sensorSamplesProto.has_magX = true;
      sensorSamplesProto.magX = magX;
      sensorSamplesProto.has_magY = true;
      sensorSamplesProto.magY = magY;
      sensorSamplesProto.has_magZ = true;
      sensorSamplesProto.magZ = magZ;

      /* Create a stream that will write to our buffer and encode the data with protocol buffer */
      pb_ostream_t protoStream = pb_ostream_from_buffer(serializedSensorData, SensorSamplesProto_size);
      protoStatus = pb_encode(&protoStream, SensorSamplesProto_fields, &sensorSamplesProto);

      /* Insert header to the sample string, then copy the data after that */
      snprintf(sensorString, SENSOR_PRINT_MAX_STRING_SIZE, "%c %c ", SENSOR_SAMPLES_MSG_ENUM, protoStream.bytes_written);
      strLen = strlen(sensorString);
      if(strLen + protoStream.bytes_written + strlen("\r\n") < SENSOR_PRINT_MAX_STRING_SIZE) {
        memcpy(&sensorString[strLen], serializedSensorData, protoStream.bytes_written);
        memcpy(&sensorString[strLen+protoStream.bytes_written], "\r\n", strlen("\r\n"));
      }

      if(protoStatus) {
        USBComSendData((uint8_t*)sensorString, strLen+protoStream.bytes_written+strlen("\r\n"));
      } else {
        ErrorHandler();
      }
    }
    break;
  }
}

  /*
   * @brief  Sets the serialization type of printed sensor sample values
   * @param  serializationType : Data serialization type enum
   * @retval None.
 */
void SetSensorPrintSamplingSerialization(const SerializationType serializationType) {
  sensorValuesPrintSerializationType = serializationType;
}

/* Private functions ---------------------------------------------------------*/

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

  while (1) {
    if (pdFALSE == xQueueReceive (qFcbSensors,
        &msg,
        portMAX_DELAY /* 1000 *//* configTICK_RATE_HZ is 1000 */)) {
      /*
       * if no message was received, no interrupts from the sensors
       * aren't arriving and this is a serious error.
       */
      ErrorHandler();
      goto Error;
    }

    switch (msg.event) {
    case FCB_SENSOR_GYRO_DATA_READY:
      /*
       * As settings are in BSP_GYRO_Init, the callback is called with a frequency
       * of 94.5 Hz according to oscilloscope.
       */
      FetchDataFromGyroscope(msg.deltaTime);
      break;
    case FCB_SENSOR_GYRO_CALIBRATE:
      break;
    case FCB_SENSOR_ACC_DATA_READY:
      FetchDataFromAccelerometer();
      break;
    case FCB_SENSOR_ACC_CALIBRATE:
      break;
    case FCB_SENSOR_MAGNETO_DATA_READY:
      FetchDataFromMagnetometer();
      break;
    case FCB_SENSOR_MAGNETO_CALIBRATE:
      break;
    }

    /* todo: call the state correction part of the Kalman Filter every time
     * we get new sensor values.
     *
     * Either from this function or the separate gyro / acc / magnetometer functions.
     */
  }
  Exit:
  return;
  Error:
  goto Exit;
}


static uint8_t _CountSensorDrdy(uint8_t msg) {
  /*
   * When reading the values directly with debugger, it's apparent there's
   * some fluctuation in the values for the accelerometer and magnetometer.
   *
   * Using an oscilloscope on PE2 it's evident there's a rate fluctuation of
   * about 0.02 Hz, the fluctuation measured by this function is bigger, but the
   * mean is still closer to the true rates than simply using the nominal sample
   * value configured in the sensor code.
   *
   * the function returns 1 when samples have been collected.
   *
   * At the time of writing, only the gyroscope period is used.
   */
  enum { DRDY_ENOUGH = 200 }; /* below 256 as sensorDrdyCalc.count is uint8_t */

  uint8_t idx = 0;
  uint8_t done = 0;

  switch(msg) {
  case FCB_SENSOR_GYRO_DATA_READY:
    idx = GYRO_IDX;
    break;
  case FCB_SENSOR_ACC_DATA_READY:
    idx = ACC_IDX;
    break;
  case FCB_SENSOR_MAGNETO_DATA_READY:
    idx = MAG_IDX;
    break;
  default:
    /* do nothing */
    break;
  }

  if (0 == sensorDrdyCalc[idx].intervalStart) {
    sensorDrdyCalc[idx].intervalStart = HAL_GetTick();
  } else if (DRDY_ENOUGH > sensorDrdyCalc[idx].count) {
    sensorDrdyCalc[idx].count += 1;

    if (DRDY_ENOUGH == sensorDrdyCalc[idx].count) {
      uint32_t elapsedTime = HAL_GetTick() - sensorDrdyCalc[GYRO_IDX].intervalStart;

      sensorDrdyCalc[idx].samplePeriod =
          elapsedTime / (float) DRDY_ENOUGH / (float)configTICK_RATE_HZ;

#ifdef FCB_SENSORS_DEBUG
      sensorSampleRates[idx] = 1 / sensorDrdyCalc[idx].samplePeriod;
#endif
    }
  }

  if ((DRDY_ENOUGH == sensorDrdyCalc[GYRO_IDX].count) &&
      (DRDY_ENOUGH == sensorDrdyCalc[ACC_IDX].count) &&
      (DRDY_ENOUGH == sensorDrdyCalc[MAG_IDX].count)) {
    done= 1;

    SetGyroMeasuredSamplePeriod(sensorDrdyCalc[GYRO_IDX].samplePeriod);
    SetAccMagMeasuredSamplePeriod(sensorDrdyCalc[ACC_IDX].samplePeriod,
                                  sensorDrdyCalc[MAG_IDX].samplePeriod);
  }

  return done;
}
static uint8_t _CalculateDrdyDeltaT(void) {
  static volatile uint32_t previousDrdyTick = 0;
  static volatile uint32_t previousDifference = 0;
  uint32_t nowTick = HAL_GetTick();
  uint8_t deltaT = 0;
  uint32_t difference = 0;

    if (previousDrdyTick == nowTick) /* two interrupts within same tick .. */ {
      difference = previousDifference; /* so re-use the previous diff */
    } else if (previousDrdyTick < nowTick) {
      difference = nowTick - previousDrdyTick;
    } else  /* wraparound */ {
      difference = nowTick + (UINT32_MAX - previousDrdyTick);
    }

    if (difference > 255) {
      static uint8_t first = 1;
      if (--first) {
        /* ignore first time b/c SENSORS thread needs time to start */
        previousDrdyTick = nowTick;
        difference = 10; /* use a dummy value */
      } else {
        ErrorHandler(); /* sanity check */
      }
    }

    previousDrdyTick = nowTick;
    previousDifference = difference;

    return deltaT = (uint8_t) difference;
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

    PrintSensorValues(sensorValuesPrintSerializationType);

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
