/******************************************************************************
 * @file    fcb_gyroscope.c
 * @author  Dragonfly
 *
 * @brief   File contains functionality to handle and read the STM32F3-Discovery
 * 			on-board gyroscope sensor L3DG20.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "fcb_gyroscope.h"
#include "fcb_sensors.h"
#include "l3gd20.h"


#include "fcb_error.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usbd_cdc_if.h"

#include "trace.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FCB_GYRO_DEBUG

/* static & local declarations */

static SendCorrectionUpdateCallback_TypeDef SendCorrectionUpdateCallback = NULL;

enum { XDOT_IDX = 0 }; /* index of sGyroXYZAngleDot & ditto Offset */
enum { YDOT_IDX = 1 }; /* as above */
enum { ZDOT_IDX = 2 }; /* as above */

const float32_t GYRO_X_AXIS_VARIANCE = 0.098603; // TODO These could be used to set Kalman filters Correction variance (R)
const float32_t GYRO_Y_AXIS_VARIANCE = 0.104274;
const float32_t GYRO_Z_AXIS_VARIANCE = 0.103256;
const float32_t GYRO_AXIS_VARIANCE_ROUGH = 0.000256;


/**
 * Angular velocities, in degrees.
 *
 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static float32_t sGyroXYZAngleDot[3] = { 0.0, 0.0, 0.0 }; /* not volatile - only print thread reads */
static float32_t sGyroSamplePeriod = 0.0f;
static xSemaphoreHandle mutexGyro;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/* global fcn definitions */
uint8_t InitialiseGyroscope(void) {
    uint8_t retVal = FCB_OK;
    GPIO_InitTypeDef GPIO_InitStructure;

    if (NULL == (mutexGyro = xSemaphoreCreateMutex())) {
      return FCB_ERR_INIT;
    }

    /* configure GYRO DRDY (data ready) interrupt */
    GYRO_CS_GPIO_CLK_ENABLE(); /* happens to be GPIOE */

    GPIO_InitStructure.Pin = GPIO_PIN_1; /* STM32F3 doc UM1570 page 27/36 */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    /* sets full scale (and sensitivity) plus data rate of L3GD20 gyroscope */
    if(L3GD20_Config() != 0)
    {
        /* Initialization Error */
    	ErrorHandler();
    }

    /* start with configured data rate */
    sGyroSamplePeriod = 1 / (float) L3GD20_DataRateHz();

    FetchDataFromGyroscope(1 /* dummy value */); /* necessary so a fresh DRDY can be triggered */
    return retVal;
}

uint8_t SensorRegisterGyroClientCallback(SendCorrectionUpdateCallback_TypeDef cbk) {
  if (NULL != SendCorrectionUpdateCallback) {
    return FCB_ERR;
  }

  SendCorrectionUpdateCallback = cbk;

  return FCB_OK;
}

void FetchDataFromGyroscope(uint8_t deltaTms) {
    float gyroscopeData[3] = { 0.0f, 0.0f, 0.0f };
    HAL_StatusTypeDef status = HAL_OK;

    /* paranoia - this is used for calculations to reduce the time this function
     * needs to hold the mutexMag
     */
    float lGyroXYZAngleDot[3] = { 0.0f, 0.0f, 0.0f };

    /* returns rad/s */
    status = L3GD20_ReadXYZAngRate(gyroscopeData);
    if (status != HAL_OK) {
#ifdef FCB_GYRO_DEBUG
        USBComSendString("ERROR: L3GD20_ReadXYZAngRate\n");
#endif
        FcbSendSensorMessage(FCB_SENSOR_GYRO_DATA_READY); // Re-send data ready read request if read fails
        return;
    }

    /* see "Sensors" wiki page for gyroscope vs Quadcopter axes orientations */
    lGyroXYZAngleDot[XDOT_IDX] = -gyroscopeData[YDOT_IDX];
    lGyroXYZAngleDot[YDOT_IDX] = -gyroscopeData[XDOT_IDX];
    lGyroXYZAngleDot[ZDOT_IDX] = -gyroscopeData[ZDOT_IDX];

    if (pdTRUE != xSemaphoreTake(mutexGyro,  portMAX_DELAY /* wait forever */)) {
      ErrorHandler();
      return;
    }

    sGyroXYZAngleDot[XDOT_IDX] = lGyroXYZAngleDot[XDOT_IDX];
    sGyroXYZAngleDot[YDOT_IDX] = lGyroXYZAngleDot[YDOT_IDX];
    sGyroXYZAngleDot[ZDOT_IDX] = lGyroXYZAngleDot[ZDOT_IDX];

    if (pdTRUE != xSemaphoreGive(mutexGyro)) {
      ErrorHandler();
      return;
    }

    if (SendCorrectionUpdateCallback != NULL) {
        SendCorrectionUpdateCallback(GYRO_IDX, deltaTms, sGyroXYZAngleDot);
    }
}


void GetGyroAngleDot(float32_t * xAngleDot, float32_t * yAngleDot, float * zAngleDot) {
  if (pdTRUE != xSemaphoreTake(mutexGyro,  portMAX_DELAY /* wait forever */)) {
    ErrorHandler();
    return;
  }

  *xAngleDot = sGyroXYZAngleDot[XDOT_IDX];
  *yAngleDot = sGyroXYZAngleDot[YDOT_IDX];
  *zAngleDot = sGyroXYZAngleDot[ZDOT_IDX];

  if (pdTRUE != xSemaphoreGive(mutexGyro)) {
    ErrorHandler();
    return;
  }
}


void GetGyroAngleDotNoMutex(float32_t * xAngleDot, float32_t * yAngleDot, float * zAngleDot) {
  *xAngleDot = sGyroXYZAngleDot[XDOT_IDX];
  *yAngleDot = sGyroXYZAngleDot[YDOT_IDX];
  *zAngleDot = sGyroXYZAngleDot[ZDOT_IDX];
}


void SetGyroMeasuredSamplePeriod(float32_t measuredPeriod) {
  sGyroSamplePeriod = measuredPeriod;
}


float32_t GetGyroMeasuredSamplePeriod(void) {
  return sGyroSamplePeriod;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
