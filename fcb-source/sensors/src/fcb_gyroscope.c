/******************************************************************************
 * @file    gyroscope.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-26
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
// #define FCB_GYRO_DEBUG

/* static & local declarations */

enum { GYROSCOPE_OFFSET_SAMPLES = 100 };

enum { XDOT_IDX = 0 }; /* index of sGyroXYZAngleDot & ditto Offset */
enum { YDOT_IDX = 1 }; /* as above */
enum { ZDOT_IDX = 2 }; /* as above */

const float32_t GYRO_X_AXIS_VARIANCE = 0.098603;
const float32_t GYRO_Y_AXIS_VARIANCE = 0.104274;
const float32_t GYRO_Z_AXIS_VARIANCE = 0.103256;
const float32_t GYRO_AXIS_VARIANCE_ROUGH = 0.000256;


/**
 * Angular velocities, in degrees.
 *
 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile float32_t sGyroXYZAngleDot[3] = { 0.0, 0.0, 0.0 };
static volatile float32_t sGyroXYZAngleDotOffset[3] = { 0.0, 0.0, 0.0 };
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

    FetchDataFromGyroscope(); /* necessary so a fresh DRDY can be triggered */
    return retVal;
}


void FetchDataFromGyroscope(void) {
  float gyroscopeData[3] = { 0.0f, 0.0f, 0.0f };

  /* paranoia - this is used for calculations to reduce the time this function
   * needs to hold the mutexMag
   */
  float lGyroXYZAngleDot[3] = { 0.0f, 0.0f, 0.0f };
#ifdef FCB_GYRO_DEBUG
    static uint32_t call_counter = 0;
#else
    static uint16_t call_counter = 0;
#endif
    /* returns rad/s */
    L3GD20_ReadXYZAngRate(gyroscopeData);

    /* see "Sensors" wiki page for gyroscope vs Quadcopter axes orientations */
    lGyroXYZAngleDot[XDOT_IDX] = - gyroscopeData[YDOT_IDX];
    lGyroXYZAngleDot[YDOT_IDX] = - gyroscopeData[XDOT_IDX];
    lGyroXYZAngleDot[ZDOT_IDX] = - gyroscopeData[ZDOT_IDX];

    if (GYROSCOPE_OFFSET_SAMPLES > call_counter) {
    	sGyroXYZAngleDotOffset[XDOT_IDX] += lGyroXYZAngleDot[XDOT_IDX];
    	sGyroXYZAngleDotOffset[YDOT_IDX] += lGyroXYZAngleDot[YDOT_IDX];
    	sGyroXYZAngleDotOffset[ZDOT_IDX] += lGyroXYZAngleDot[ZDOT_IDX];
        call_counter++;
    } else if (GYROSCOPE_OFFSET_SAMPLES  == call_counter) {
    	sGyroXYZAngleDotOffset[XDOT_IDX] = sGyroXYZAngleDotOffset[XDOT_IDX] / GYROSCOPE_OFFSET_SAMPLES;
    	sGyroXYZAngleDotOffset[YDOT_IDX] = sGyroXYZAngleDotOffset[YDOT_IDX] / GYROSCOPE_OFFSET_SAMPLES;
    	sGyroXYZAngleDotOffset[ZDOT_IDX] = sGyroXYZAngleDotOffset[ZDOT_IDX] / GYROSCOPE_OFFSET_SAMPLES;
        call_counter++;
    } else {
      /* TODO apply calibration
       *
       * as things are, the gyroscope drifts in time. Even taking 200 samples
       * at startup and then using this to calculate an offset will not
       * produce zero drift.
       */
    	lGyroXYZAngleDot[XDOT_IDX] = lGyroXYZAngleDot[XDOT_IDX] - sGyroXYZAngleDotOffset[XDOT_IDX];
    	lGyroXYZAngleDot[YDOT_IDX] = lGyroXYZAngleDot[YDOT_IDX] - sGyroXYZAngleDotOffset[YDOT_IDX];
    	lGyroXYZAngleDot[ZDOT_IDX] = lGyroXYZAngleDot[ZDOT_IDX] - sGyroXYZAngleDotOffset[ZDOT_IDX];

#ifdef FCB_GYRO_DEBUG
    	if (call_counter % 200) {
    		TRACE_SYNC("tim:%u sum xyzdot:%1.1f, %1.1f, %1.1f\n",
    				(uint32_t) call_counter,
    				sGyroXYZAngleDot[XDOT_IDX],
    				sGyroXYZAngleDot[YDOT_IDX],
    				sGyroXYZAngleDot[ZDOT_IDX]);
    	}
    	call_counter++;
#endif
    }

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

    FcbPush2Client(GYRO_IDX, sGyroSamplePeriod, sGyroXYZAngleDot);
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
