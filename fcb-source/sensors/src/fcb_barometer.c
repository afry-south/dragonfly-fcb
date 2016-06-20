/*
 * fcb_barometer.c
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#include "fcb_barometer.h"
#include "bmp180.h"
#include "fcb_sensors.h"
#include "fcb_error.h"

#include "fcb_retval.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include <stdbool.h>

/* Private define ------------------------------------------------------------*/

// Define the period for starting and reading new pressure values in mSec.
// Note that the actual period set to the timer will be half of this as also one
// temperature reading will done for every pressure reading. Minimum time for both
// temperature and pressure readings are 4.5 mSec each (without any filters).
#define FCB_SENSOR_BAR_PERIOD	10

/* Private typedef -----------------------------------------------------------*/

typedef enum {
	PRESSURE_MEASUREMENT = 0x30,
	TEMPERATURE_MEASUREMENT = 0x31
} CurrentMeasurementType;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static xSemaphoreHandle barometerMutex;
static xTimerHandle barometerTimer;

//static SendCorrectionUpdateCallback_TypeDef SendCorrectionUpdateCallback = NULL;
static float32_t sPressure;

/* Private function prototypes -----------------------------------------------*/

static void InitBarometerTimeEvent(CurrentMeasurementType measurmentType);
void FetchDataFromBarometer(void);

/* Exported functions --------------------------------------------------------*/

uint8_t FcbInitialiseBarometer(void) {
    uint8_t retVal = FCB_OK;

    if (NULL == (barometerMutex = xSemaphoreCreateMutex())) {
        return FCB_ERR_INIT;
    }

    BMP180_init();

    InitBarometerTimeEvent(TEMPERATURE_MEASUREMENT);
    BMP180_StartTemperatureMeasure();

    return retVal;
}

void vTimerCallback( xTimerHandle pxTimer ) {
	uint32_t timerID = (uint32_t)pvTimerGetTimerID( pxTimer );
	if (timerID == PRESSURE_MEASUREMENT) {
		FetchDataFromBarometer();
		// Start a new temperature measurement.
        InitBarometerTimeEvent(TEMPERATURE_MEASUREMENT);
        BMP180_StartTemperatureMeasure();
	}
	else if (timerID == TEMPERATURE_MEASUREMENT) {
		BMP180_UpdateInternalTempValue();
		// Start a new pressure measurement.
        InitBarometerTimeEvent(PRESSURE_MEASUREMENT);
        BMP180_StartPressureMeasure();
	}
}

//uint8_t SensorRegisterBaroClientCallback(SendCorrectionUpdateCallback_TypeDef cbk) {
//  if (NULL != SendCorrectionUpdateCallback) {
//    return FCB_ERR;
//  }
//
//  SendCorrectionUpdateCallback = cbk;
//
//  return FCB_OK;
//}

/* Private functions ---------------------------------------------------------*/

void FetchDataFromBarometer(void) {
    float pressureData = 0.0f;
    HAL_StatusTypeDef status = HAL_OK;

    status = BMP180_ReadPressureValue(&pressureData);

    if (pdTRUE != xSemaphoreTake(barometerMutex,  portMAX_DELAY /* wait forever */)) {
      ErrorHandler();
      return;
    }

    sPressure = pressureData;

    if (pdTRUE != xSemaphoreGive(barometerMutex)) {
      ErrorHandler();
      return;
    }

//    if (SendCorrectionUpdateCallback != NULL) {
//        SendCorrectionUpdateCallback(BARO_IDX, pressureData);
//    }
}

static void InitBarometerTimeEvent(CurrentMeasurementType measurmentType) {
	barometerTimer = xTimerCreate((signed char *)"BarometerTimer",         // Just a text name, not used by the kernel.
			                      FCB_SENSOR_BAR_PERIOD,                   // The timer period in ticks.
                                  pdFALSE, //pdTRUE,                       // The timer will auto-reload it selves when expired.
                                  (void *)measurmentType,                  // Assign timer a unique id.
                                  vTimerCallback                           // Timer calls this callback when it expires.
                                  );

	if (barometerTimer == NULL ) {
		ErrorHandler();
    }
    else {
        if (xTimerStart(barometerTimer, 0) != pdPASS ) {
        	ErrorHandler();
        }
    }
}

