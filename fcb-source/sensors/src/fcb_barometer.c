/*
 * fcb_barometer.c
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#if !defined(USE_BAROMETER)
#warning "USE_BAROMETER not defined. Barometer not used."
#endif

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

static xTimerHandle barometerTimer;
static CurrentMeasurementType currentMeasurementType;

static SendCorrectionUpdateCallback_TypeDef SendCorrectionUpdateCallback = NULL;
static float32_t sAltitude;

/* Private function prototypes -----------------------------------------------*/

static void InitBarometerTimeEvent(void);
static float32_t CalcAltitudeFromPressure(int32_t pressure);

/* Exported functions --------------------------------------------------------*/

uint8_t FcbInitialiseBarometer(void) {
    uint8_t retVal = FCB_OK;

    BMP180_init();

    currentMeasurementType = TEMPERATURE_MEASUREMENT;
    InitBarometerTimeEvent();
    BMP180_StartTemperatureMeasure();

    return retVal;
}

void vTimerCallback( xTimerHandle pxTimer ) {
	FcbSendSensorMessage(FCB_SENSOR_BAR_DATA_READY);
}

void FetchDataFromBarometer(void) {
    if (currentMeasurementType == PRESSURE_MEASUREMENT) {
        int32_t pressureData = 0;
        BMP180_ReadPressureValue(&pressureData);
        float32_t newAltitude = CalcAltitudeFromPressure(pressureData);

        if (SendCorrectionUpdateCallback != NULL) {
            SendCorrectionUpdateCallback(BARO_IDX, &newAltitude);
        }

        sAltitude = newAltitude;

        // Start a new temperature measurement.
		currentMeasurementType = TEMPERATURE_MEASUREMENT;
        BMP180_StartTemperatureMeasure();
	}
	else if (currentMeasurementType == TEMPERATURE_MEASUREMENT) {
		BMP180_UpdateInternalTempValue();
		// Start a new pressure measurement.
		currentMeasurementType = PRESSURE_MEASUREMENT;
        BMP180_StartPressureMeasure();
	}
}

uint8_t SensorRegisterBaroClientCallback(SendCorrectionUpdateCallback_TypeDef cbk) {
    if (NULL != SendCorrectionUpdateCallback) {
        return FCB_ERR;
    }

    SendCorrectionUpdateCallback = cbk;

    return FCB_OK;
}

void GetAltitude(float32_t * alt) {
    *alt = sAltitude;
}

/* Private functions ---------------------------------------------------------*/

static void InitBarometerTimeEvent(void) {
	barometerTimer = xTimerCreate((signed char *)"BarometerTimer",         // Just a text name, not used by the kernel.
			                      FCB_SENSOR_BAR_PERIOD,                   // The timer period in ticks.
                                  pdTRUE,                                  // The timer will auto-reload it selves when expired.
                                  (void *)PRESSURE_MEASUREMENT,            // Assign timer a unique id.
                                  vTimerCallback                           // Timer calls this callback when it expires.
                                  );

	if (barometerTimer == NULL ) {
		ErrorHandler();
    }

    if (xTimerStart(barometerTimer, 0) != pdPASS ) {
      	ErrorHandler();
    }
}

static float32_t CalcAltitudeFromPressure(int32_t pressure) {
	float32_t tmp = powf((float32_t)pressure / 101325.0f, 1.0f/5.255f);
	float32_t altitude = 44330.0f * ( 1.0f - tmp);

	return altitude;
}
