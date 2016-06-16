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

static xSemaphoreHandle mutexPressure;

static SendCorrectionUpdateCallback_TypeDef SendCorrectionUpdateCallback = NULL;
static BMP180CalibVals_t calibVals;
static float32_t sPressure;

uint8_t FcbInitialiseBarometer(void) {
    uint8_t retVal = FCB_OK;

    if (NULL == (mutexPressure = xSemaphoreCreateMutex())) {
        return FCB_ERR_INIT;
    }

    BMP180_init();
    BMP180_readCalibVals(&calibVals);

    return retVal;
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

void FetchDataFromBarometer(uint8_t deltaTms) {
    float pressureData = 0.0f;
    HAL_StatusTypeDef status = HAL_OK;

    status = BMP180_ReadPressureValue(&pressureData);
    if (status != HAL_OK) {
#ifdef FCB_GYRO_DEBUG
        USBComSendString("ERROR: L3GD20_ReadXYZAngRate\n");
#endif
        FcbSendSensorMessage(FCB_SENSOR_BAR_DATA_READY); // Re-send data ready read request if read fails
        return;
    }

    if (pdTRUE != xSemaphoreTake(mutexPressure,  portMAX_DELAY /* wait forever */)) {
      ErrorHandler();
      return;
    }

    sPressure = pressureData;

    if (pdTRUE != xSemaphoreGive(mutexPressure)) {
      ErrorHandler();
      return;
    }

    if (SendCorrectionUpdateCallback != NULL) {
        SendCorrectionUpdateCallback(BARO_IDX, &sPressure);
    }
}
