/**
 * @file fcb_sensors.c
 *
 * Implementation of interface publicised in fcb_sensors.h
 *
 * @see fcb_sensors.h
 */

#include "fcb_sensors.h"
#include "gyroscope.h"
#include "fcb_error.h"
#include "fcb_retval.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#define FCB_SENSORS_DEBUG /* todo delete */

#ifdef FCB_SENSORS_DEBUG
static uint32_t cbk_gyro_counter = 0;
#endif

/* static data declarations */

static xTaskHandle tFcbSensors;
static xQueueHandle qFcbSensors = NULL;

static void ProcessSensorValues(void*);

/* static fcn declarations */

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
                   xTaskCreate((pdTASK_CODE)ProcessSensorValues,
                               (signed portCHAR*)"tFcbSensors",
                               4 * configMINIMAL_STACK_SIZE,
                               NULL /* parameter */,
                               1 /* priority */,
                               &tFcbSensors))) {
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

void FcbSendSensorMessageFromISR(uint8_t msg) {
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
#ifdef FCB_SENSORS_DEBUG
    if ((cbk_gyro_counter % 48) == 0) {
        BSP_LED_Toggle(LED5);
    }
    cbk_gyro_counter++;
#endif

    if (pdTRUE != xQueueSendFromISR(qFcbSensors, &msg, &higherPriorityTaskWoken)) {
    	ErrorHandler();
    }

    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}


static void ProcessSensorValues(void* val) {
	/*
	 * configures the sensors to start giving Data Ready interrupts
	 * and then polls the queue in an infinite loop
	 */
    uint8_t msg;

    if (FCB_OK != InitialiseGyroscope()) {
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

        switch (msg) {
            case FCB_SENSOR_GYRO_DATA_READY:
                /*
                 * As settings are in BSP_GYRO_Init, the callback is called with a frequency
                 * of 94.5 Hz according to oscilloscope.
                 */
                FetchAngleDotFromGyroscope();
                break;
            case FCB_SENSOR_GYRO_CALIBRATE:
                break;
            case FCB_SENSOR_MAGNETO_ACC_DATA_READY:
                break;
            case FCB_SENSOR_MAGNETO_ACC_CALIBRATE:
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

/* static fcn definitions */
