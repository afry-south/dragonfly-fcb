#ifndef FCB_SENSORS_H
#define FCB_SENSORS_H

#include "usbd_cdc_if.h"

#include "fcb_retval.h"
#include <stdint.h>

/**
 * @file fcb_retval.h
 *
 * The sensors send Data Ready interrupts and when they are received,
 * the ISRs send a message to the qFcbSensors queue. The tFcbSensors
 * thread is pended on that queue, the message is checked and the
 * thread fetches the data from the sensor.
 *
 * The tFcbSensors thread then delegates functionality according
 * to sensor.
 */

/**
 * declared public as queue sizes & memory are a resources
 * common to all threads in this cpu.
 *
 * @todo tune size
 */
enum { FCB_SENSORS_QUEUE_SIZE = 5 };

/**
 * The messages are sent as uin8_t under the assumption that
 * less than 255 different kinds of messages will ever be
 * needed.
 */
enum { FCB_SENSORS_Q_MSG_SIZE = 1 };

enum FcbSensorMessage {
    FCB_SENSOR_GYRO_DATA_READY = 0x0A,
    FCB_SENSOR_GYRO_CALIBRATE = 0x0B,
    FCB_SENSOR_ACC_DATA_READY = 0x1A,
    FCB_SENSOR_ACC_CALIBRATE = 0x1B,
    FCB_SENSOR_MAGNETO_DATA_READY = 0x2A,
    FCB_SENSOR_MAGNETO_CALIBRATE = 0x2B,
};

typedef enum {
	SENSORS_ERROR = 0, SENSORS_OK = !SENSORS_ERROR
} SensorsReturnCode;


/**
 * Creates a thread which is pended on a queue. The threads runs when FreeRTOS scheduler
 * is launched.
 * @note This function must be called before the scheduler is started.
 *
 * @return see fcb_retval.h
 */
int FcbSensorsConfig(void);



/**
 * posts a FcbSensorMessage to the queue which is
 * polled by tFcbSensors thread.
 */
void FcbSendSensorMessageFromISR(uint8_t msg);

void PrintSensorValues(const SerializationType_TypeDef serializationType);
SensorsReturnCode StartSensorSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
SensorsReturnCode StopSensorSamplingTask(void);
void SetSensorPrintSamplingSerialization(const SerializationType_TypeDef serializationType);

/**
 * Wrapper fcn for enabling GPIO pin with default settings for receiving
 * interrupts by calling HAL_GPIO_Init.
 *
 * @param  GPIOx: where x can be (A..F) to select the GPIO peripheral
 * @param pin the pin to enable.
 *
 * @todo
 */
void FcbSensorsInitGpioPinForInterrupt(GPIO_TypeDef  *GPIOx, uint32_t pin);

#endif /* FCB_SENSORS_H */
