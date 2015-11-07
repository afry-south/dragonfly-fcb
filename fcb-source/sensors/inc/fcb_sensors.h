#ifndef FCB_SENSORS_H
#define FCB_SENSORS_H

#include "usbd_cdc_if.h"

#include "fcb_retval.h"
#include "arm_math.h"
#include <stdint.h>

/**
 * @file fcb_sensors.h
 *
 * The sensors send Data Ready interrupts and when they are received,
 * the ISRs send a message to the qFcbSensors queue. The SENSORS
 * task is pended on that queue, the message is checked and the
 * task fetches the data from the sensor.
 *
 * The SENSORS task then delegates functionality according
 * to sensor.
 */


typedef enum FcbSensorIndex {
  GYRO_IDX = 0,
  ACC_IDX = 1,
  MAG_IDX = 2,
  FCB_SENSOR_NBR = 3 } FcbSensorIndexType;

/* index into
 * gyroscope values as array of angle rates for x y z
 * accelerometer values as array of accelerations for x y z
 * magmnetometer values as array of x y z
 */
typedef enum FcbAxleIndex {
  X_IDX = 0,
  Y_IDX = 1,
  Z_IDX = 2
} FcbAxleIndexType; /* as above */


/**
 * declared public as queue sizes & memory are a resources
 * common to all tasks in this cpu.
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


/**
 * This is a callback client code registers with a sensor.
 *
 * It will notify the client about which sensor to read.
 *
 * The client callback may switch on the sensor type.
 * For gyro, the xyz values are angular rates around FCB axes
 * for accelerometer, the xyz are accelerations in m/s2 along FCB axes
 * for magnetometer, the xyz are the magnetic vector along FCB axes.
 *
 * @param sensorType: type of sensor
 * @param samplePeriod: sample period for sensor
 * @param xyz pointer-to-array with XYZ reading for this sensor. Use FcbAxleIndexType to index into array.
 *
 * @see FcbSensorIndexType
 */
typedef void (*FcbSensorCbk)(FcbSensorIndexType sensorType, float32_t samplePeriod, float32_t const * xyz);


/**
 * Creates a task which is pended on a queue. The tasks runs when FreeRTOS scheduler
 * is launched.
 * @note This function must be called before the scheduler is started.
 *
 * @return see fcb_retval.h
 */
int FcbSensorsConfig(void);


/**
 * This registers a client callback which will be called when data arrives
 * in the SENSORS task. The callback is NULL per default and one
 * has to be set by client code to receive asynchronous data updates.
 *
 * To delete the existing callback, set it to NULL.
 *
 * @return FCB_OK: callback registered success
 * @return FCB_ERR:  there is already a callback, try registering a NULL cbk first
 */
uint8_t FcbSensorRegisterClientCallback(FcbSensorCbk cbk);


/**
 * This function is intended to be called from the various sensors to
 * give sensor values to our one client callback.
 */
void FcbPush2Client(FcbSensorIndexType sensorType, float32_t samplePeriod, float32_t const * xyz);


/**
 * posts a FcbSensorMessage to the queue which is
 * polled by SENSORS task.
 */
void FcbSendSensorMessageFromISR(uint8_t msg);

void PrintSensorValues(const SerializationType serializationType);
FcbRetValType StartSensorSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
FcbRetValType StopSensorSamplingTask(void);
void SetSensorPrintSamplingSerialization(const SerializationType serializationType);


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
