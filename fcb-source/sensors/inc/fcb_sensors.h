#ifndef FCB_SENSORS_H
#define FCB_SENSORS_H

#include "communication.h"
#include "stm32f3xx_hal.h"

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
  BARO_IDX = 3,
  FCB_SENSOR_NBR = 4 } FcbSensorIndexType;

/* index into
 * gyroscope values as array of angle rates for x y z
 * accelerometer values as array of accelerations for x y z
 * magmnetometer values as array of x y z
 */
typedef enum FcbAxleIndex {
  X_IDX = 0,
  Y_IDX = 1,
  Z_IDX = 2
} FcbAxisIndexType; /* as above */


/**
 * declared public as queue sizes & memory are a resources
 * common to all tasks in this cpu.
 *
 * @todo tune size
 */
enum { FCB_SENSORS_QUEUE_SIZE = FCB_SENSOR_NBR*4 };

#define FCB_SENSORS_Q_MSG_SIZE (sizeof(FcbSensorMsgType))

/**
 * Enumeration must fit in uint8_t
 *
 * @see FcbSensorMsg
 */
typedef enum FcbSensorEvent {
    FCB_SENSOR_GYRO_DATA_READY = 0x0A,
    FCB_SENSOR_ACC_DATA_READY = 0x1A,
    FCB_SENSOR_MAGNETO_DATA_READY = 0x2A
} FcbSensorEventType;

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
 * @param xyz pointer-to-array with XYZ reading for this sensor. Use FcbAxleIndexType to index into array.
 *
 * @see FcbSensorIndexType
 */
typedef void (*SendCorrectionUpdateCallback_TypeDef)(FcbSensorIndexType sensorType, float32_t xyz[3]);


/**
 * Creates a task which is pended on a queue. The tasks runs when FreeRTOS scheduler
 * is launched.
 * @note This function must be called before the scheduler is started.
 *
 * @return see fcb_retval.h
 */
int FcbSensorsConfig(void);

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

/**
 * posts a FcbSensorMessage to the queue which is
 * polled by SENSORS task.
 *
 * @param event see FcbSensorEventType
 */
void FcbSendSensorMessageFromISR(uint8_t event);

// TODO Description
void FcbSendSensorMessage(uint8_t event);

/* Debug Print functions ---------------------------------------------------------*/
void PrintSensorValues(void);
FcbRetValType StartSensorSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
FcbRetValType StopSensorSamplingTask(void);

#endif /* FCB_SENSORS_H */
