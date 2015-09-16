#ifndef FCB_ACCELEROMETER_H
#define FCB_ACCELEROMETER_H
/**
 * @file fcb_accelerometer_magnetometer.h
 *
 * Implements an interface to the accelerometer part of the LSM303DLHC
 * combined magnetometer & accelerometer.
 *
 * @see fcb_magnetometer.h
 */

#include "fcb_retval.h"
#include "stm32f3_discovery.h"

/**
 * The Data Ready input from the magnetometer.
 *
 * This definition is intended to be used in the
 * HAL_GPIO_EXTI_Callback function.
 */
#define GPIO_MAGNETOMETER_DRDY GPIO_PIN_2

/**
 * The Data Ready input from the magnetometer.
 *
 * This definition is intended to be used in the
 * HAL_GPIO_EXTI_Callback function.
 */
#define GPIO_ACCELEROMETER_DRDY GPIO_PIN_4


/**
 * Initialises
 *
 * @retval FCB_OK, error otherwise
 */
uint8_t FcbInitialiseAccMagSensor(void);


/**
 * Fetches data (rotation speed, or angle dot) from accelerometer
 * sensor.
 */
void FetchDataFromAccelerometer(void);

/*
 * get the current reading from the accelerometer.
 *
 * It is updated at a rate of 50 Hz (configurable).
 */
void GetAcceleration(int16_t * xDotDot, int16_t * yDotDot, int16_t * zDotDot);


/**
 * This method is intended to be called from the EXTI1 ISR.
 *
 * It won't get called until InitialiseAccMag has returned
 * success.
 *
 * It reads the accelerometer values.
 */
void MagnetometerHandleDataReady(void);


/**
 * Fetches data (rotation speed, or angle dot) from accelerometer
 * sensor.
 */
void FetchDataFromMagnetometer(void);

/*
 * get the current reading from the accelerometer.
 *
 * It is updated at a rate of 50 Hz (configurable).
 */
void GetAcceleration(int16_t * xDotDot, int16_t * yDotDot, int16_t * zDotDot);

/**
 * Print accelerometer values to USB, intended to be used from the
 * sensor sampling task.
 */
void PrintAccelerometerValues(void);

/**
 * ditto magnetometer
 * @see PrintAccelerometerValues
 */
void PrintMagnetometerValues(void);


#endif /* FCB_ACCELEROMETER_H */
