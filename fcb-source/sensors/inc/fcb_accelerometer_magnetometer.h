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

#include "arm_math.h"
#include <stdint.h>

/**
 * Issue 2 & 3:
 *
 * start off by implementing Gauss-Newton in SciLab
 *
 * This means simply print the value to the console,
 * copy them into a file and then run SciLab script to calculate
 * the calibration data.
 *
 * TODO add script name.
 */
#define FCB_SENSORS_SCILAB_CALIB


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
 *
 */
enum FcbAccMagMode {
  ACCMAGMTR_UNINITIALISED = 0,
  ACCMAGMTR_PREFLIGHT = 1,
  ACCMAGMTR_FETCHING = 2, /** fetching data from sensor */
  ACCMAGMTR_CALIBRATING = 3, /** fetching calibration samples */
};


enum { ACCMAG_CALIBRATION_SAMPLES_N = 6 }; /* calibration samplin. TODO increase */



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


/**
 * When this function has been called, GetAcceleration and GetMagVector
 * will return uncalibrated values until the CPU has rebooted or
 * StopAccMagMtrCalibration has been called.
 *
 * @param samples ACCMAG_CALIBRATION_SAMPLES_N <= val <= 250 (data type limitation)
 *
 * @note samples param is intended for future use
 *
 * @see ACCMAG_CALIBRATION_SAMPLES_N
 */
void StartAccMagMtrCalibration(uint8_t samples);


/**
 * When this function has been called, GetAcceleration and GetMagVector
 * will return calibrated values (which is the default).
 *
 * @note this function will not be needed if or when all calibration
 * is done onboard the FCB board, today the calculations are done offline
 * in SciLab.
 */
void StopAccMagMtrCalibration(uint8_t samples);


/*
 * get the current calibrated reading from the accelerometer.
 *
 * It is updated at a rate of 50 Hz (configurable in lsm303dlhc.c).
 *
 * By default, it returns calibrated values, but after StartAccMagMtrCalibration
 * is called, it returns uncalibrated values.
 *
 * The caller allocates memory for input variables.
 */
void GetAcceleration(float32_t * xDotDot, float32_t * yDotDot, float32_t * zDotDot);


/**
 * Fetches data (rotation speed, or angle dot) from accelerometer
 * sensor.
 */
void FetchDataFromMagnetometer(void);


/*
 * get the current reading from the magnetometer.
 *
 * The magnetometer values are updated at a rate
 * of 75 Hz (configurable in lsm303dlhc.c).
 *
 * By default, it returns calibrated values, but after StartAccMagMtrCalibration
 * is called, it returns uncalibrated values.
 *
 *
 * The caller allocates memory for input variables.
 *
 * @see lsm303dlhc.c
 */
void GetMagVector(float32_t * x, float32_t * y, float32_t * z);


/**
 * Print accelerometer values to USB.
 */
void PrintAccelerometerValues(void);


/*
 * Set an updated value of the samplePeriod. The true sample period is not
 * exactly equal to the nominal value. Observed difference is about 2-3 Hz than
 * for the accelerometer and magnetometer. The accelerometer and magnetometer
 * have individual sample rates.
 *
 * @param measuredPeriod this value will be used as its sampling period henceforth
 */
void SetAccMagMeasuredSamplePeriod(float32_t accMeasuredPeriod, float32_t magMeasuredPeriod);

/*
 * This is the current gyro sample period used by the accelerometer and
 * magnetometer code respectively.
 */
void GetAccMagMeasuredSamplePeriod(float32_t * accMeasuredPeriod, float32_t *magMeasuredPeriod);

#endif /* FCB_ACCELEROMETER_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
