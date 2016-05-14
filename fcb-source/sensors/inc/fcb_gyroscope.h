#ifndef GYROSCOPE_H
#define GYROSCOPE_H
/**
 * @file fcb_gyroscope.h
 *
 * This file exports the API to the L3GD20 gyroscope.
 */

#include "fcb_retval.h"
#include "stm32f3_discovery.h"
#include "arm_math.h"
#include "fcb_sensors.h"


/**
 * sensor variance as measured with GyroVariance.sce SciLab script tool
 */
extern const float32_t GYRO_X_AXIS_VARIANCE;
extern const float32_t GYRO_Y_AXIS_VARIANCE;
extern const float32_t GYRO_Z_AXIS_VARIANCE;
extern const float32_t GYRO_AXIS_VARIANCE_ROUGH;

/**
 * The Data Ready input from the gyro sensor.
 *
 * This definition is intended to be used in the
 * HAL_GPIO_EXTI_Callback function.
 */
#define GPIO_GYRO_DRDY GPIO_PIN_1


/**
 * Initialises gyroscope.
 *
 * @retval FCB_OK, error otherwise
 */
uint8_t InitialiseGyroscope(void);

uint8_t SensorRegisterGyroClientCallback(SendCorrectionUpdateCallback_TypeDef cbk);


/**
 * Fetches data (rotation speed, or angle dot) from gyroscope
 * sensor.
 * @param deltaTms time in ms since previous DRDY
 */
void FetchDataFromGyroscope(uint8_t deltaTms);

/*
 * get the current reading from the gyroscope.
 *
 * It is updated at a rate of 94.5Hz (configurable).
 */
void GetGyroAngleDot(float32_t * xAngleDot, float32_t * yAngleDot, float32_t * zAngleDot);

/*
 * As GetGyroAngleDot but does not take/release mutex, OK to use in
 * FcbSensorCbk functions which are called in the SENSRS task context
 */
void GetGyroAngleDotNoMutex(float32_t * xAngleDot, float32_t * yAngleDot, float32_t * zAngleDot);


/*
 * Set an updated value of the samplePeriod. The true sample period is not
 * exactly equal to the nominal value. Observed difference is less than
 * 1Hz for the gyro.
 *
 * @param measuredPeriod this value will be used as its sampling period henceforth
 */
void SetGyroMeasuredSamplePeriod(float32_t measuredPeriod);

/*
 * This is the current gyro sample period used by the sensors code.
 */
float32_t GetGyroMeasuredSamplePeriod(void);

#endif /* GYROSCOPE_H */
