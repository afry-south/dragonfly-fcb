#ifndef GYROSCOPE_H
#define GYROSCOPE_H
/**
 * @file gyroscope.h
 *
 * This file exports the API to the L3GD20 gyroscope.
 */

#include "fcb_retval.h"
#include "stm32f3_discovery.h"

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


/**
 * Fetches data (rotation speed, or angle dot) from gyroscope
 * sensor.
 */
void FetchAngleDotFromGyroscope(void);

/*
 * get the current reading from the gyroscope.
 *
 * It is updated at a rate of 94.5Hz (configurable).
 */
void GetAngleDot(float * xAngleDot, float * yAngleDot, float * zAngleDot);

void PrintGyroscopeValues(void);

#endif /* GYROSCOPE_H */
