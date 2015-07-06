#ifndef GYROSCOPE_H
#define GYROSCOPE_H

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
 * @file gyroscope.h
 *
 * This file exports the API to the gyroscope.
 */

/**
 * Initialises
 *
 * @retval DF_OK, error otherwise
 */
uint8_t InitialiseGyroscope(void);


/**
 * This method is intended to be called from the EXTI1 ISR.
 */
void GyroHandleDataReady(void);

#endif
