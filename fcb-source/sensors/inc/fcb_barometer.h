/*
 * fcb_barometer.h
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#ifndef INC_FCB_BAROMETER_H_
#define INC_FCB_BAROMETER_H_

#include <stdint.h>
#include <arm_math.h>
#include "fcb_sensors.h"

uint8_t FcbInitialiseBarometer(void);
uint8_t SensorRegisterBaroClientCallback(SendCorrectionUpdateCallback_TypeDef cbk);

void FetchDataFromBarometer(void);
void GetAltitude(float32_t * alt);

#endif /* INC_FCB_BAROMETER_H_ */
