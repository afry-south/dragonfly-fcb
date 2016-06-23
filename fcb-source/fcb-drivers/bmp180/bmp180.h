/*
 * bmp180.h
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#ifndef BMP180_BMP180_H_
#define BMP180_BMP180_H_

#include <stdint.h>
#include "stm32f3_discovery.h"

void BMP180_init(void);

void BMP180_StartPressureMeasure(void);
void BMP180_StartTemperatureMeasure(void);
HAL_StatusTypeDef BMP180_ReadPressureValue(int32_t * pData);
HAL_StatusTypeDef BMP180_UpdateInternalTempValue();

#endif /* BMP180_BMP180_H_ */
