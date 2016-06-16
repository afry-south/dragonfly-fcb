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

typedef struct {
	uint16_t AC[6];
	uint16_t B[2];
	uint16_t MB;
	uint16_t MC;
	uint16_t MD;
} BMP180CalibVals_t;

void BMP180_init(void);
void BMP180_readCalibVals(BMP180CalibVals_t *calibVals);

void BMP180_StartPressureMeasure(void);
HAL_StatusTypeDef BMP180_ReadPressureValue(float * pData);

#endif /* BMP180_BMP180_H_ */
