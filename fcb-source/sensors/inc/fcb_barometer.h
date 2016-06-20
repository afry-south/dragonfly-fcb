/*
 * fcb_barometer.h
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#ifndef INC_FCB_BAROMETER_H_
#define INC_FCB_BAROMETER_H_

#include <stdint.h>

uint8_t FcbInitialiseBarometer(void);
void FetchDataFromBarometer(void);

#endif /* INC_FCB_BAROMETER_H_ */
