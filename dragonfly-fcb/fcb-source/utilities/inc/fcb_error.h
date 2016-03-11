#ifndef FCB_ERROR_H
#define FCB_ERROR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3_discovery.h"

/*
 * This is called when we encounter an error state.
 *
 * It lights up all the LEDs, then goes into infinite loop.
 */
void ErrorHandler(void);
void GetRegistersFromStack(uint32_t* pulFaultStackAddress);

#endif
