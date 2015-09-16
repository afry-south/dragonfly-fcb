/******************************************************************************
 * @file    common.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Header file for common functions
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Macro function that determines if its parameter is positive */
#define IS_POS(X) 						((X) >= 0)

#define IS_NOT_GREATER_UINT16_MAX(X)	((X) <= UINT16_MAX)

/* Exported function prototypes --------------------------------------------- */
void InitCRC(void);
uint32_t CalculateCRC(const uint8_t* dataBuffer, const uint32_t dataBufferSize);
uint16_t UInt16Mean(const uint16_t* buffer, const uint16_t length);
void ConfigPVD(void);
void InitLEDs(void);
void LEDsOff(void);

#endif /* __COMMON_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
