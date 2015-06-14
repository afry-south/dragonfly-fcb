/******************************************************************************
 * @file    fcb/common.c
 * @author  ÅF Dragonfly
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

/* Exported function prototypes --------------------------------------------- */
uint32_t Calculate_CRC(const uint8_t* dataBuffer, const uint32_t dataBufferSize);
uint16_t UInt16_Mean(const uint16_t* buffer, const uint16_t length);

void Init_LEDs(void);
void LEDs_Off(void);
void ToggleLEDs(void);

#endif /* __COMMON_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
