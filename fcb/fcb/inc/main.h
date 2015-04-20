/**
  ******************************************************************************
  * @file    fcb/main.h
  * @author  ÅF Dragonfly - Embedded Systems
  * @version v. 0.0.1
  * @date    30-October-2014
  * @brief   Header for main.c module
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
__IO uint32_t GetUserButton(void);
void ResetUserButton(void);
void Delay(uint32_t mTime);
void Millisecond_Update(void);

#endif /* __MAIN_H */

/* ****END OF FILE****/
