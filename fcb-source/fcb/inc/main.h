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
#include "stm32f3xx.h"
#include "stm32f3_discovery.h"
#include "stm32f3xx_hal_conf.h"

#include "usbd_cdc_if.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Error_Handler(void);

#endif /* __MAIN_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
