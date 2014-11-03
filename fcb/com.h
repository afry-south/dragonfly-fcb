/**
  ******************************************************************************
  * @file    fcb/com.h
  * @author  ÅF Dragonfly - Embedded Systems
  * @version v. 0.0.1
  * @date    30-October-2014
  * @brief   Header for com.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_H
#define __COM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void initUSB(void);
void rwUSB(void);

#endif /* __COM_H */

/* ****END OF FILE****/
