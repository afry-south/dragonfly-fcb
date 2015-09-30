/******************************************************************************
 * @file    rotation_transformation.h
 * @brief   Header file for coordinate system rotation transformation module
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_ROTATION_TRANSFORMATION_H_
#define INC_ROTATION_TRANSFORMATION_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void InitRotationMatrix(void);
void UpdateRotationMatrix(float32_t roll, float32_t pitch, float32_t yaw);

#endif /* INC_ROTATION_TRANSFORMATION_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
