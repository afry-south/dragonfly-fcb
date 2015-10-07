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
void UpdateRotationMatrix(const float32_t roll, const float32_t pitch, const float32_t yaw);
void GetAttitudeFromMagnetometer(float32_t* dstAttitude, float32_t* bodyMagneticReadings);
void Vector3DCrossProduct(float32_t* dstVector, const float32_t* srcVector1, const float32_t* srcVector2);
void Vector3DNormalize(float32_t* dstVector, float32_t* srcVector);

#endif /* INC_ROTATION_TRANSFORMATION_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
