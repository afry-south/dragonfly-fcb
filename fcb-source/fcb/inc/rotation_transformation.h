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
typedef enum  {
    TRANSF_OK = 1, TRANSF_ERROR = !TRANSF_OK
} TransformationErrorStatus;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void InitRotationMatrix(void);
void InitAngularRotationMatrix(void);
void UpdateRotationMatrix(const float32_t roll, const float32_t pitch, const float32_t yaw);
TransformationErrorStatus UpdateAngularRotationMatrix(const float32_t roll, const float32_t pitch);
void GetAttitudeFromAccelerometer(float32_t* dstAttitude, float32_t const * bodyAccelerometerReadings);
void GetAttitudeFromMagnetometer(float32_t* dstAttitude, float32_t* bodyMagneticReadings);
void Vector3DCrossProduct(float32_t* dstVector, const float32_t* srcVector1, const float32_t* srcVector2);
void Vector3DNormalize(float32_t* dstVector, float32_t* srcVector);
void Vector3DTiltCompensate(float32_t* dstVector, float32_t* srcVector, float32_t roll, float32_t pitch);
float32_t GetAccRollAngle(void);
float32_t GetAccPitchAngle(void);
float32_t GetMagYawAngle(float32_t* magValues, const float32_t roll, const float32_t pitch);
TransformationErrorStatus GetEulerAngularRates(float32_t* rateDst, const float32_t* bodyAngularRates, const float32_t roll, const float32_t pitch);

#endif /* INC_ROTATION_TRANSFORMATION_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
