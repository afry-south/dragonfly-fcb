/******************************************************************************
 * @file    rotation_transformation.c
 * @brief   Functions to keep track of coordinate system representations and
 * 			transformations between world and body frames. The transformations
 * 			are based on Euler angle rotations (Z-Y-X / roll-pitch-yaw) and
 * 			rotation matrices.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "rotation_transformation.h"

#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
arm_matrix_instance_f32 DCM;
arm_matrix_instance_f32 DCMInv;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes the Direction Cosine Matrix
 * @param  None
 * @retval None
 */
void InitRotationMatrix(void)
{
	/* Initializes the DCM to the unit matrix */
	UpdateRotationMatrix(0.0, 0.0, 0.0);
}

/*
 * @brief  Updates the Direction Cosine Matrix that transforms FROM the inertial frame TO the body frame
 * @param  roll : roll angle in radians
 * @param  pitch : pitch angle in radians
 * @param  yaw : yaw angle in radians
 * @retval None
 */
void UpdateRotationMatrix(float32_t roll, float32_t pitch, float32_t yaw)
{
	/* Calculate the DCM based on roll, pitch and yaw angles */
	float32_t DCMUpdate[9] =
	{
			arm_cos_f32(pitch)*arm_cos_f32(yaw),     arm_cos_f32(pitch)*arm_sin_f32(yaw),     -arm_sin_f32(pitch),
			-arm_cos_f32(roll)*arm_sin_f32(yaw)+arm_sin_f32(roll)*arm_sin_f32(pitch)*arm_cos_f32(yaw),     arm_cos_f32(roll)*arm_cos_f32(yaw)+arm_sin_f32(roll)*arm_sin_f32(pitch)*arm_sin_f32(yaw),     arm_sin_f32(roll)*arm_cos_f32(pitch),
			arm_sin_f32(roll)*arm_sin_f32(yaw)+arm_cos_f32(roll)*arm_sin_f32(pitch)*arm_cos_f32(yaw),     -arm_sin_f32(roll)*arm_cos_f32(yaw)+arm_cos_f32(roll)*arm_sin_f32(pitch)*arm_sin_f32(yaw),     arm_cos_f32(roll)*arm_cos_f32(pitch),
	};

	/* (Re-)init arm_matrix_instance_f32 type */
	arm_mat_init_f32(&DCM, 3, 3, DCMUpdate);

	/* Calculate the DCM inverse, which is the same as matrix transpose since DCM is an orthonormal matrix */
	arm_mat_trans_f32(&DCM, &DCMInv);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
