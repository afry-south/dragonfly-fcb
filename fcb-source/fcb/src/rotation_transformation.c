/******************************************************************************
 * @file    rotation_transformation.c
 * @brief   Functions to keep track of coordinate system representations and
 * 			transformations between world and body frames. The transformations
 * 			are based on Euler angle rotations (Z-Y-X / roll-pitch-yaw) and
 * 			rotation matrices.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "rotation_transformation.h"
#include "fcb_accelerometer_magnetometer.h"
#include "trace.h"

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
	float32_t DCMInvInit[9] =
	{
			1,     0,     1,
			0,     1,     0,
			0,     0,     1,
	};

	/* Initialize the inverse DCM to the unit matrix (3x3) */
	arm_mat_init_f32(&DCMInv, 3, 3, DCMInvInit);

	/* Initializes the DCM to the unit matrix (3x3) */
	UpdateRotationMatrix(0.0, 0.0, 0.0);
}

/*
 * @brief  Updates the Direction Cosine Matrix
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

	/* Init the DCM that transforms FROM the inertial frame TO the body frame */
	arm_mat_init_f32(&DCM, 3, 3, DCMUpdate);

	/* Calculate the DCM inverse, which is the same as matrix transpose since DCM is an orthonormal matrix. The inverse
	 * transforms FROM the body frame TO the inertial frame*/
	arm_mat_trans_f32(&DCM, &DCMInv);
}

/*
 * @brief	Returns the roll angle calculated from accelerometer values.
 * @param	None
 * @retval	rollAngle: roll angle in radians.
 */
float32_t GetAccRollAngle(void)
{
	float32_t accX;
	float32_t accY;
	float32_t accZ;
	float32_t rollAngle;

	GetAcceleration(&accX, &accY, &accZ);
	rollAngle = atan2(accY, accZ);

	//trace_printf("The rollAngle is: %1.3f radians.\n", rollAngle);
	return rollAngle;
}

/*
 * @brief	Returns the pitch angle calculated from accelerometer values.
 * @param	None
 * @retval	pitchAngle: pitch angle in radians.
 */
float32_t GetAccPitchAngle(void)
{
	float32_t accX;
	float32_t accY;
	float32_t accZ;
	float32_t pitchAngle;

	GetAcceleration(&accX, &accY, &accZ);
	pitchAngle = atan2(accX, accZ);


	//trace_printf("The pitchAngle is: %1.3f radians.\n", pitchAngle);
	return pitchAngle;
}

/*
 * @brief	Returns the yaw angle calculated from magnetometer values.
 * @param	roll: roll angle in radians (kalman estimated or from acc)
 * @param	pitch: pitch angle in radians (kalman estimated or from acc)
 * @retval	yawAngle: yaw angle in radians.
 */
float32_t GetMagYawAngle(const float32_t roll, const float32_t pitch)
{
	float32_t magX;
	float32_t magY;
	float32_t magZ;
	float32_t yawAngle;

	GetMagVector(&magX, &magY, &magZ);

	/* Equation found at https://www.pololu.com/file/download/...?file_id=0J434 */
	yawAngle = atan2(magX*arm_sin_f32(roll)*arm_sin_f32(pitch)+magY*arm_cos_f32(roll)-magZ*arm_sin_f32(roll)*arm_cos_f32(pitch),
						magX*arm_cos_f32(pitch)+magZ*arm_sin_f32(pitch));

	//trace_printf("The yawAngle is: %1.3f radians.\n", yawAngle);
	return yawAngle;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
