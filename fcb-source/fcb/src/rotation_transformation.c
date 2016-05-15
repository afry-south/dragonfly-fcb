/******************************************************************************
 * @brief   Functions to keep track of coordinate system representations and
 * 			transformations between world and body frames. The transformations
 * 			are based on Euler angle rotations (Z-Y-X / roll-pitch-yaw) and
 * 			rotation matrices.
 *
  * @license
 * Dragonfly FCB firmware to control the Dragonfly quadrotor UAV
 * Copyright (C) 2016  ÅF Technology South: Dragonfly Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 *****************************************************************************/

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

/* [Unit: Gauss] Set to the magnetic vector in Malmö, SE, year 2015 (Components in north, east, down convention)
* Data used from http://www.ngdc.noaa.gov/geomag-web/
* TODO: Optionally, perform calibration of this vector at system startup */
// float32_t inertialMagneticVector[3] = {0.171045, 0.01055, 0.472443};
float32_t inertialMagneticVectorNormalized[3] = {0.340345, 0.0209924, 0.940066};

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes the Direction Cosine Matrix
 * @param  None
 * @retval None
 */
void InitRotationMatrix(void) {
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
void UpdateRotationMatrix(const float32_t roll, const float32_t pitch, const float32_t yaw) {
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
 * @brief  Calculates the attitude (roll and pitch, NOT yaw) based on accelerometer input, assuming
 * 	  	   that only gravity influences the accelerometer sensor readings
 * @param  dstAttitude : Destination vector in which to store the calculated attitude (roll and pitch)
 * @param  bodyAccelerometerReadings : The accelerometer sensor readings in the UAV body-frame
 * @retval None
 */
void GetAttitudeFromAccelerometer(float32_t* dstAttitude, float32_t const * bodyAccelerometerReadings) {
  float32_t accNormalized[3];

  /* Get unit length normalized version of accelerometer readings vector */
  Vector3DNormalize(accNormalized, (float32_t*) bodyAccelerometerReadings);

  /* Calculate roll and pitch Euler angles  */
  dstAttitude[0] = atan2f(-accNormalized[1], -accNormalized[2]); // Roll-Phi need sign on both params to get right section of unit circle
  dstAttitude[1] = asinf(accNormalized[0]); // Pitch-Theta (direction of g and minus sign cancel)
}

/*
 * @brief  Calculates the attitude (roll, pitch, yaw angles) based on magnetometer input
 * @note   This function does not work very well, but can't find anything wrong with the implementation
 * @param  dstAttitude : Destination vector in which to store the calculated attitude (roll, pitch, yaw)
 * @param  bodyMagneticReadings : The magnetometer sensor readings in the UAV body-frame
 * @retval None
 */
void GetAttitudeFromMagnetometer(float32_t* dstAttitude, float32_t* bodyMagneticReadings) {

	/* Calculate the axis/angle representing the rotation from inertial-frame magnetic field to the body-frame sensor
	 * readings. NOTE: Inertial magnetic field vector depends on where on earth UAV is operating - Malmö, SE assumed.
	 * */

	float32_t bodyMagneticVectorNormalized[3], rotationAxisVector[3], rotationAxisVectorNormalized[3];
	float32_t rotationAngle, dotProd, cosHalfAngle, sinHalfAngle, q0, q1, q2, q3;

	/* Get unit length normalized versions of the vectors */
	Vector3DNormalize(bodyMagneticVectorNormalized, bodyMagneticReadings);

	/* Get the rotation axis vector between the two magnetic vectors (inertial and body) */
	Vector3DCrossProduct(rotationAxisVector, bodyMagneticVectorNormalized, inertialMagneticVectorNormalized);
	Vector3DNormalize(rotationAxisVectorNormalized, rotationAxisVector);

	/* Get the angle for the body to the inertial frame vectors rotated around rotation vector axis */
	arm_dot_prod_f32(bodyMagneticVectorNormalized, inertialMagneticVectorNormalized, 3, &dotProd);
	rotationAngle = acosf(dotProd);

	/* Calculate the axis/angle quaternion representation (q = q0 + q1*i + q2*j + q3*k) */
	cosHalfAngle = arm_cos_f32(rotationAngle*0.5);
	sinHalfAngle = arm_sin_f32(rotationAngle*0.5);
	q0 = cosHalfAngle;
	q1 = rotationAxisVectorNormalized[0]*sinHalfAngle;
	q2 = rotationAxisVectorNormalized[1]*sinHalfAngle;
	q3 = rotationAxisVectorNormalized[2]*sinHalfAngle;

	/* From the quaternion, the Euler angles (roll, pitch, yaw) are obtained */
	dstAttitude[0] = atan2f(2.0*q0*q1 + 2.0*q2*q3, q0*q0 - q1*q1 - q2*q2 + q3*q3); // Roll-Phi
	dstAttitude[1] = asinf(2.0*q0*q2 - 2.0*q1*q3); // Pitch-Theta
	dstAttitude[2] = atan2f(2.0*q0*q3 + 2.0*q1*q2, q0*q0 + q1*q1 - q2*q2 - q3*q3); // Yaw-Psi
}

/*
 * @brief  Calculates the cross product of two 3D vectors and stores the result in a third
 * @param  dstVector : Destination vector
 * @param  srcVector1 : Source vector 1
 * @param  srcVector2 : Source vector 2
 * @retval None
 */
void Vector3DCrossProduct(float32_t* dstVector, const float32_t* srcVector1, const float32_t* srcVector2) {
	dstVector[0] = srcVector1[1]*srcVector2[2] - srcVector1[2]*srcVector2[1];
	dstVector[1] = srcVector1[2]*srcVector2[0] - srcVector1[0]*srcVector2[2];
	dstVector[2] = srcVector1[0]*srcVector2[1] - srcVector1[1]*srcVector2[0];;
}

/*
 * @brief  Normalizes the input 3D vector to unit length
 * @param  dstVector : normalized vector result
 * @param  srcVector : vector to be normalized
 * @retval None
 */
void Vector3DNormalize(float32_t* dstVector, float32_t* srcVector) {
	float32_t norm;

	/* Calculate norm of source vector */
	arm_dot_prod_f32(srcVector, srcVector, 3, &norm);
	norm = sqrt(norm);

	/* Normalize the vector to unit length and store it in destination vector */
	arm_scale_f32(srcVector, 1.0/norm, dstVector, 3);
}

/*
 * @brief  Tilt compensate the input 3D vector of magnetic values
 * @param  dstVector : tilt compensated vector result
 * @param  srcVector : normalized vector with magnetic sensor values
 * @retval None
 */
void Vector3DTiltCompensate(float32_t* dstVector, float32_t* srcVector, float32_t roll, float32_t pitch) {
	dstVector[0] = srcVector[0]*arm_cos_f32(pitch)
                 + srcVector[2]*arm_sin_f32(pitch);
	dstVector[1] = srcVector[0]*arm_sin_f32(roll)*arm_sin_f32(pitch)
                 + srcVector[1]*arm_cos_f32(roll)
                 - srcVector[2]*arm_sin_f32(roll)*arm_cos_f32(pitch);
	dstVector[2] = -srcVector[0]*arm_cos_f32(roll)*arm_sin_f32(pitch)
                 + srcVector[1]*arm_sin_f32(roll)
                 + srcVector[2]*arm_cos_f32(roll)*arm_cos_f32(pitch);
}

/*
 * @brief  Returns the yaw angle calculated from magnetometer values with tilt (roll/pitch) compensation
 * @note   TODO The tilt-compensation could also be performed with the (previous) state values instead of accelerometer angle values
 * @param  magValues : Vector containing 3D magnetometer values
 * @param  roll : roll angle in radians (kalman estimated or from accelerometer)
 * @param  pitch : pitch angle in radians (kalman estimated or from accelerometer)
 * @retval yawAngle : yaw angle in radians
 */
float32_t GetMagYawAngle(float32_t* magValues, const float32_t roll, const float32_t pitch)
{
	float32_t normalizedMag[3];
	float32_t yawAngle;

	Vector3DNormalize(normalizedMag, magValues);
	Vector3DTiltCompensate(normalizedMag, normalizedMag, roll, pitch);

    /* Equation found in LSM303DLH Application Note document. Minus sign in first parameter in atan2 to get correct rotation direction around Z axis ("down") */
    yawAngle = atan2f(-normalizedMag[1], normalizedMag[0]);

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
