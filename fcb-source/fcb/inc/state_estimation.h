/****************************************************************************
* @file    state_estimation.h
* @author  Dragonfly
* @version v. 1.0.0
* @date    2014-09-26
* @brief   Header file for state_estimation.c
*****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	float q1;	// Process noise covariance matrix components
	float q2;
	float r1;	// Measurement noise covariance matrix component
	float p11;	// Error covariance matrix components
	float p12;
	float p21;
	float p22;
	float k1;	// Kalman gains
	float k2;
}KalmanFilter_TypeDef;

typedef struct
{
	float ZVelocity;
	float roll;
	float rollRateBias;
	float pitch;
	float pitchRateBias;
	float yaw;
	float yawRate;
	float yawRateBias;
}StateVector_TypeDef;

/* Exported constants --------------------------------------------------------*/
#define ACC_CALIBRATION_SAMPLES		(int)		100
#define MAG_CALIBRATION_SAMPLES    	(int)		2000
#define INIT_SAMPLES			(int)		1000

#define G_ACC				(float)	   	9.815					/* Gravitational acceleration constant approx. 9.815 m/s^2 in
																 * Smygehuk, Sweden (according to Lantm�teriet) */
#define COMPASS_DECLINATION		   (float)		3.226*PI/180.0	/* For Malmoe, Sweden the compass declination is about 3.226 deg East
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 /* The total field strength is 50552 nT (505.52 mGauss) */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void CompassConfig(void);
void CompassReadMag(volatile float* pfData);
void CompassReadAcc(volatile float* pfData);

void CalibrateGyro(void);
void CalibrateAcc(void);
void CalibrateMag(void);
void InitStateEstimation(void);

float GetRoll(void);
float GetPitch(void);
float GetYawRate(void);
float GetZVelocity(void);
float GetHeading(void);

void UpdateStates(void);
void InitRollEstimator(void);
void UpdateRollEstimate(void);
void InitPitchEstimator(void);
void UpdatePitchEstimate(void);
void UpdateZVelocityEstimate(void);
void UpdateYawRateEstimate(void);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
