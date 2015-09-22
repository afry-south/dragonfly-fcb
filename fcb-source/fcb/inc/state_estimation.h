/****************************************************************************
* @file    state_estimation.h
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
	float roll;
	float rollRateBias;
	float pitch;
	float pitchRateBias;
	float yaw;
	float yawRateBias;
}StateVector_TypeDef;


/* Exported constants --------------------------------------------------------*/
#define ACC_CALIBRATION_SAMPLES		(int)		100
#define MAG_CALIBRATION_SAMPLES    	(int)		2000
#define INIT_SAMPLES			(int)		1000
#define	CONTROL_SAMPLE_PERIOD (float)		0

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
float GetRollAngle(void);
float GetPitchAngle(void);
float GetYawAngle(void);
float GetHeading(void);

void InitStatesXYZ(void);
void PredictStatesXYZ(float newRatesXYZ[]);
void CorrectStatesXYZ(float newAnglesXYZ[]);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
