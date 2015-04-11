/****************************************************************************
* @file    fcb/sensors.h
* @author  ÅF Dragonfly - Daniel Stenberg, Embedded Systems
* @version v. 0.0.1
* @date    2014-09-26
* @brief   Header file for sensors.c
*****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

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

/* Private define ------------------------------------------------------------*/
#define L3G_Sensitivity_250dps		(float)   	114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     	(float)    	57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    	(float)    	14.285f	      	/*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

#define LSM_Acc_Sensitivity_2g     	(float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     	(float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     	(float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    	(float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define GYRO_CALIBRATION_SAMPLES   	(int)	   	500

#define ACC_CALIBRATION_SAMPLES		(int)		100

#define MAG_CALIBRATION_SAMPLES    	(int)		2000

#define INIT_SAMPLES			(int)		1000

#define G_ACC				(float)	   	9.815			/* Gravitational acceleration constant approx. 9.815 m/s^2 in
																 * Smygehuk, Sweden (according to Lantmäteriet) */
#define COMPASS_DECLINATION		   (float)		3.226*PI/180.0	/* For Malmö, Sweden the compass declination is about 3.226 deg East
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * The total field strength is 50552 nT (505.52 mGauss) */

/* Includes */
#include "stm32f30x.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery.h"
#include <stdio.h>
#include "math.h"
#include "arm_math.h"

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

void GyroConfig(void);
void CompassConfig(void);
void GyroReadAngRate(volatile float* pfData);
void CompassReadMag(volatile float* pfData);
void CompassReadAcc(volatile float* pfData);

void CalibrateGyro(void);
void CalibrateAcc(void);
void CalibrateMag(void);
void InitializeStateEstimation(void);
char GetGyroCalibrated(void);
char GetAccCalibrated(void);
char GetMagCalibrated(void);
char GetStatesInitialized(void);

float GetRoll(void);
float GetPitch(void);
float GetYawRate(void);
float GetZVelocity(void);
float GetHeading(void);

void ReadSensors(void);
void UpdateStates(void);
void InitRollEstimator(void);
void UpdateRollEstimate(void);
void InitPitchEstimator(void);
void UpdatePitchEstimate(void);
void UpdateZVelocityEstimate(void);
void UpdateYawRateEstimate(void);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
