/****************************************************************************
* @file    state_estimation.h
* @brief   Header file for state_estimation.c
*****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "usbd_cdc_if.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float32_t q1;	// Process noise covariance matrix components
	float32_t q2;
	float32_t r1;	// Measurement noise covariance matrix component
	float32_t p11;	// Error covariance matrix components
	float32_t p12;
	float32_t p21;
	float32_t p22;
	float32_t k1;	// Kalman gains
	float32_t k2;
}KalmanFilter_TypeDef;

typedef struct
{
	float32_t roll;
	float32_t rollRateBias;
	float32_t pitch;
	float32_t pitchRateBias;
	float32_t yaw;
	float32_t yawRateBias;
}StateVector_TypeDef;

typedef enum {
	STATE_ERROR = 0, STATE_OK = !STATE_ERROR
} StateErrorStatus;


/* Exported constants --------------------------------------------------------*/
#define ACC_CALIBRATION_SAMPLES		(int)		100
#define MAG_CALIBRATION_SAMPLES    	(int)		2000
#define INIT_SAMPLES			(int)		1000
#define	CONTROL_SAMPLE_PERIOD (float32_t)		0
#define Q1_CAL (float32_t)						0.5
#define	Q2_CAL (float32_t)						0.05
#define	R1_CAL (float32_t)						1.5

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
float32_t GetRollAngle(void);
float32_t GetPitchAngle(void);
float32_t GetYawAngle(void);
float32_t GetHeading(void);

void InitStatesXYZ(void);
void PredictStatesXYZ(const float32_t sensorRateRoll, const float32_t sensorRatePitch, const float32_t sensorRateYaw);
void CorrectStatesXYZ(const float32_t sensorAngleRoll, const float32_t sensorAnglePitch, const float32_t sensorAngleYaw);
StateErrorStatus StartStateSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
StateErrorStatus StopStateSamplingTask(void);
void SetStateSamplingSerialization(const SerializationType_TypeDef serializationType);
void PrintStateValues(const SerializationType_TypeDef serializationType);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
