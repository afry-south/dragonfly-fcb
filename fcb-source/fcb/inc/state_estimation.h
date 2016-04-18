/****************************************************************************
* @file    state_estimation.h
* @brief   Header file for state_estimation.c
*****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"
#include "arm_math.h"

#include "communication.h"
#include "flight_control.h"
#include "fcb_retval.h"
#include "common.h"

/* Exported types ------------------------------------------------------------*/

/**
 * This type is to make indexing into arrays of Roll-Pitch-Yaw values have
 * fewer magic numbers.
 *
 * This index is interchangeable with the X, Y, Z axes based index but
 * we use different indices depending on application context.
 */
typedef enum FcbRPYIndex {
  ROLL_IDX = 0,
  PITCH_IDX = 1,
  YAW_IDX = 2,
  AXES_NPR
} FcbRPYIndexType;

typedef struct KalmanFilter
{
    /* process noise covariance matrix components */
	float32_t q1;   /* multiply with deltaT^2 to get q1 */
	float32_t q2;
	float32_t q3;
	float32_t r1;	// Measurement noise covariance matrix component
	float32_t r2;   // Measurement noise covariance matrix component
	float32_t p11;	// Error covariance matrix component
	float32_t p12;  // Error covariance matrix component
	float32_t p13;  // Error covariance matrix component
	float32_t p21;  // Error covariance matrix component
	float32_t p22;  // Error covariance matrix component
	float32_t p23;  // Error covariance matrix component
	float32_t p31;  // Error covariance matrix component
	float32_t p32;  // Error covariance matrix component
	float32_t p33;  // Error covariance matrix component
	float32_t k11;	// Kalman gain
	float32_t k12;  // Kalman gain
	float32_t k21;  // Kalman gain
	float32_t k22;  // Kalman gain
	float32_t k31;  // Kalman gain
	float32_t k32;   // note: k<x> these don't have to be part of the struct as they
	                // need not be carried over from one iteration to the next
	                // but it's useful for displaying variables.
	float32_t h;    // Sample time [s]
} KalmanFilterType;
// TODO use vectors/matrices for Q, R, P, K

/**
 * This is used for roll, pitch & yaw attitude.
 */
typedef struct AttitudeStateVector
{
  float32_t volatile angle; /* for yaw, aka "heading" */
  float32_t angleRate; /* not used */
  float32_t angleRateBias;
  float32_t angleRateUnbiased; // Not used in Kalman filter derivation, but should be fed in to control
} AttitudeStateVectorType;

/* Exported constants --------------------------------------------------------*/
#define STATE_ESTIMATION_UPDATE_TIM                     TIM7
#define STATE_ESTIMATION_UPDATE_TIM_CLK_ENABLE()        __TIM7_CLK_ENABLE()
#define STATE_ESTIMATION_UPDATE_TIM_CLK_DISABLE()       __TIM7_CLK_DISABLE()
#define STATE_ESTIMATION_UPDATE_TIM_IRQn                TIM7_IRQn
#define STATE_ESTIMATION_UPDATE_TIM_IRQHandler          TIM7_IRQHandler
#define STATE_ESTIMATION_UPDATE_TIM_IRQ_PREEMPT_PRIO    configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY // Set to this since used w/ RTOS
#define STATE_ESTIMATION_UPDATE_TIM_IRQ_SUB_PRIO        0
#define STATE_ESTIMATION_TIME_UPDATE_PERIOD             (60000-1)
#define STATE_ESTIMATION_TIME_UPDATE_PRESCALER          12

// TODO we need separate values for roll pitch and yaw as well as separate init values of P matrix
#define	STATE_ESTIMATION_SAMPLE_PERIOD	(float32_t) 	FLIGHT_CONTROL_TASK_PERIOD / 1000.0
#define Q1_CAL (float32_t)								0.05
#define	Q2_CAL (float32_t)								0.5 //0.005
#define Q3_CAL (float32_t)                              0.0005
#define	R1_CAL (float32_t)						   		0.000185 /* 480 measured from USB console and
                                                          * calculated with SensorVariance.sce
                                                          */
#define R2_CAL                                          0.005

typedef enum {
    STATE_EST_ERROR = 0, STATE_EST_OK = !STATE_EST_ERROR
} StateEstimationStatus;

/* Exported variables --------------------------------------------------------*/
TIM_HandleTypeDef StateEstimationTimHandle;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
float32_t GetRollAngle(void);
float32_t GetPitchAngle(void);
float32_t GetYawAngle(void);
float32_t GetRollRate(void);
float32_t GetPitchRate(void);
float32_t GetYawRate(void);

void InitStatesXYZ(void);
StateEstimationStatus InitStateEstimationTimeEvent(void);
void UpdatePredictionState(void);
void UpdateCorrectionState(FcbSensorIndexType sensorType, float32_t deltaT, float32_t const * pXYZ);

FcbRetValType StartStateSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration);
FcbRetValType StopStateSamplingTask(void);
void SetStatePrintSamplingSerialization(const SerializationType serializationType);
void PrintStateValues(const SerializationType serializationType);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
