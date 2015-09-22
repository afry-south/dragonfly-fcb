/****************************************************************************
 * @file    state_estimation.c
 * @brief   Module implements the Kalman state estimation algorithm
 *****************************************************************************/

// TODO NOTE: Alot of code below is from the old project. It may be used as reference for further development.
// Eventually, it should be (re)moved and/or replaced or reimplemented.

/* Includes ------------------------------------------------------------------*/
#include "state_estimation.h"
#include "stm32f3xx.h"
#include "fcb_gyroscope.h"

/* Private variables ---------------------------------------------------------*/
StateVector_TypeDef States;

KalmanFilter_TypeDef RollEstimator;
KalmanFilter_TypeDef PitchEstimator;
KalmanFilter_TypeDef YawEstimator;

/* Private functions -----------------------------------------------*/
static void StateInit(KalmanFilter_TypeDef * Estimator);
static void StatePrediction(float* newRate, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias);
static void StateCorrection(float* newAngle, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias);

/* InitEstimator
 * @brief  Initializes the roll state Kalman estimator
 * @param  None
 * @retval None
 */
void InitStatesXYZ(void)
{
  StateInit(&RollEstimator);
  StateInit(&PitchEstimator);
  StateInit(&YawEstimator);
}

/* InitEstimator
 * @brief  Initializes the roll state Kalman estimator
 * @param  None
 * @retval None
 */
static void StateInit(KalmanFilter_TypeDef * Estimator)
{
  /* P matrix init is the Identity matrix*/
  Estimator->p11 = 1.0;
  Estimator->p12 = 0.0;
  Estimator->p21 = 0.0;
  Estimator->p22 = 1.0;

  /* q1 = sqrt(var(rate))*CONTROL_SAMPLE_PERIOD^2
   * q2 = sqrt(var(rateBias))
   * r1 = sqrt(var(angle)) */
  Estimator->q1 = Q1_CAL;
  Estimator->q2 = Q2_CAL;
  Estimator->r1 = R1_CAL;
}

/* PredictStatesXYZ
 * @brief  Updates the state estimates for X, Y, Z (roll, pitch, yaw)
 * @param  newRatesXYZ: Pointer to measured gyroscope rates, X,Y,Z
 * @retval None
 */
void PredictStatesXYZ(float newRatesXYZ[])
{
  StatePrediction( &newRatesXYZ[0], &RollEstimator, &(States.roll), &(States.rollRateBias));
  StatePrediction( &newRatesXYZ[1], &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
  StatePrediction( &newRatesXYZ[2], &YawEstimator, &(States.yaw), &(States.yawRateBias));
}

/* StatePrediction
 * @brief	Performs the prediction part of the Kalman filtering.
 * @param 	newRate: Pointer to measured gyroscope rate (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StatePrediction(float* newRate, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias)
{
	/* Prediction */
	/* Step 1: Calculate a priori state estimation*/

	/* WARNING: CONTROL_SAMPLE_PERIOD is set to 0 right now!! WARNING */
	*stateAngle += CONTROL_SAMPLE_PERIOD * (*newRate) - CONTROL_SAMPLE_PERIOD * (*stateRateBias);

	/* Step 2: Calculate a priori error covariance matrix P*/
	Estimator->p11 += (CONTROL_SAMPLE_PERIOD*Estimator->p22 - Estimator->p12 - Estimator->p21 + Estimator->q1*CONTROL_SAMPLE_PERIOD*CONTROL_SAMPLE_PERIOD)*CONTROL_SAMPLE_PERIOD;
	Estimator->p12 -= Estimator->p22 * CONTROL_SAMPLE_PERIOD;
	Estimator->p21 -= Estimator->p22 * CONTROL_SAMPLE_PERIOD;
	Estimator->p22 += Estimator->q2 * CONTROL_SAMPLE_PERIOD;
}

/* CorrectStatesXYZ
 * @brief  Updates the state estimates
 * @param  None
 * @retval None
 */
void CorrectStatesXYZ(float newAnglesXYZ[])
{
	StateCorrection( &newAnglesXYZ[0], &RollEstimator, &(States.roll), &(States.rollRateBias));
	StateCorrection( &newAnglesXYZ[1], &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
	StateCorrection( &newAnglesXYZ[2], &YawEstimator, &(States.yaw), &(States.yawRateBias));
}

/* StateCorrection
 * @brief	Performs the correction part of the Kalman filtering.
 * @param 	newAngle: Pointer to measured angle using accelerometer or magnetometer (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StateCorrection(float* newAngle, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias)
{
	/* Correction */
	/* Step3: Calculate y, difference between a-priori state and measurement z. */
	float y = *newAngle - *stateAngle;

	/* Step 4: Calculate innovation covariance matrix S*/
	float s = Estimator->p11 + Estimator->r1;

	/* Step 5: Calculate Kalman gain*/
	Estimator->k1 = Estimator->p11 /s;
	Estimator->k2 = Estimator->p21 /s;

	/* Step 6: Update a posteriori state estimation*/
	*stateAngle += Estimator->k1 * y;
	*stateRateBias += Estimator->k2 * y;

	/* Step 7: Update a posteriori error covariance matrix P*/
	float p11_tmp = Estimator->p11;
	float p12_tmp = Estimator->p12;
	Estimator->p11 -= Estimator->k1 * p11_tmp;
	Estimator->p12 -= Estimator->k1 * p12_tmp;
	Estimator->p21 -= Estimator->k1 * p11_tmp;
	Estimator->p22 -= Estimator->k1 * p12_tmp;
}

/* GetRoll
 * @brief  Gets the roll angle
 * @param  None
 * @retval Roll angle state
 */
float GetRollAngle(void)
{
  return States.roll;
}

/* GetPitch
 * @brief  Gets the pitch angle
 * @param  None
 * @retval Pitch angle state
 */
float GetPitchAngle(void)
{
  return States.pitch;
}

/* GetYaw
 * @brief  Gets the yaw angle
 * @param  None
 * @retval Yaw angle state
 */
float GetYawAngle(void)
{
  return States.yaw;
}


float GetHeading(void)
{
#ifdef TODO
  float fTiltedX = MagBuffer[0]*cosf(States.pitch) + MagBuffer[1]*sinf(States.roll)*sinf(States.pitch) + MagBuffer[2]*cosf(States.roll)*sinf(States.pitch);
  float fTiltedY = MagBuffer[1]*cosf(States.roll) - MagBuffer[2]*sinf(States.roll);

  float HeadingValue = atan2f(fTiltedY, fTiltedX) - COMPASS_DECLINATION;

  if (HeadingValue < 0)
    {
      HeadingValue = HeadingValue + 2.0*PI;
    }

  return HeadingValue;
#endif
  return 0.0;
}


/*****END OF FILE****/
