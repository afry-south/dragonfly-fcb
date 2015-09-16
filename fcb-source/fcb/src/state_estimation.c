/****************************************************************************
 * @file    state_estimation.c
 * @brief   Module implements the Kalman state estimation algorithm
 *****************************************************************************/

// TODO NOTE: Alot of code below is from the old project. It may be used as reference for further development.
// Eventually, it should be (re)moved and/or replaced or reimplemented.

/* Includes ------------------------------------------------------------------*/
#include "state_estimation.h"

#include "stm32f3xx.h"

/* Private variables ---------------------------------------------------------*/
float MagOffsets[3] = {0.0f};
uint16_t MagCalSample = 0;
float CalMagXMax = 1.0, CalMagXMin = -1.0, CalMagYMax = 1.0, CalMagYMin = -1.0, CalMagZMax = 1.0, CalMagZMin = -1.0;
float MagXNorm = 1.0, MagYNorm = 1.0, MagZNorm = 1.0;

StateVector_TypeDef States;

KalmanFilter_TypeDef RollEstimator;
KalmanFilter_TypeDef PitchEstimator;
KalmanFilter_TypeDef YawRateEstimator;
KalmanFilter_TypeDef ZVelocityEstimator;

/* Private functions -----------------------------------------------*/

/* InitRollEstimator
 * @brief  Initializes the roll state Kalman estimator
 * @param  None
 * @retval None
 */
void InitRollEstimator(void)
{
  RollEstimator.p11 = 1.0;
  RollEstimator.p12 = 0.0;
  RollEstimator.p21 = 0.0;
  RollEstimator.p22 = 1.0;

  RollEstimator.q1 = 0.5;
  RollEstimator.q2 = 0.05;
  RollEstimator.r1 = 1.5;
}

/* UpdateRoll
 * @brief  Updates the roll state estimate using Kalman filtering
 * @param  None
 * @retval None
 */
void UpdateRollEstimate(void)
{
#ifdef TODO
  /* Get roll rate from sensors: RollRate = p + (q*sinRoll+r*cosRoll)*tanPitch */
  float rollRateGyro = GyroBuffer[0] + (GyroBuffer[1]*sinf(States.roll) + GyroBuffer[1]*cosf(States.roll))*tanf(States.pitch);

  /* Prediction */
  States.roll = States.roll + CONTROL_SAMPLE_PERTIOD*rollRateGyro - CONTROL_SAMPLE_PERTIOD*States.rollRateBias;

  RollEstimator.p11 = RollEstimator.p11-RollEstimator.p21*CONTROL_SAMPLE_PERTIOD - CONTROL_SAMPLE_PERTIOD*(RollEstimator.p12-RollEstimator.p22*CONTROL_SAMPLE_PERTIOD) + RollEstimator.q1;
  RollEstimator.p12 = RollEstimator.p12 - RollEstimator.p22*CONTROL_SAMPLE_PERTIOD;
  RollEstimator.p21 = RollEstimator.p21 - RollEstimator.p22*CONTROL_SAMPLE_PERTIOD;
  RollEstimator.p22 = RollEstimator.p22 + RollEstimator.q2;

  /* Correction */
  RollEstimator.k1 = RollEstimator.p11/(RollEstimator.p11+RollEstimator.r1);
  RollEstimator.k2 = RollEstimator.p21/(RollEstimator.p11+RollEstimator.r1);

  States.roll = States.roll + RollEstimator.k1*(AccAttitudeBuffer[0]-States.roll);
  States.rollRateBias = States.rollRateBias + RollEstimator.k2*(AccAttitudeBuffer[0]-States.roll);

  RollEstimator.p11 = RollEstimator.p11*(1-RollEstimator.k1);
  RollEstimator.p12 = RollEstimator.p12*(1-RollEstimator.k1);
  RollEstimator.p21 = RollEstimator.p21 - RollEstimator.k2*RollEstimator.p11;
  RollEstimator.p22 = RollEstimator.p22 - RollEstimator.k2*RollEstimator.p12;
#endif
}

/* InitPitchEstimator
 * @brief  Initializes the pitch state Kalman estimator
 * @param  None
 * @retval None
 */
void InitPitchEstimator(void)
{
  PitchEstimator.p11 = 1.0;
  PitchEstimator.p12 = 0.0;
  PitchEstimator.p21 = 0.0;
  PitchEstimator.p22 = 1.0;

  PitchEstimator.q1 = 0.5;
  PitchEstimator.q2 = 0.05;
  PitchEstimator.r1 = 1.5;
}

/* UpdatePitch
 * @brief  Updates the pitch state estimate using Kalman filtering
 * @param  None
 * @retval None
 */
void UpdatePitchEstimate(void)
{
#ifdef TODO
  /* Get pitch rate from sensors: PitchRate = q*cosRoll - r*sinRoll */
  float pitchRateGyro = GyroBuffer[1]*cosf(States.roll) - GyroBuffer[1]*sinf(States.roll);

  /* Prediction */
  States.pitch = States.pitch + CONTROL_SAMPLE_PERIOD*pitchRateGyro - CONTROL_SAMPLE_PERIOD*States.pitchRateBias;

  PitchEstimator.p11 = PitchEstimator.p11-PitchEstimator.p21*CONTROL_SAMPLE_PERIOD - CONTROL_SAMPLE_PERIOD*(PitchEstimator.p12-PitchEstimator.p22*CONTROL_SAMPLE_PERIOD) + PitchEstimator.q1;
  PitchEstimator.p12 = PitchEstimator.p12 - PitchEstimator.p22*CONTROL_SAMPLE_PERIOD;
  PitchEstimator.p21 = PitchEstimator.p21 - PitchEstimator.p22*CONTROL_SAMPLE_PERIOD;
  PitchEstimator.p22 = PitchEstimator.p22 + PitchEstimator.q2;

  /* Correction */
  PitchEstimator.k1 = PitchEstimator.p11/(PitchEstimator.p11+PitchEstimator.r1);
  PitchEstimator.k2 = PitchEstimator.p21/(PitchEstimator.p11+PitchEstimator.r1);

  States.pitch = States.pitch + PitchEstimator.k1*(AccAttitudeBuffer[1]-States.pitch);
  States.pitchRateBias = States.pitchRateBias + PitchEstimator.k2*(AccAttitudeBuffer[1]-States.pitch);

  PitchEstimator.p11 = PitchEstimator.p11*(1-PitchEstimator.k1);
  PitchEstimator.p12 = PitchEstimator.p12*(1-PitchEstimator.k1);
  PitchEstimator.p21 = PitchEstimator.p21 - PitchEstimator.k2*RollEstimator.p11;
  PitchEstimator.p22 = PitchEstimator.p22 - PitchEstimator.k2*PitchEstimator.p12;
#endif
}

/* GetRoll
 * @brief  Gets the roll angle
 * @param  None
 * @retval Roll angle state
 */
float GetRoll(void)
{
  return States.roll;
}

/* GetPitch
 * @brief  Gets the pitch angle
 * @param  None
 * @retval Pitch angle state
 */
float GetPitch(void)
{
  return States.pitch;
}

/* GetYawRate
 * @brief  Gets the yaw angular rate
 * @param  None
 * @retval Yaw angular rate state
 */

float GetYawRate(void)
{
  return States.yawRate;
}

/* UpdateZVelocity
 * @brief  Updates the Z velocity state
 * @param  None
 * @retval None
 */
void UpdateZVelocityEstimate(void)
{
#ifdef TODO
  float ZAcc = (-AccBuffer[0]*sinf(States.pitch)+AccBuffer[1]*sinf(States.roll)*cosf(States.pitch)+AccBuffer[2]*cosf(States.roll)*cosf(States.pitch))*G_ACC - G_ACC;
  States.ZVelocity += ZAcc*CONTROL_SAMPLE_PERTIOD;
#endif
}

/* GetZVelocity
 * @brief  Gets the vertical z velocity
 * @param  None
 * @retval Z velocity state
 */
float GetZVelocity(void)
{
  return States.ZVelocity;
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

/* UpdateStates
 * @brief  Updates the state estimates
 * @param  None
 * @retval None
 */
void UpdateStates(void)
{
  UpdateRollEstimate();
  UpdatePitchEstimate();
  UpdateYawRateEstimate();
  UpdateZVelocityEstimate();
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
