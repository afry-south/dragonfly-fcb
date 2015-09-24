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
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fcb_error.h"

/* Private define ------------------------------------------------------------*/
#define STATE_PRINT_SAMPLING_TASK_PRIO			1
#define STATE_PRINT_MINIMUM_SAMPLING_TIME		2	// Motor control updated every 2.5 ms
#define STATE_PRINT_MAX_STRING_SIZE				128

/* Private variables ---------------------------------------------------------*/
StateVector_TypeDef States;
KalmanFilter_TypeDef RollEstimator;
KalmanFilter_TypeDef PitchEstimator;
KalmanFilter_TypeDef YawEstimator;

/* Task handle for printing of sensor values task */
static volatile uint16_t statePrintSampleTime;
static volatile uint16_t statePrintSampleDuration;
xTaskHandle StatePrintSamplingTaskHandle = NULL;
static SerializationType_TypeDef statePrintSerializationType;

/* Private function prototypes -----------------------------------------------*/
static void StateInit(KalmanFilter_TypeDef * Estimator);
static void StatePrediction(float* newRate, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias);
static void StateCorrection(float* newAngle, KalmanFilter_TypeDef * Estimator, float* stateAngle, float* stateRateBias);
static void StatePrintSamplingTask(void const *argument);
static void PrintStateValues(const SerializationType_TypeDef serializationType);


/* Exported functions --------------------------------------------------------*/

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

  States.roll = 0;
  States.rollRateBias = 0;
  States.pitch = 0;
  States.pitchRateBias = 0;
  States.yaw = 0;
  States.yawRateBias = 0;

}

/* PredictStatesXYZ
 * @brief  Updates the state estimates for X, Y, Z (roll, pitch, yaw)
 * @param  newRatesXYZ: Pointer to measured gyroscope rates, X,Y,Z
 * @retval None
 */
void PredictStatesXYZ(float32_t newRatesXYZ[])
{
  StatePrediction( &newRatesXYZ[0], &RollEstimator, &(States.roll), &(States.rollRateBias));
  StatePrediction( &newRatesXYZ[1], &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
  StatePrediction( &newRatesXYZ[2], &YawEstimator, &(States.yaw), &(States.yawRateBias));
}

/* CorrectStatesXYZ
 * @brief  Updates the state estimates
 * @param  None
 * @retval None
 */
void CorrectStatesXYZ(float32_t newAnglesXYZ[])
{
	StateCorrection( &newAnglesXYZ[0], &RollEstimator, &(States.roll), &(States.rollRateBias));
	StateCorrection( &newAnglesXYZ[1], &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
	StateCorrection( &newAnglesXYZ[2], &YawEstimator, &(States.yaw), &(States.yawRateBias));
}

/* GetRoll
 * @brief  Gets the roll angle
 * @param  None
 * @retval Roll angle state
 */
float32_t GetRollAngle(void)
{
  return States.roll;
}

/* GetPitch
 * @brief  Gets the pitch angle
 * @param  None
 * @retval Pitch angle state
 */
float32_t GetPitchAngle(void)
{
  return States.pitch;
}

/* GetYaw
 * @brief  Gets the yaw angle
 * @param  None
 * @retval Yaw angle state
 */
float32_t GetYawAngle(void)
{
  return States.yaw;
}

/*
 * @brief  Creates a task to print states over USB.
 * @param  sampleTime : Sets how often a sample should be printed.
 * @param  sampleDuration : Sets for how long sampling should be performed.
 * @retval MOTORCTRL_OK if thread started, else MOTORCTRL_ERROR.
 */
StateErrorStatus StartStateSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration) {
	if(sampleTime < STATE_PRINT_MINIMUM_SAMPLING_TIME)
		statePrintSampleTime = STATE_PRINT_MINIMUM_SAMPLING_TIME;
	else
		statePrintSampleTime = sampleTime;

	statePrintSampleDuration = sampleDuration;

	/* State value print sampling handler thread creation
	 * Task function pointer: StatePrintSamplingTask
	 * Task name: STATE_PRINT_SAMPL
	 * Stack depth: configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: STATE_PRINT_SAMPLING_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: StatePrintSamplingTaskHandle
	 **/
	if (pdPASS != xTaskCreate((pdTASK_CODE )StatePrintSamplingTask, (signed portCHAR*)"STATE_PRINT_SAMPL",
			configMINIMAL_STACK_SIZE, NULL, STATE_PRINT_SAMPLING_TASK_PRIO, &StatePrintSamplingTaskHandle)) {
		ErrorHandler();
		return STATE_ERROR;
	}


	return STATE_OK;

}

/*
 * @brief  Stops motor control print sampling by deleting the task.
 * @param  None.
 * @retval MOTORCTRL_OK if task deleted, MOTORCTRL_ERROR if not.
 */
StateErrorStatus StopStateSamplingTask(void) {
	if(StatePrintSamplingTaskHandle != NULL) {
		vTaskDelete(StatePrintSamplingTaskHandle);
		StatePrintSamplingTaskHandle = NULL;
		return STATE_OK;
	}
	return STATE_ERROR;
}

/*
 * @brief  Sets the serialization type of printed motor signal values
 * @param  serializationType : Data serialization type enum
 * @retval None.
 */
void SetStateSamplingSerialization(const SerializationType_TypeDef serializationType) {
	statePrintSerializationType = serializationType;
}

/* Private functions ---------------------------------------------------------*/

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



/* StatePrediction
 * @brief	Performs the prediction part of the Kalman filtering.
 * @param 	newRate: Pointer to measured gyroscope rate (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StatePrediction(float32_t* newRate, KalmanFilter_TypeDef * Estimator, float32_t* stateAngle, float32_t* stateRateBias)
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



/* StateCorrection
 * @brief	Performs the correction part of the Kalman filtering.
 * @param 	newAngle: Pointer to measured angle using accelerometer or magnetometer (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StateCorrection(float32_t* newAngle, KalmanFilter_TypeDef * Estimator, float32_t* stateAngle, float32_t* stateRateBias)
{
	/* Correction */
	/* Step3: Calculate y, difference between a-priori state and measurement z. */
	float32_t y = *newAngle - *stateAngle;

	/* Step 4: Calculate innovation covariance matrix S*/
	float32_t s = Estimator->p11 + Estimator->r1;

	/* Step 5: Calculate Kalman gain*/
	Estimator->k1 = Estimator->p11 /s;
	Estimator->k2 = Estimator->p21 /s;

	/* Step 6: Update a posteriori state estimation*/
	*stateAngle += Estimator->k1 * y;
	*stateRateBias += Estimator->k2 * y;

	/* Step 7: Update a posteriori error covariance matrix P*/
	float32_t p11_tmp = Estimator->p11;
	float32_t p12_tmp = Estimator->p12;
	Estimator->p11 -= Estimator->k1 * p11_tmp;
	Estimator->p12 -= Estimator->k1 * p12_tmp;
	Estimator->p21 -= Estimator->k1 * p11_tmp;
	Estimator->p22 -= Estimator->k1 * p12_tmp;
}


/**
 * @brief  Task code handles state (angle, rate, ratebias) print sampling
 * @param  argument : Unused parameter
 * @retval None
 */
static void StatePrintSamplingTask(void const *argument) {
	(void) argument;

	portTickType xLastWakeTime;
	portTickType xSampleStartTime;

	/* Initialize the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();
	xSampleStartTime = xLastWakeTime;

	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, statePrintSampleTime);

		PrintStateValues(statePrintSerializationType);

		/* If sampling duration exceeded, delete task to stop sampling */
		if (xTaskGetTickCount() >= xSampleStartTime + statePrintSampleDuration * configTICK_RATE_HZ)
			StopStateSamplingTask();
	}
}

static void PrintStateValues(const SerializationType_TypeDef serializationType) {
	static char stateString[STATE_PRINT_MAX_STRING_SIZE];

	if(serializationType == NO_SERIALIZATION) {
		snprintf((char*) stateString, STATE_PRINT_MAX_STRING_SIZE,
				"States (float32_t):\nrollAngle: %f\nrollRateBias: %f\npitchAngle: %f\npitchRateBias: %f\nyawAngle: %f\nyawRateBias: %f\n\r\n",
				States.roll, States.rollRateBias, States.pitch, States.pitchRateBias, States.yaw, States.yawRateBias);

		USBComSendString(stateString);
	}
	else if(serializationType == PROTOBUFFER_SERIALIZATION) {
		//TODO
	}
}



/*****END OF FILE****/
