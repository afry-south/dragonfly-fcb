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
#include "dragonfly_fcb.pb.h"
#include "usb_com_cli.h"
#include "pb_encode.h"
#include "math.h"
#include "rotation_transformation.h"
#include "fcb_accelerometer_magnetometer.h"

/* Private define ------------------------------------------------------------*/
#define STATE_PRINT_SAMPLING_TASK_PRIO			1
#define STATE_PRINT_MINIMUM_SAMPLING_TIME		20	// updated every 2.5 ms
#define STATE_PRINT_MAX_STRING_SIZE				256
#define RAD_TO_DEG								180/PI

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
static void StatePrediction(const float32_t sensorRate, KalmanFilter_TypeDef* Estimator, float32_t* stateAngle, float32_t* stateRateBias);
static void StateCorrection(const float32_t sensorAngle, KalmanFilter_TypeDef* Estimator, float32_t* stateAngle, float32_t* stateRateBias);
static void StatePrintSamplingTask(void const *argument);

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

  States.roll = 0.0;
  States.rollRateBias = 0.0;
  States.pitch = 0.0;
  States.pitchRateBias = 0.0;
  States.yaw = 0.0;  // TODO ISSUE119 - our state for yaw is turn rate, not heading? No, we are estimating the yaw angle.
  States.yawRateBias = 0.0;
}

/* PredictStatesXYZ
 * @brief  Updates the state estimates for X, Y, Z (roll, pitch, yaw)
 * @param  newRatesXYZ: Pointer to measured gyroscope rates, X,Y,Z
 * @retval None
 */
void PredictStatesXYZ(const float32_t sensorRateRoll, const float32_t sensorRatePitch, const float32_t sensorRateYaw)
{
	StatePrediction(sensorRateRoll, &RollEstimator, &(States.roll), &(States.rollRateBias));
	StatePrediction(sensorRatePitch, &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
	StatePrediction(sensorRateYaw, &YawEstimator, &(States.yaw), &(States.yawRateBias));
}

/* CorrectStatesXYZ
 * @brief  Updates the state estimates
 * @param  None
 * @retval None
 */
void CorrectStatesXYZ(const float32_t sensorAngleRoll, const float32_t sensorAnglePitch, const float32_t sensorAngleYaw)
{
	StateCorrection(sensorAngleRoll, &RollEstimator, &(States.roll), &(States.rollRateBias));
	StateCorrection(sensorAnglePitch, &PitchEstimator, &(States.pitch), &(States.pitchRateBias));
	StateCorrection(sensorAngleYaw, &YawEstimator, &(States.yaw), &(States.yawRateBias));
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

	// TODO do not start a new task if one is already running, just update sampleTime/sampleDuration

	if(sampleTime < STATE_PRINT_MINIMUM_SAMPLING_TIME) {
		statePrintSampleTime = STATE_PRINT_MINIMUM_SAMPLING_TIME;
	}
	else {
		statePrintSampleTime = sampleTime;
	}

	statePrintSampleDuration = sampleDuration;

	/* State value print sampling handler thread creation
	 * Task function pointer: StatePrintSamplingTask
	 * Task name: STATE_PRINT_SAMPL
	 * Stack depth: 2*configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: STATE_PRINT_SAMPLING_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: StatePrintSamplingTaskHandle
	 **/
	if (pdPASS != xTaskCreate((pdTASK_CODE )StatePrintSamplingTask, (signed portCHAR*)"STATE_PRINT_SAMPL",
			3*configMINIMAL_STACK_SIZE, NULL, STATE_PRINT_SAMPLING_TASK_PRIO, &StatePrintSamplingTaskHandle)) {
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
void SetStatePrintSamplingSerialization(const SerializationType_TypeDef serializationType) {
	statePrintSerializationType = serializationType;
}

/*
 * @brief	Converts radians to degrees
 * @param	radian: The value in radians that should be converted.
 * @return	degree: The converted value in degrees.
 */
float32_t RadianToDegree(float32_t radian) {
	float32_t degree = radian*RAD_TO_DEG;

	while(degree>180) {
		degree -= 360;
	}
	while(degree<-180) {
		degree += 360;
	}

	return degree;
}

/*
 * @brief Prints the state values
 * @param serializationType: Data serialization type enum
 * @retval None
 */
void PrintStateValues(const SerializationType_TypeDef serializationType) {
	static char stateString[STATE_PRINT_MAX_STRING_SIZE];
	int usedLen = 0;

	if ((serializationType == NO_SERIALIZATION) || (serializationType == CALIBRATION_SERIALIZATION)) {
		float32_t sensorAttitude[3], accValues[3], magValues[3];

		// TODO Delete sensor attitude printouts
		/* Get magnetometer values */
		GetMagVector(&magValues[0], &magValues[1], &magValues[2]);

		/* Get accelerometer values */
		GetAcceleration(&accValues[0], &accValues[1], &accValues[2]);

		/* Calculate roll, pitch, yaw based on accelerometer and magnetometer values */
		GetAttitudeFromAccelerometer(sensorAttitude, accValues);
		sensorAttitude[2] = GetMagYawAngle(magValues, sensorAttitude[0], sensorAttitude[1]);

		if (serializationType == NO_SERIALIZATION) {
		  snprintf((char*) stateString, STATE_PRINT_MAX_STRING_SIZE,
		      "States:\nrollAngle: %1.3f deg\npitchAngle: %1.3f deg\nyawAngle: %1.4f deg\nrollRateBias: %1.3f\npitchRateBias: %1.3f\nyawRateBias: %1.3f\naccRoll:%1.3f, accPitch:%1.3f, magYaw:%1.3f\n\r\n",
		      RadianToDegree(States.roll), RadianToDegree(States.pitch), RadianToDegree(States.yaw),
		      States.rollRateBias, States.pitchRateBias, States.yawRateBias,
		      sensorAttitude[0], sensorAttitude[1], sensorAttitude[2]);
		} else /* CALIBRATION_SERIALIZATION */ {
		  usedLen = snprintf((char*) stateString, STATE_PRINT_MAX_STRING_SIZE,
		      "States:\nAngle-RPY [deg]: %1.3f, %1.3f, %1.4f\n Bias-RPY: %1.3f, %1.3f, %1.3f\nAcc-RPY: %f, %f, %f\n\r\n",
		      RadianToDegree(States.roll), RadianToDegree(States.pitch), RadianToDegree(States.yaw),
		      States.rollRateBias, States.pitchRateBias, States.yawRateBias,
		      sensorAttitude[0], sensorAttitude[1], sensorAttitude[2]);

		  if (usedLen < STATE_PRINT_MAX_STRING_SIZE) {
		    usedLen = snprintf((char*) stateString+usedLen, STATE_PRINT_MAX_STRING_SIZE - usedLen,
		        "KF: P11-RPY: %e, %e, %e\n"
		        "KF: K1-RPY: %e, %e, %e\n\r\n",
		        RollEstimator.p11, PitchEstimator.p11, YawEstimator.p11, RollEstimator.k1, PitchEstimator.k1, YawEstimator.k1);
		  }
		}

		USBComSendString(stateString);
	}
	else if(serializationType == PROTOBUFFER_SERIALIZATION) {
		bool protoStatus;
		uint8_t serializedStateData[FlightStatesProto_size];
		FlightStatesProto stateValuesProto;
		uint32_t strLen;

		/* Add estimated attitude states to protobuffer type struct members */
		stateValuesProto.has_rollAngle = true;
		stateValuesProto.rollAngle = States.roll;
		stateValuesProto.has_pitchAngle = true;
		stateValuesProto.pitchAngle = States.pitch;
		stateValuesProto.has_yawAngle = true;
		stateValuesProto.yawAngle = States.yaw;

		// TODO add attitude rates when available
		stateValuesProto.has_rollRate = false;
		stateValuesProto.rollRate = 0.0;
		stateValuesProto.has_pitchRate = false;
		stateValuesProto.pitchRate = 0.0;
		stateValuesProto.has_yawRate = false;
		stateValuesProto.yawRate = 0.0;

		// TODO add position estimates when available
		stateValuesProto.has_posX = false;
		stateValuesProto.posX = 0.0;
		stateValuesProto.has_posY = false;
		stateValuesProto.posY = 0.0;
		stateValuesProto.has_posZ = false;
		stateValuesProto.posZ = 0.0;

		// TODO add velocity estimates when available
		stateValuesProto.has_velX = false;
		stateValuesProto.velX = 0.0;
		stateValuesProto.has_velY = false;
		stateValuesProto.velY = 0.0;
		stateValuesProto.has_velZ = false;
		stateValuesProto.velZ = 0.0;

		/* Create a stream that will write to our buffer and encode the data with protocol buffer */
		pb_ostream_t protoStream = pb_ostream_from_buffer(serializedStateData, FlightStatesProto_size);
		protoStatus = pb_encode(&protoStream, FlightStatesProto_fields, &stateValuesProto);

		/* Insert header to the sample string, then copy the data after that */
		snprintf(stateString, STATE_PRINT_MAX_STRING_SIZE, "%c %c ", FLIGHT_STATE_MSG_ENUM, protoStream.bytes_written);
		strLen = strlen(stateString);
		if(strLen + protoStream.bytes_written + strlen("\r\n") < STATE_PRINT_MAX_STRING_SIZE) {
			memcpy(&stateString[strLen], serializedStateData, protoStream.bytes_written);
			memcpy(&stateString[strLen+protoStream.bytes_written], "\r\n", strlen("\r\n"));
		}

		if(protoStatus)
			USBComSendData((uint8_t*)stateString, strLen+protoStream.bytes_written+strlen("\r\n"));
		else
			ErrorHandler();
	}
}

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Initializes the roll state Kalman estimator
 * @param  None
 * @retval None
 */
static void StateInit(KalmanFilter_TypeDef * Estimator)
{
  /* P matrix init is the Identity matrix*/
  Estimator->p11 = 0.1;
  Estimator->p12 = 0.0;
  Estimator->p21 = 0.0;
  Estimator->p22 = 0.1;

  /* q1 = sqrt(var(rate))*STATE_ESTIMATION_SAMPLE_PERIOD^2
   * q2 = sqrt(var(rateBias))
   * r1 = sqrt(var(angle)) */
  Estimator->q1 = GYRO_AXIS_VARIANCE_ROUGH * STATE_ESTIMATION_SAMPLE_PERIOD * STATE_ESTIMATION_SAMPLE_PERIOD; /* Q1_CAL */
  Estimator->q2 = Q2_CAL;
  Estimator->r1 = R1_CAL; /* TODO_ISSUE119 - accelerometer based angle variance */
}

/*
 * @brief	Performs the prediction part of the Kalman filtering.
 * @param 	newRate: Pointer to measured gyroscope rate (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StatePrediction(const float32_t sensorRate, KalmanFilter_TypeDef* Estimator, float32_t* stateAngle, float32_t* stateRateBias)
{
	/* Prediction */
	/* Step 1: Calculate a priori state estimation*/
	*stateAngle += STATE_ESTIMATION_SAMPLE_PERIOD * sensorRate - STATE_ESTIMATION_SAMPLE_PERIOD * (*stateRateBias);

	/* Step 2: Calculate a priori error covariance matrix P*/
	Estimator->p11 += (STATE_ESTIMATION_SAMPLE_PERIOD*Estimator->p22 - Estimator->p12 - Estimator->p21 + Estimator->q1)*STATE_ESTIMATION_SAMPLE_PERIOD;
	Estimator->p12 -= Estimator->p22 * STATE_ESTIMATION_SAMPLE_PERIOD;
	Estimator->p21 -= Estimator->p22 * STATE_ESTIMATION_SAMPLE_PERIOD;
	Estimator->p22 += Estimator->q2 * STATE_ESTIMATION_SAMPLE_PERIOD;
}

/*
 * @brief	Performs the correction part of the Kalman filtering.
 * @param 	newAngle: Pointer to measured angle using accelerometer or magnetometer (roll, pitch or yaw)
 * @param 	Estimator: Pointer to KalmanFilter_TypeDef struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVector_TypeDef (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVector_TypeDef (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void StateCorrection(const float32_t sensorAngle, KalmanFilter_TypeDef* Estimator, float32_t* stateAngle, float32_t* stateRateBias)
{
	/* Correction */
	/* Step3: Calculate y, difference between a-priori state and measurement z */
	float32_t y = sensorAngle - *stateAngle;

	/* Step 4: Calculate innovation covariance matrix S*/
	float32_t s = Estimator->p11 + Estimator->r1;

	/* Step 5: Calculate Kalman gain*/
	Estimator->k1 = Estimator->p11 /s;
	Estimator->k2 = Estimator->p21 /s;

	/* Step 6: Update a posteriori state estimation*/
	*stateAngle += Estimator->k1 * y;
	(void*) stateRateBias; // To avoid warnings

	/* Step 7: Update a posteriori error covariance matrix P */
	float32_t p11_tmp = Estimator->p11;
	float32_t p12_tmp = Estimator->p12;
	Estimator->p11 = p11_tmp - Estimator->k1*p11_tmp;
	Estimator->p12 = p12_tmp - Estimator->k1*p12_tmp;
	Estimator->p21 = -Estimator->k2*p11_tmp;
	Estimator->p22 = -Estimator->k2*p12_tmp;
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

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
