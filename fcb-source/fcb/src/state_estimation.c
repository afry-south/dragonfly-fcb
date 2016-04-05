/****************************************************************************
 * @file    state_estimation.c
 * @brief   Module implements the Kalman state estimation algorithm
 *****************************************************************************/

// TODO NOTE: Alot of code below is from the old project. It may be used as reference for further development.
// Eventually, it should be (re)moved and/or replaced or reimplemented.

/* Includes ------------------------------------------------------------------*/
#include "state_estimation.h"

#include "stm32f3xx.h"

#include "string.h"
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "dragonfly_fcb.pb.h"
#include "pb_encode.h"

#include "fcb_sensors.h"
#include "fcb_gyroscope.h"
#include "fcb_error.h"
#include "rotation_transformation.h"
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_retval.h"
#include "usbd_cdc_if.h"

/* Private define ------------------------------------------------------------*/
enum { VAR_SAMPLE_MAX = 100 }; /* number of samples for variance - max 256 */
typedef struct FcbSensorVarianceCalc {
  float32_t samples[3][VAR_SAMPLE_MAX];
} FcbSensorVarianceCalcType;

static FcbSensorVarianceCalcType * pSampleData = NULL;

#define STATE_PRINT_SAMPLING_TASK_PRIO			1
#define STATE_PRINT_MINIMUM_SAMPLING_TIME		20	// updated every 2.5 ms
#define STATE_PRINT_MAX_STRING_SIZE				256


/* Private variables ---------------------------------------------------------*/
static AttitudeStateVectorType rollState = { 0, 0 , 0 };
static AttitudeStateVectorType pitchState = { 0, 0 , 0 };
static AttitudeStateVectorType yawState = { 0, 0 , 0 };

static KalmanFilterType rollEstimator;
static KalmanFilterType pitchEstimator;
static KalmanFilterType yawEstimator;

/* Task handle for printing of sensor values task */
static volatile uint16_t statePrintSampleTime;
static volatile uint16_t statePrintSampleDuration;
xTaskHandle StatePrintSamplingTaskHandle = NULL;
static SerializationType statePrintSerializationType;

static float32_t sensorAttitudeRPY[3] = { 0.0f, 0.0f, 0.0f };

/* Private function prototypes -----------------------------------------------*/
static void StateSensorsEventCallback(FcbSensorIndexType sensorType, float32_t deltaT, float32_t const * xyz); /* type FcbSensorCbk  */
static void StateInit(KalmanFilterType * pEstimator);
static uint8_t ProfileSensorMeasurements(FcbSensorIndexType sensorType, float32_t const * pXYZData);

static void PredictAttitudeState(float32_t const deltaT, const float32_t sensorRate, KalmanFilterType* pEstimator, AttitudeStateVectorType * pState);
static void CorrectAttitudeState(const float32_t sensorAngle, KalmanFilterType* pEstimator, AttitudeStateVectorType * pState);
static void CorrectAttitudeRateState(float32_t const deltaT, const float32_t sensorRate, KalmanFilterType* pEstimator, AttitudeStateVectorType * pState);
static void StatePrintSamplingTask(void const *argument);

// TODO Need mutex between sensors between each sensor as well?) and time update events...? Shared resources P-matrix
// TODO Need RTOS task with queue pending on each sensor and time-event and handling it accordingly
// TODO Also need mutex protecting states when written/read

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes the Kalman state estimator
 * @param  None
 * @retval None
 */
void InitStatesXYZ(void)
{
  StateInit(&rollEstimator);
  StateInit(&pitchEstimator);
  StateInit(&yawEstimator);

  rollState.angle = 0.0;
  rollState.angleRateBias = 0.0;
  pitchState.angle = 0.0;
  pitchState.angleRateBias = 0.0;

  yawState.angle = 0.0; /* should be initialised with current heading */
  yawState.angleRateBias = 0.0;

  FcbSensorRegisterClientCallback(StateSensorsEventCallback);
}

/*
 * @brief  Initializes state estimation time-event generation timer
 * @param  None
 * @retval None
 */
StateEstimationStatus InitStateEstimationTimeEvent(void)
{
    StateEstimationStatus status = STATE_EST_OK;

    /*##-1- Configure the TIM peripheral #######################################*/

    /* Set TIMx instance */
    StateEstimationTimHandle.Instance = STATE_ESTIMATION_UPDATE_TIM;

    /* Initialize TIM3 peripheral as follows:
         + Period = 10000 - 1
         + Prescaler = (SystemCoreClock/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
     */
    StateEstimationTimHandle.Init.Period = 60000 - 1;
    StateEstimationTimHandle.Init.Prescaler = 12;
    StateEstimationTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    StateEstimationTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&StateEstimationTimHandle) != HAL_OK)
    {
        /* Initialization Error */
        status = STATE_EST_ERROR;
        ErrorHandler();
    }

    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if(HAL_TIM_Base_Start_IT(&StateEstimationTimHandle) != HAL_OK)
    {
        /* Starting Error */
        status = STATE_EST_ERROR;
        ErrorHandler();
    }

    return status;
}

/*
 * @brief  State estimation time-update callback
 * @param  None
 * @retval None
 */
void StateEstimationTimeEventCallback(void)
{
    /* run prediction step */
//    float32_t const * pSensorAngleRate = pXYZ;
//    PredictAttitudeState(deltaT, pSensorAngleRate[ROLL_IDX], &rollEstimator, &rollState);
//    PredictAttitudeState(deltaT, pSensorAngleRate[PITCH_IDX], &pitchEstimator, &pitchState);
//    PredictAttitudeState(deltaT, pSensorAngleRate[YAW_IDX], &yawEstimator, &yawState);
}

/* GetRoll
 * @brief  Gets the roll angle
 * @param  None
 * @retval Roll angle state
 */
float32_t GetRollAngle(void)
{
  return rollState.angle;
}

/* GetPitch
 * @brief  Gets the pitch angle
 * @param  None
 * @retval Pitch angle state
 */
float32_t GetPitchAngle(void)
{
  return pitchState.angle;
}

/* GetYaw
 * @brief  Gets the yaw angle
 * @param  None
 * @retval Yaw angle state
 */
float32_t GetYawAngle(void)
{
  return yawState.angle;
}

/*
 * @brief  Creates a task to print states over USB.
 * @param  sampleTime : Sets how often a sample should be printed.
 * @param  sampleDuration : Sets for how long sampling should be performed.
 * @retval MOTORCTRL_OK if thread started, else MOTORCTRL_ERROR.
 */
FcbRetValType StartStateSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration) {

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
		return FCB_ERR;
	}
	return FCB_OK;
}

/*
 * @brief  Stops motor control print sampling by deleting the task.
 * @param  None.
 * @retval MOTORCTRL_OK if task deleted, MOTORCTRL_ERROR if not.
 */
FcbRetValType StopStateSamplingTask(void) {
	if(StatePrintSamplingTaskHandle != NULL) {
		vTaskDelete(StatePrintSamplingTaskHandle);
		StatePrintSamplingTaskHandle = NULL;
		return FCB_OK;
	}
	return FCB_ERR;
}

/*
 * @brief  Sets the serialization type of printed motor signal values
 * @param  serializationType : Data serialization type enum
 * @retval None.
 */
void SetStatePrintSamplingSerialization(const SerializationType serializationType) {
	statePrintSerializationType = serializationType;
}


/*
 * @brief Prints the state values
 * @param serializationType: Data serialization type enum
 * @retval None
 */
void PrintStateValues(const SerializationType serializationType) {
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
		      "States:\nrollAngle: %1.3f deg\npitchAngle: %1.3f deg\nyawAngle: %1.3f deg\nrollRateBias: %1.3f\npitchRateBias: %1.3f\nyawRateBias: %1.3f\naccRoll:%1.3f, accPitch:%1.3f, magYaw:%1.3f\n\r\n",
		      Radian2Degree(rollState.angle), Radian2Degree(pitchState.angle), Radian2Degree(yawState.angle),
		      rollState.angleRateBias, pitchState.angleRateBias, yawState.angleRateBias,
		      sensorAttitude[0], sensorAttitude[1], sensorAttitude[2]);
		} else /* CALIBRATION_SERIALIZATION */ {
		  usedLen = snprintf((char*) stateString, STATE_PRINT_MAX_STRING_SIZE,
		      "States:\nAngle-RPY[deg]: %1.3f, %1.3f, %1.3f\nBias-RPY: %1.3f, %1.3f, %1.3f\nAcc-RPY: %1.3f, %1.3f, %1.3f\n\r\n",
		      Radian2Degree(rollState.angle), Radian2Degree(pitchState.angle), Radian2Degree(yawState.angle),
		      rollState.angleRateBias, pitchState.angleRateBias, yawState.angleRateBias,
		      sensorAttitude[0], sensorAttitude[1], sensorAttitude[2]);

		  if (usedLen < STATE_PRINT_MAX_STRING_SIZE) {
		    usedLen = snprintf((char*) stateString+usedLen, STATE_PRINT_MAX_STRING_SIZE - usedLen,
		        "KF: P11-RPY: %e, %e, %e\n"
		        "KF: K1-RPY: %e, %e, %e\n\r\n",
		        rollEstimator.p11, pitchEstimator.p11, yawEstimator.p11, rollEstimator.k11, pitchEstimator.k11, yawEstimator.k11);
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
		stateValuesProto.rollAngle = rollState.angle;
		stateValuesProto.has_pitchAngle = true;
		stateValuesProto.pitchAngle = pitchState.angle;
		stateValuesProto.has_yawAngle = true;
		stateValuesProto.yawAngle = yawState.angle;

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
static void StateInit(KalmanFilterType * Estimator)
{
  float32_t samplePeriod = GetGyroMeasuredSamplePeriod();

  /* P matrix init is the Identity matrix*/
  Estimator->p11 = 0.1;
  Estimator->p12 = 0.0;
  Estimator->p21 = 0.0;
  Estimator->p22 = 0.1;

  /* q1 = sqrt(var(rate))*STATE_ESTIMATION_SAMPLE_PERIOD^2
   * q2 = sqrt(var(rateBias))
   * r1 = sqrt(var(angle)) */
  Estimator->q1 = GYRO_AXIS_VARIANCE_ROUGH;
  Estimator->q2 = Q2_CAL;
  Estimator->r1 = R1_CAL; /* accelerometer based angle variance */
}

/*
 * @brief  State estimation sensor value update callback
 * @param  sensorType : Type of sensor (accelerometer, gyroscope, magnetometer)
 * @param  deltaT : Time difference between sensor values
 * @param  pXYZ : Pointer to sensor values array
 * @retval None
 */
static void StateSensorsEventCallback(FcbSensorIndexType sensorType, float32_t deltaT, float32_t const * pXYZ) {
    static uint8_t varianceCalcDone = 0;
    /* keep these around because yaw calculations need data already calculated
     * when accelerometer data was available
     */

    if (0 == varianceCalcDone) {
        varianceCalcDone = ProfileSensorMeasurements(sensorType, pXYZ);
        return;
    }

    switch (sensorType) { /* interpret values according to sensor type */
    case GYRO_IDX: {
        /* run correction step */
        float32_t const * pSensorAngleRate = pXYZ;
        CorrectAttitudeRateState(deltaT, pSensorAngleRate[ROLL_IDX], &rollEstimator, &rollState);
        CorrectAttitudeRateState(deltaT, pSensorAngleRate[PITCH_IDX], &pitchEstimator, &pitchState);
        CorrectAttitudeRateState(deltaT, pSensorAngleRate[YAW_IDX], &yawEstimator, &yawState);
    }
        break;
    case ACC_IDX: {
        /* run correction step */
        float32_t const * pAccMeterXYZ = pXYZ; /* interpret values as accelerations */
        GetAttitudeFromAccelerometer(sensorAttitudeRPY, pAccMeterXYZ);
        CorrectAttitudeState(sensorAttitudeRPY[ROLL_IDX], &rollEstimator, &rollState);
        CorrectAttitudeState(sensorAttitudeRPY[PITCH_IDX], &pitchEstimator, &pitchState);
    }
        break;
    case MAG_IDX: {
        /* run correction step */
        float32_t const * pMagMeter = pXYZ;
        sensorAttitudeRPY[YAW_IDX] = GetMagYawAngle((float32_t*) pMagMeter, sensorAttitudeRPY[ROLL_IDX],
                sensorAttitudeRPY[PITCH_IDX]);
        CorrectAttitudeState(sensorAttitudeRPY[YAW_IDX], &yawEstimator, &yawState);
    }
        break;
    default:
        ErrorHandler();
    }
}

/*
 * @brief	Performs the prediction step of the Kalman filtering.
 *
 * @param 	deltaT: time passed in s since last correction or estimate
 * @param   sensorRate: angular rate of gyroscope
 * @param 	pEstimator: Pointer to KalmanFilterType struct (roll pitch or yaw estimator)
 * @param 	pState: Pointer to struct member of AngleStateVectorType (roll, pitch or yaw)
 *
 * @retval 	None
 */
static void PredictAttitudeState(float32_t const deltaT, const float32_t sensorRate, KalmanFilterType* pEstimator, AttitudeStateVectorType * pState)
{
	/* Prediction */
	/* Step 1: Calculate a priori state estimation*/
	pState->angle += deltaT * sensorRate - deltaT * (pState->angleRateBias);
	/* angleRateBias not corrected, see equations in section "State Estimation Theory" */

	/* Step 2: Calculate a priori error covariance matrix P*/
	pEstimator->p11 += (deltaT*pEstimator->p22 - pEstimator->p12 - pEstimator->p21 + (pEstimator->q1 * deltaT * deltaT))*deltaT;
	pEstimator->p12 -= pEstimator->p22 * deltaT;
	pEstimator->p21 -= pEstimator->p22 * deltaT;
	pEstimator->p22 += pEstimator->q2 * deltaT;
}

/*
 * @brief	Performs the correction part of the Kalman filtering.
 * @param 	newAngle: Pointer to measured angle using accelerometer or magnetometer (roll, pitch or yaw)
 * @param 	pEstimator: Pointer to KalmanFilterType struct (roll pitch or yaw estimator)
 * @param 	stateAngle: Pointer to struct member of StateVectorType (roll, pitch or yaw)
 * @param 	stateRateBias: Pointer to struct member of StateVectorType (rollRateBias, pitchRateBias or yawRateBias)
 * @retval 	None
 */
static void CorrectAttitudeState(const float32_t sensorAngle, KalmanFilterType* pEstimator, AttitudeStateVectorType * pState)
{
	/* Correction */
	/* Step3: Calculate y, difference between a-priori state and measurement z */
	float32_t y = sensorAngle - pState->angle;

	/* Step 4: Calculate innovation covariance matrix S*/
	float32_t s = pEstimator->p11 + pEstimator->r1;

	/* Step 5: Calculate Kalman gain*/
	pEstimator->k11 = pEstimator->p11 /s;
	pEstimator->k12 = pEstimator->p21 /s;

	/* Step 6: Update a posteriori state estimation*/
	pState->angle += pEstimator->k11 * y;
	/* angleRateBias deliberately not updated as per equations, see FCB PDF */

	/* Step 7: Update a posteriori error covariance matrix P */
	float32_t p11_tmp = pEstimator->p11;
	float32_t p12_tmp = pEstimator->p12;
	pEstimator->p11 = p11_tmp - pEstimator->k11*p11_tmp;
	pEstimator->p12 = p12_tmp - pEstimator->k11*p12_tmp;
	pEstimator->p21 -= pEstimator->k12*p11_tmp;
	pEstimator->p22 -= pEstimator->k12*p12_tmp;
}

/*
 * @brief   Performs the correction part of the Kalman filtering.
 * @param   newAngle: Pointer to measured angle using accelerometer or magnetometer (roll, pitch or yaw)
 * @param   pEstimator: Pointer to KalmanFilterType struct (roll pitch or yaw estimator)
 * @param   stateAngle: Pointer to struct member of StateVectorType (roll, pitch or yaw)
 * @param   stateRateBias: Pointer to struct member of StateVectorType (rollRateBias, pitchRateBias or yawRateBias)
 * @retval  None
 */
static void CorrectAttitudeRateState(float32_t const deltaT, const float32_t sensorRate, KalmanFilterType* pEstimator,
        AttitudeStateVectorType * pState) {
    float32_t y1, y2, s11, s12, s21, s22, detS_inv, p11_tmp, p12_tmp;

    /* Correction */
    /* Step3: Calculate y, difference between a-priori state and measurement z */
   // y1 = sensorAngle - pState->angle;
    y2 = sensorRate - pState->angleRate;

    /* Step 4: Calculate innovation covariance matrix S */
    s11 = pEstimator->p11 + pEstimator->r1;
    s12 = pEstimator->p12;
    s21 = pEstimator->p21;
    s22 = pEstimator->p22 + pEstimator->r2;
    detS_inv = 1/(s11*s22 - s12*s21); // Determinant of S

    /* Step 5: Calculate Kalman gains */
    pEstimator->k21 = detS_inv * (pEstimator->p21*s22 + pEstimator->p22*s21);
    pEstimator->k22 = detS_inv * (pEstimator->p22*s11 + pEstimator->p21*s12);

    /* Step 6: Update a posteriori state estimation */
    pState->angle = pState->angle + pEstimator->k12*y2;
    pState->angleRate = pState->angleRate + pEstimator->k22*y2;
    pState->angleRateBias = pState->angleRateBias + pEstimator->k32*y2;
    /* angleRateBias deliberately not updated as per equations, see FCB PDF */

    /* Step 7: Update a posteriori error covariance matrix P */
    p11_tmp = pEstimator->p11;
    p12_tmp = pEstimator->p12;
    pEstimator->p11 = p11_tmp - pEstimator->k11*p11_tmp;
    pEstimator->p12 = p12_tmp - pEstimator->k11*p12_tmp;
    pEstimator->p21 -= pEstimator->k12*p11_tmp;
    pEstimator->p22 -= pEstimator->k12*p12_tmp;
}

/**
 * gathers VAR_SAMPLE_MAX and calculates variance to be used in
 * estimation equations.
 */
static uint8_t ProfileSensorMeasurements(FcbSensorIndexType sensorType, float32_t const * pXYZData) {
  enum {
    PROC,  /* process samples, measurement samples, in this case gyroscope */
    MEAS   /* measurement samples, angles from accelero & magnetometer */
  };

  static uint8_t sCount[FCB_SENSOR_NBR] = { 0 };

  uint8_t done = 0;

  if (NULL == pSampleData) {
    /*
     * one set of measurements in rpy for
     */
    if (NULL == (pSampleData = malloc(2 * sizeof(FcbSensorVarianceCalcType)))) {
      ErrorHandler();
    }
  }


  switch (sensorType) {
    case GYRO_IDX: {
      pSampleData[PROC].samples[ROLL_IDX][sCount[GYRO_IDX]] = pXYZData[X_IDX];
      pSampleData[PROC].samples[PITCH_IDX][sCount[GYRO_IDX]] = pXYZData[Y_IDX];
      pSampleData[PROC].samples[YAW_IDX][sCount[GYRO_IDX]] = pXYZData[Z_IDX];
    } break;
    case ACC_IDX:
      GetAttitudeFromAccelerometer(sensorAttitudeRPY, pXYZData);
      /* sample pitch measurement */
      pSampleData[MEAS].samples[ROLL_IDX][sCount[ACC_IDX]] = sensorAttitudeRPY[ROLL_IDX];
      /* sample roll measurement */
      pSampleData[MEAS].samples[PITCH_IDX][sCount[ACC_IDX]] = sensorAttitudeRPY[PITCH_IDX];
      break;
    case MAG_IDX:
        sensorAttitudeRPY[YAW_IDX] = GetMagYawAngle((float32_t*) pXYZData, sensorAttitudeRPY[ROLL_IDX], sensorAttitudeRPY[PITCH_IDX]);
      pSampleData[MEAS].samples[YAW_IDX][sCount[MAG_IDX]] = sensorAttitudeRPY[YAW_IDX];
      break;
    default:
      ErrorHandler();
  }

  sCount[sensorType] += 1;

  if (VAR_SAMPLE_MAX == sCount[sensorType]) {
    float32_t variance = 0.0f;
    float32_t mean = 0.0f;
    switch (sensorType) {
      case GYRO_IDX: {
        arm_mean_f32(pSampleData[PROC].samples[ROLL_IDX], VAR_SAMPLE_MAX, &mean);
        arm_var_f32(pSampleData[PROC].samples[ROLL_IDX], VAR_SAMPLE_MAX, &variance);
        rollState.angleRateBias = mean;
        rollEstimator.q1 = variance;
        rollEstimator.p22 = variance;

        arm_mean_f32(pSampleData[PROC].samples[PITCH_IDX], VAR_SAMPLE_MAX, &mean);
        arm_var_f32(pSampleData[PROC].samples[PITCH_IDX], VAR_SAMPLE_MAX, &variance);
        pitchState.angleRateBias = mean;
        pitchEstimator.q1 = variance;
        pitchEstimator.p22 = variance;

        arm_mean_f32(pSampleData[PROC].samples[YAW_IDX], VAR_SAMPLE_MAX, &mean);
        arm_var_f32(pSampleData[PROC].samples[YAW_IDX], VAR_SAMPLE_MAX, &variance);
        yawState.angleRateBias = mean;
        yawEstimator.q1 = variance;
        yawEstimator.p22 = variance;
      } break;
      case ACC_IDX :
        arm_var_f32(pSampleData[MEAS].samples[ROLL_IDX], VAR_SAMPLE_MAX, &variance);
        rollEstimator.r1 = variance;
        rollEstimator.p11 = variance;
        arm_var_f32(pSampleData[MEAS].samples[PITCH_IDX], VAR_SAMPLE_MAX, &variance);
        pitchEstimator.r1 = variance;
        pitchEstimator.p11 = variance;
        break;
      case MAG_IDX:
        arm_var_f32(pSampleData[MEAS].samples[YAW_IDX], VAR_SAMPLE_MAX, &variance);
        yawEstimator.r1 = variance;
        yawEstimator.p11 = variance;
        break;
      default:
        ErrorHandler();
    }
  }

  if ((VAR_SAMPLE_MAX <= sCount[GYRO_IDX]) &&
      (VAR_SAMPLE_MAX <= sCount[ACC_IDX]) &&
      (VAR_SAMPLE_MAX <= sCount[MAG_IDX])) {
    done = 1;
    free(pSampleData);
    pSampleData = NULL;
  }

  return done;
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
