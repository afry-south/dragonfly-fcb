/******************************************************************************
 * @brief   Flight control module responsible for executing flight control
 * 			activities
 *
 * @license
 * Dragonfly FCB firmware to control the Dragonfly quadrotor UAV
 * Copyright (C) 2016  ÅF Technology South: Dragonfly Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flight_control.h"

#include "stm32f3_discovery.h"

#include "fcb_error.h"
#include "receiver.h"
#include "motor_control.h"
#include "pid_control.h"
#include "fcb_gyroscope.h"
#include "rotation_transformation.h"
#include "state_estimation.h"
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_barometer.h"
#include "flash.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/

typedef enum {
  FLIGHT_CONTROL_UPDATE,
  CORRECTION_UPDATE,
  PREDICTION_UPDATE
} FlightControlMsgType_TypeDef;

typedef struct {
  FcbSensorIndexType sensorType;
  float32_t xyz[3];
} sensorReading_TypeDef;

/**
 * Messages sent to the queue
 */
typedef struct FlightControlMsg {
  sensorReading_TypeDef sensorReading;
  FlightControlMsgType_TypeDef type;
} FlightControlMsg_TypeDef;

/* Private define ------------------------------------------------------------*/
#define FLIGHT_CONTROL_TASK_PRIO		configMAX_PRIORITIES-1

#define FLIGHT_CONTROL_QUEUE_SIZE		      6
#define FLIGHT_CONTROL_QUEUE_TIMEOUT          2000 // [ms]

#define GOT_GYRO_SENSOR_SAMPLE  1
#define GOT_ACC_SENSOR_SAMPLE   2
#define GOT_MAG_SENSOR_SAMPLE   4
#define GOT_ALL_SENSOR_SAMPLES  (GOT_ACC_SENSOR_SAMPLE | GOT_MAG_SENSOR_SAMPLE)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static xQueueHandle qFlightControl = NULL;

static RefSignals_TypeDef refSignals; // Control reference signals
static RefSignals_TypeDef refSignalsLimits; // Max limits for reference signals
static CtrlSignals_TypeDef ctrlSignals; // Physical control signals

/* Flight mode */
static enum FlightControlMode flightControlMode = FLIGHT_CONTROL_IDLE;

xTaskHandle FlightControlTaskHandle; // Task handle for flight control task

/* Private function prototypes -----------------------------------------------*/
static void UpdateFlightControl(void);
static void UpdateFlightMode(void);
static void SetRefSignals(void);

void setMaxLimitForReferenceSignalToDefault(void);

static void FlightControlTask(void const *argument);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Creates flight control task.
 * @param  None.
 * @retval None.
 */
void CreateFlightControlTask(void) {
    if (0 == (qFlightControl = xQueueCreate(FLIGHT_CONTROL_QUEUE_SIZE, sizeof(FlightControlMsg_TypeDef)))) {
        ErrorHandler();
    }

	/* Flight Control task creation
	 * Task function pointer: FlightControlTask
	 * Task name: FLIGHT_CTRL
	 * Stack depth: 2*configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: FLIGHT_CONTROL_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: FlightControlTaskHandle
	 * */
	if (pdPASS != xTaskCreate((pdTASK_CODE )FlightControlTask, (signed portCHAR*)"FLIGHT_CTRL",
			2*configMINIMAL_STACK_SIZE, NULL, FLIGHT_CONTROL_TASK_PRIO, &FlightControlTaskHandle)) {
		ErrorHandler();
	}
}

/**
 * @brief  Returns the current flight control mode.
 * @param  None.
 * @retval Flight control mode enum
 */
enum FlightControlMode GetFlightControlMode(void) {
	return flightControlMode;
}

/*
 * @brief  Get Z velocity reference
 * @param  None
 * @retval Z velocity reference signal value
 */
float32_t GetZVelocityReferenceSignal(void) {
	return refSignals.zVelocity;
}

/*
 * @brief  Get roll angle reference
 * @param  None
 * @retval Roll angle reference signal value
 */
float32_t GetRollAngleReferenceSignal(void) {
	return refSignals.rollAngle;
}

/*
 * @brief  Get pitch angle reference
 * @param  None
 * @retval Pitch angle reference signal value
 */
float32_t GetPitchAngleReferenceSignal(void) {
	return refSignals.pitchAngle;
}

/*
 * @brief  Get yaw angle reference
 * @param  None
 * @retval Pitch angle reference signal value
 */
float32_t GetYawAngleReferenceSignal(void) {
    return refSignals.yawAngle;
}

/*
 * @brief  Get yaw angular velocity reference
 * @param  None
 * @retval Yaw angular velocity reference signal value
 */
float32_t GetYawAngularRateReferenceSignal(void) {
	return refSignals.yawAngleRate;
}

/*
 * @brief  Gets thrust force control signal, which is an approximate of the real thrust force [N]
 * @param  None
 * @retval Thrust force [N]
 */
float32_t GetThrustControlSignal(void) {
	return ctrlSignals.thrust;
}

/*
 * @brief  Gets roll moment control signal, which is an approximate of the real roll moment [Nm]
 * @param  None
 * @retval Roll moment [Nm]
 */
float32_t GetRollControlSignal(void) {
	return ctrlSignals.rollMoment;
}

/*
 * @brief  Gets pitch moment control signal, which is an approximate of the real pitch moment [Nm]
 * @param  None
 * @retval Pitch moment [Nm]
 */
float32_t GetPitchControlSignal(void) {
	return ctrlSignals.pitchMoment;
}

/*
 * @brief  Gets yaw moment control signal, which is an approximate of the real yaw moment [Nm]
 * @param  None
 * @retval Yaw moment [Nm]
 */
float32_t GetYawControlSignal(void) {
	return ctrlSignals.yawMoment;
}

/*
 * @brief  Resets control reference signals
 * @param  refSignals : Reference signals type struct
 * @retval None
 */
void ResetRefSignals(RefSignals_TypeDef* refSignals) {
	refSignals->rollAngle = 0.0;
	refSignals->pitchAngle = 0.0;
    refSignals->yawAngle = 0.0;
	refSignals->yawAngleRate = 0.0;
	refSignals->zVelocity = 0.0;
}

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Performs program flight control duties with regular intervals.
 * @param  None.
 * @retval None.
 */
static void UpdateFlightControl(void) {

	/* Updates the current flight mode */
	UpdateFlightMode();

	switch (flightControlMode) {

	case FLIGHT_CONTROL_IDLE:
		ShutdownMotors();

		/* Reset control signals, values and parameters */
		ResetCtrlSignals(&ctrlSignals);
		ResetRefSignals(&refSignals);
		InitPIDControllers();	// Set PID control variables to initial values

		return;

	case FLIGHT_CONTROL_RAW:
		MotorAllocationRaw();

		/* Reset control signals, values and parameters */
		ResetCtrlSignals(&ctrlSignals);
		ResetRefSignals(&refSignals);
		InitPIDControllers();	// Set PID control variables to initial values

		return;

	case FLIGHT_CONTROL_PID:
		/* Set the control reference signals based on RC receiver input */
		SetRefSignals();

		/* Update PID control output */
		ctrlSignals.thrust = -(GetThrottleReceiverChannel()-INT16_MIN)*MAX_THRUST/((float32_t)UINT16_MAX); // NOTE: Raw throttle control for now until control developed for Z position/velocity
		UpdatePIDControlSignals(&ctrlSignals);

		/* Allocate control signal action to motors based on thrust/torque<->motor signal mapping for each motor */
		MotorAllocationPhysical(ctrlSignals.thrust, ctrlSignals.rollMoment, ctrlSignals.pitchMoment, ctrlSignals.yawMoment);

		return;

	case FLIGHT_CONTROL_AUTONOMOUS:
		// TODO : Should get reference signals from FMS board and call position controllers
		return;

	default:
		ShutdownMotors();

		return;
	}
}

/*
 * @brief  Sets the Flight Mode - handles transitions between flight modes.
 * @param  None.
 * @retval None.
 */
static void UpdateFlightMode(void) {
	if (!IsReceiverActive())
		flightControlMode = FLIGHT_CONTROL_IDLE;
	else if (GetReceiverRawFlightSet())
		flightControlMode = FLIGHT_CONTROL_RAW;
	else if (GetReceiverPIDFlightSet())
		flightControlMode = FLIGHT_CONTROL_PID;
	else
		flightControlMode = FLIGHT_CONTROL_IDLE;
}

/*
 * @brief  Sets the reference values based on RC receiver input. This sets the limits for maximum pilot input.
 * @param  None
 * @retval None
 */
static void SetRefSignals(void) {
	int32_t throttle, aileron, elevator, rudder;

	throttle = GetThrottleReceiverChannel();
	aileron = GetAileronReceiverChannel();
	elevator = GetElevatorReceiverChannel();
	rudder = GetRudderReceiverChannel();

	/* Set Z velocity reference depending on receiver throttle channel */
	// TODO set Z velocity reference to control altitude when such a controller is available
	/*
	if (throttle <= RECEIVER_TO_REFERENCE_ZERO_PADDING && throttle >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		refSignals.zVelocity = 0.0;
	} else if (throttle >= 0) {
		refSignals.zVelocity = -refSignalsLimits.zVelocity*(throttle - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING); // Negative sign because Z points downwards
	} else {
		refSignals.zVelocity = -refSignalsLimits.zVelocity*(throttle + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING); // Negative sign because Z points downwards
	}
	*/

	/* Set roll angle reference depending on receiver aileron channel */
	if (aileron <= RECEIVER_TO_REFERENCE_ZERO_PADDING && aileron >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		refSignals.rollAngle = 0.0;
	} else if (aileron >= 0) {
		refSignals.rollAngle = -refSignalsLimits.rollAngle*(aileron - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		refSignals.rollAngle = -refSignalsLimits.rollAngle*(aileron + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}

	/* Set pitch angle reference depending on receiver elevator channel */
	if (elevator <= RECEIVER_TO_REFERENCE_ZERO_PADDING && elevator >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		refSignals.pitchAngle = 0.0;
	} else if (elevator >= 0) {
		refSignals.pitchAngle = -refSignalsLimits.pitchAngle*(elevator - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		refSignals.pitchAngle = -refSignalsLimits.pitchAngle*(elevator + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}

    /* Set yaw reference depending on receiver rudder channel */
	// TODO Use yaw angle reference instead of yaw rate ref? Should be able to go between (-180, 180] degrees
	// Needs special handling, should not have a zero padding zone but should perhaps have some padding relative
	// to the current
//    if (rudder <= RECEIVER_TO_REFERENCE_ZERO_PADDING && rudder >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
//        refSignals.yawAngle = 0.0;
//    } else if (rudder >= 0) {
//        refSignals.yawAngle = -refSignalsLimits.yawAngle*(rudder - RECEIVER_TO_REFERENCE_ZERO_PADDING)
//                / (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
//    } else {
//        refSignals.yawAngle = -refSignalsLimits.yawAngle*(rudder + RECEIVER_TO_REFERENCE_ZERO_PADDING)
//                / (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
//    }

	/* Set yaw rate reference depending on receiver rudder channel */
	if (rudder <= RECEIVER_TO_REFERENCE_ZERO_PADDING && rudder >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		refSignals.yawAngleRate = 0.0;
	} else if (rudder >= 0) {
		refSignals.yawAngleRate = -refSignalsLimits.yawAngleRate*(rudder - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		refSignals.yawAngleRate = -refSignalsLimits.yawAngleRate*(rudder + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}

	/* Go to "safe" reference signal values if receiver transmission becomes inactive */
	if(RECEIVER_OK != IsReceiverActive()) {
	    refSignals.rollAngle = 0.0;
	    refSignals.pitchAngle = 0.0;
	    refSignals.yawAngleRate = 0.0;
	    refSignals.zVelocity = 0.0;
	}
}

void SendFlightControlUpdateToFlightControl(void)
{
    FlightControlMsg_TypeDef msg;
    msg.type = FLIGHT_CONTROL_UPDATE;
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (pdTRUE != xQueueSendFromISR(qFlightControl, &msg, &higherPriorityTaskWoken)) {
        ErrorHandler();
    }
}

void SendPredictionUpdateToFlightControl(void)
{
    FlightControlMsg_TypeDef msg;
    msg.type = PREDICTION_UPDATE;
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (pdTRUE != xQueueSendFromISR(qFlightControl, &msg, &higherPriorityTaskWoken)) {
        ErrorHandler();
    }
}

void SendCorrectionUpdateToFlightControl(FcbSensorIndexType sensorType, float32_t xyz[3])
{
    FlightControlMsg_TypeDef msg;
    msg.type = CORRECTION_UPDATE;
    msg.sensorReading.sensorType = sensorType;
    memcpy(msg.sensorReading.xyz, xyz, sizeof(float32_t)*3);
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (pdTRUE != xQueueSend(qFlightControl, &msg, portMAX_DELAY)) {
        ErrorHandler();
    }
}

void initKalmanFiler(void) {
    float32_t startupSensorValues[3];
    uint32_t nbrOfSamples[3] = {0, 0, 0};
	FlightControlMsg_TypeDef msg;

	// Get some samples from accelerometer and magnetometer to be used as start values for Kalman filter.
    while (nbrOfSamples[ACC_IDX] < 5 && nbrOfSamples[MAG_IDX] < 5) {
    	xQueueReceive(qFlightControl, &msg,  FLIGHT_CONTROL_QUEUE_TIMEOUT);

    	switch (msg.sensorReading.sensorType) {
//    	case GYRO_IDX:
//    		gotFirstSensorSamples &= GOT_GYRO_SENSOR_SAMPLE;
//    		break;
    	case ACC_IDX:
    		startupSensorValues[0] = msg.sensorReading.xyz[0];
    		startupSensorValues[1] = msg.sensorReading.xyz[1];
    		nbrOfSamples[ACC_IDX]++;
    		break;
    	case MAG_IDX:
    		if (nbrOfSamples[ACC_IDX]) {
    			startupSensorValues[2] = GetMagYawAngle(msg.sensorReading.xyz, startupSensorValues[0], startupSensorValues[1]);
    			nbrOfSamples[MAG_IDX]++;
    		}
    		break;
    	default:
			break;
    	}
    }

    /* Init the states for the Kalman filter */
    InitStatesXYZ(startupSensorValues);
    InitStateEstimationTimeEvent();
}

void setMaxLimitForReferenceSignal(float32_t maxZVelocity, float32_t maxRollAngle, float32_t maxPitchAngle, float32_t maxYawAngleRate) {
	refSignalsLimits.zVelocity = maxZVelocity;
	refSignalsLimits.rollAngle = maxRollAngle;
	refSignalsLimits.pitchAngle = maxPitchAngle;
	refSignalsLimits.yawAngle = DEFAULT_MAX_YAW_ANGLE; // Should always be 180 deg
	refSignalsLimits.yawAngleRate = maxYawAngleRate;

	WriteReferenceMaxLimitsToFlash(&refSignalsLimits);
}

void setMaxLimitForReferenceSignalToDefault(void) {
	refSignalsLimits.zVelocity = DEFAULT_MAX_Z_VELOCITY;
	refSignalsLimits.rollAngle = DEFAULT_MAX_ROLLPITCH_ANGLE;
	refSignalsLimits.pitchAngle = DEFAULT_MAX_ROLLPITCH_ANGLE;
	refSignalsLimits.yawAngle = DEFAULT_MAX_YAW_ANGLE;
	refSignalsLimits.yawAngleRate = DEFAULT_MAX_YAW_RATE;
}

void getMaxLimitForReferenceSignal(float32_t* maxZVelocity, float32_t* maxRollAngle, float32_t* maxPitchAngle, float32_t* maxYawAngle, float32_t* maxYawAngleRate) {
	*maxZVelocity = refSignalsLimits.zVelocity;
	*maxRollAngle = refSignalsLimits.rollAngle;
	*maxPitchAngle = refSignalsLimits.pitchAngle;
	*maxYawAngle = refSignalsLimits.yawAngle;
	*maxYawAngleRate = refSignalsLimits.yawAngleRate;
}

/**
 * @brief  Flight control task function
 * @param  argument : Unused parameter
 * @retval None
 */
static void FlightControlTask(void const *argument) {
	(void) argument;

	uint32_t ledFlashCounter = 0;

    if (SensorRegisterAccClientCallback(SendCorrectionUpdateToFlightControl)) {
    	ErrorHandler();
    }
    if (SensorRegisterGyroClientCallback(SendCorrectionUpdateToFlightControl)) {
    	ErrorHandler();
    }
    if (SensorRegisterBaroClientCallback(SendCorrectionUpdateToFlightControl)) {
    	ErrorHandler();
    }

	if (FLASH_OK != ReadReferenceMaxLimitsFromFlash(&refSignalsLimits)) {
		setMaxLimitForReferenceSignalToDefault();
	}

    initKalmanFiler();

	for (;;) {
        FlightControlMsg_TypeDef msg;

        if (pdFALSE == xQueueReceive(qFlightControl, &msg,  FLIGHT_CONTROL_QUEUE_TIMEOUT)) {
            /*
             * if no message was received, interrupts from the sensors
             * aren't arriving and this is a serious error.
             */
            ErrorHandler();
        }

        switch (msg.type) {
        case PREDICTION_UPDATE:
            UpdatePredictionState();
            // Intended fall through. But has to comment next case state to remove warning.
        //case FLIGHT_CONTROL_UPDATE:
            /* Perform flight control activities */
            UpdateFlightControl();

            /* Blink with LED to indicate thread is alive */
            if(ledFlashCounter % 200 == 0) {
            	BSP_LED_Toggle(LED6);
            }
            ledFlashCounter++;

        	switch (flightControlMode) {
        	case FLIGHT_CONTROL_IDLE:
        		BSP_LED_Off(LED10);
        		BSP_LED_Off(LED3);
        		break;
        	case FLIGHT_CONTROL_RAW:
        		BSP_LED_On(LED10);
        		BSP_LED_Off(LED3);
        		break;
        	default:
        		BSP_LED_On(LED10);
        		BSP_LED_On(LED3);
        		break;
        	}

            break;
        case CORRECTION_UPDATE:
            UpdateCorrectionState(msg.sensorReading.sensorType, msg.sensorReading.xyz);
            break;
        default:
            break;
        }
    }
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
