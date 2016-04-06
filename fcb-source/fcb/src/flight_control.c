/******************************************************************************
 * @file    flight_control.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-31
 * @brief   Flight control module responsible for executing flight control
 * 			activities
 ******************************************************************************/

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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  float ZVelocity;		// [m/s]
  float RollAngle;		// [rad]
  float PitchAngle;		// [rad]
  float YawAngleRate;	// [rad/s]
} RefSignals_TypeDef;

/* Private define ------------------------------------------------------------*/
#define FLIGHT_CONTROL_TASK_PRIO		configMAX_PRIORITIES-1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static RefSignals_TypeDef RefSignals; // Control reference signals
static CtrlSignals_TypeDef ctrlSignals; // Physical control signals

/* Flight mode */
static enum FlightControlMode flightControlMode = FLIGHT_CONTROL_IDLE;

xTaskHandle FlightControlTaskHandle; // Task handle for flight control task

/* Private function prototypes -----------------------------------------------*/
static void UpdateFlightControl(void);
static void UpdateFlightMode(void);
static void SetReferenceSignals(void);

static void FlightControlTask(void const *argument);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Creates flight control task.
 * @param  None.
 * @retval None.
 */
void CreateFlightControlTask(void) {
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
float GetZVelocityReferenceSignal(void) {
	return RefSignals.ZVelocity;
}

/*
 * @brief  Get roll angle reference
 * @param  None
 * @retval Roll angle reference signal value
 */
float GetRollAngleReferenceSignal(void) {
	return RefSignals.RollAngle;
}

/*
 * @brief  Get pitch angle reference
 * @param  None
 * @retval Pitch angle reference signal value
 */
float GetPitchAngleReferenceSignal(void) {
	return RefSignals.PitchAngle;
}

/*
 * @brief  Get yaw angular velocity reference
 * @param  None
 * @retval Yaw angular velocity reference signal value
 */
float GetYawAngularRateReferenceSignal(void) {
	return RefSignals.YawAngleRate;
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
		InitPIDControllers();	// Set PID control variables to initial values
		return;

	case FLIGHT_CONTROL_RAW:
		// TODO Check that motors are armed
		MotorAllocationRaw();
		return;

	case FLIGHT_CONTROL_PID:
		// TODO Check that motors are armed

		/* Set the control reference signals*/
		SetReferenceSignals();

		/* Update PID control output */
		ctrlSignals.Thrust = (GetThrottleReceiverChannel()-INT16_MIN)*MAX_THRUST/UINT16_MAX; // NOTE: Raw throttle control fow now until control developed for Z velocity
		UpdatePIDControlSignals(&ctrlSignals);

		/* Allocate control signal action to motors */
		MotorAllocationPhysical(ctrlSignals.Thrust, ctrlSignals.RollMoment, ctrlSignals.PitchMoment, ctrlSignals.YawMoment);
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
static void SetReferenceSignals(void) {
	int32_t throttle, aileron, elevator, rudder;

	throttle = GetThrottleReceiverChannel();
	aileron = GetAileronReceiverChannel();
	elevator = GetElevatorReceiverChannel();
	rudder = GetRudderReceiverChannel();

	/* Set Z velocity reference depending on receiver throttle channel */
	// TODO set Z velocity reference to control altitude when such a controller is available
//	if (throttle <= RECEIVER_TO_REFERENCE_ZERO_PADDING && throttle >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
//		RefSignals.ZVelocity = 0.0;
//	} else if (throttle >= 0) {
//		RefSignals.ZVelocity = -MAX_Z_VELOCITY*(throttle - RECEIVER_TO_REFERENCE_ZERO_PADDING)
//				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING); // Negative sign because Z points downwards
//	} else {
//		RefSignals.ZVelocity = -MAX_Z_VELOCITY*(throttle + RECEIVER_TO_REFERENCE_ZERO_PADDING)
//				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING); // Negative sign because Z points downwards
//	}

	/* Set roll angle reference depending on receiver aileron channel */
	if (aileron <= RECEIVER_TO_REFERENCE_ZERO_PADDING && aileron >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		RefSignals.RollAngle = 0.0;
	} else if (aileron >= 0) {
		RefSignals.RollAngle = -MAX_ROLLPITCH_ANGLE*(aileron - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		RefSignals.RollAngle = -MAX_ROLLPITCH_ANGLE*(aileron + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}

	/* Set pitch angle reference depending on receiver elevator channel */
	if (elevator <= RECEIVER_TO_REFERENCE_ZERO_PADDING && elevator >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		RefSignals.PitchAngle = 0.0;
	} else if (elevator >= 0) {
		RefSignals.PitchAngle = -MAX_ROLLPITCH_ANGLE*(elevator - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		RefSignals.PitchAngle = -MAX_ROLLPITCH_ANGLE*(elevator + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}

	/* Set yaw rate reference depending on receiver rudder channel */
	if (rudder <= RECEIVER_TO_REFERENCE_ZERO_PADDING && rudder >= -RECEIVER_TO_REFERENCE_ZERO_PADDING) {
		RefSignals.YawAngleRate = 0.0;
	} else if (rudder >= 0) {
		RefSignals.YawAngleRate = -MAX_YAW_RATE*(rudder - RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (INT16_MAX - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	} else {
		RefSignals.YawAngleRate = -MAX_YAW_RATE*(rudder + RECEIVER_TO_REFERENCE_ZERO_PADDING)
				/ (-INT16_MIN - RECEIVER_TO_REFERENCE_ZERO_PADDING);
	}
}

/**
 * @brief  Flight control task function
 * @param  argument : Unused parameter
 * @retval None
 */
static void FlightControlTask(void const *argument) {
	(void) argument;

	portTickType xLastWakeTime;
	uint32_t ledFlashCounter = 0;

	/* Initialise the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();


	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, FLIGHT_CONTROL_TASK_PERIOD);

		/* Perform flight control activities */
		UpdateFlightControl();

		/* Blink with LED to indicate thread is alive */
		if(ledFlashCounter % 100 == 0) {
			BSP_LED_On(LED6);
		}
		else if(ledFlashCounter % 20 == 0) {
			BSP_LED_Off(LED6);
		}

		ledFlashCounter++;
	}
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
