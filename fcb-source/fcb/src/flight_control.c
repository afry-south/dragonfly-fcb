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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FLIGHT_CONTROL_TASK_PRIO		configMAX_PRIORITIES-1
#define FLIGHT_CONTROL_TASK_PERIOD		10 // [ms]

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RefSignals_TypeDef RefSignals; // Control reference signals

/* Flight mode */
enum FlightControlMode flightControlMode = FLIGHT_CONTROL_IDLE;

xTaskHandle FlightControlTaskHandle;

/* Private function prototypes -----------------------------------------------*/
static void UpdateFlightControl(void);
static void SetFlightMode(void);
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
	 * Stack depth: configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: FLIGHT_CONTROL_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: FlightControlTaskHandle
	 * */
	if (pdPASS
			!= xTaskCreate((pdTASK_CODE )FlightControlTask, (signed portCHAR*)"FLIGHT_CTRL", configMINIMAL_STACK_SIZE,
					NULL, FLIGHT_CONTROL_TASK_PRIO, &FlightControlTaskHandle)) {
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

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Performs program duties with regular intervals.
 * @param  None.
 * @retval None.
 */
static void UpdateFlightControl(void) {

	/* Updates the current flight mode */
	SetFlightMode();

	switch (flightControlMode) {

	case FLIGHT_CONTROL_IDLE:
		ShutdownMotors();
		return;

	case FLIGHT_CONTROL_RAW:
		// TODO Check that motors are armed
		MotorAllocationRaw();
		return;

	case FLIGHT_CONTROL_PID:
		// TODO Check that motors are armed
		SetReferenceSignals();
		// Do PID control
		// Set motors
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
static void SetFlightMode(void) {
	if (!IsReceiverActive()) {
		flightControlMode = FLIGHT_CONTROL_IDLE;
	} else if (GetReceiverRawFlightSet()) {
		flightControlMode = FLIGHT_CONTROL_RAW;
	} else {
		flightControlMode = FLIGHT_CONTROL_IDLE;
	}
}

/*
 * @brief  Sets the reference values based on RC receiver input
 * @param  None
 * @retval None
 */
static void SetReferenceSignals(void) {
#ifdef TODO
	"reimplement this for new receiver code"

	// Set velocity reference limits
	if (PWMInputTimes.Throttle >= GetRCmin() && PWMInputTimes.Throttle <= GetRCmax())
		RefSignals.ZVelocity = 2 * MAX_Z_VELOCITY * 1000 * (PWMInputTimes.Throttle - GetRCmid());
	else
		RefSignals.ZVelocity = -MAX_Z_VELOCITY;

	// Set roll reference limits
	if (PWMInputTimes.Aileron >= GetRCmin() && PWMInputTimes.Aileron <= GetRCmax())
	RefSignals.RollAngle = 2 * MAX_ROLLPITCH_ANGLE * 1000
	* (PWMInputTimes.Aileron - GetRCmid());
	else
	RefSignals.RollAngle = GetRCmid();

	// Set pitch reference limits
	if (PWMInputTimes.Elevator >= GetRCmin()
			&& PWMInputTimes.Elevator <= GetRCmax())
	RefSignals.PitchAngle = 2 * MAX_ROLLPITCH_ANGLE * 1000
	* (PWMInputTimes.Elevator - GetRCmid());
	else
	RefSignals.PitchAngle = GetRCmid();

	// Set yaw rate reference limits
	if (PWMInputTimes.Rudder >= GetRCmin() && PWMInputTimes.Rudder <= GetRCmax())
	RefSignals.YawRate = 2 * MAX_YAW_RATE * 1000
	* (PWMInputTimes.Rudder - GetRCmid());
	else
	RefSignals.YawRate = GetRCmid();
#endif
}

/**
 * @brief  Flight control task function
 * @param  argument : Unused parameter
 * @retval None
 */
static void FlightControlTask(void const *argument) {
	(void) argument;

	portTickType xLastWakeTime;

	/* Initialise the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, FLIGHT_CONTROL_TASK_PERIOD);

		/* Update the flight control state and perform flight control activities */
		UpdateFlightControl();
	}
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
