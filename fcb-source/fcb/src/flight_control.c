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
static RefSignals_TypeDef RefSignals; // Control reference signals

/* Flight mode */
static enum FlightControlMode flightControlMode = FLIGHT_CONTROL_IDLE;

xTaskHandle FlightControlTaskHandle; // Task handle for flight control task

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
		// PIDControl(&RefSignals);
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
 * @brief  Sets the reference values based on RC receiver input
 * @param  None
 * @retval None
 */
static void SetReferenceSignals(void) {
	RefSignals.ZVelocity = MAX_Z_VELOCITY*GetThrottleReceiverChannel()/INT16_MAX;
	RefSignals.RollAngle = MAX_ROLLPITCH_ANGLE*GetAileronReceiverChannel()/INT16_MAX;
	RefSignals.PitchAngle = MAX_ROLLPITCH_ANGLE*GetElevatorReceiverChannel()/INT16_MAX;
	RefSignals.YawAngleRate = MAX_YAW_RATE*GetRudderReceiverChannel()/INT16_MAX;
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

		/* Update the flight control state and perform flight control activities */
		UpdateFlightControl();

		/* Blink with LED to indicate thread is alive */
		if(ledFlashCounter % 100 == 0)
			BSP_LED_On(LED6);
		else if(ledFlashCounter % 20 == 0)
			BSP_LED_Off(LED6);

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
