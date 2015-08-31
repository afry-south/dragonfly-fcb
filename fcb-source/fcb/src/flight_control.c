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

#include "receiver.h"
#include "motor_control.h"
#include "pid_control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RefSignals_TypeDef RefSignals; // Control reference signals

/* Flight mode */
enum FlightControlMode flightMode = FLIGHT_CONTROL_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void UpdateControl(void);
static void SetFlightMode(void);
static void SetReferenceSignals(void);

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Performs program duties with regular intervals.
 * @param  None.
 * @retval None.
 */
static void UpdateControl(void) {

	/* Updates the current flight mode */
	SetFlightMode();

	switch (flightMode) {

	case FLIGHT_CONTROL_IDLE:
		ShutdownMotors();
		return;

	case FLIGHT_CONTROL_RAW:
		// Check that motors are armed
		RawFlightControl();
		return;

	case FLIGHT_CONTROL_PID:
		// Check that motors are armed
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
		flightMode = FLIGHT_CONTROL_IDLE;
	} else if (GetReceiverRawFlightSet()) {
		flightMode = FLIGHT_CONTROL_RAW;
	} else {
		flightMode = FLIGHT_CONTROL_IDLE;
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
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
