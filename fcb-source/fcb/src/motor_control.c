/*****************************************************************************
 * @file    motor_control.c
 * @author  Dragonfly
 * @version v. 0.0.1
 * @date    2014-09-29
 * @brief   File contains PWM output configuration and handling functions. PWM
 *          pulses of ~1-2 ms are used to control the ESC:s, which in turn control
 *          Dragonfly's motors.
 *
 *          The ESC:s are of the T-motor brand and can
 *          withstand up to 30 A continuous current. They take up to 400 Hz pulse
 *          control signals.
 *
 *          The motors are also of T-motor brand (U3 Power Type model) with a KV
 *          value of 700. Coupled with the 11x3.7 carbon fibre propellers, they
 *          can spin at up to 8700 rpm providing a lifting force of ~12 N.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"
#include "usbd_cdc_if.h"
#include "fcb_error.h"
#include "receiver.h"
#include "common.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MOTOR_CONTROL_PRINT_SAMPLING_TASK_PRIO			1
#define MOTOR_CONTROL_PRINT_MINIMUM_SAMPLING_TIME		2	// Motor control updated every 2.5 ms
#define MOTOR_CONTROL_PRINT_MAX_STRING_SIZE				128

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
typedef struct {
	uint16_t Motor1;
	uint16_t Motor2;
	uint16_t Motor3;
	uint16_t Motor4;
} MotorControlValues_TypeDef;

/* Motor control value struct declaration*/
MotorControlValues_TypeDef MotorControlValues;

/* Timer time base handler */
TIM_HandleTypeDef MotorControlTimHandle;

/* Task handle for printing of sensor values task */
xTaskHandle MotorControlPrintSamplingTaskHandle = NULL;
static volatile uint16_t motorControlPrintSampleTime;
static volatile uint16_t motorControlPrintSampleDuration;

/* Private function prototypes -----------------------------------------------*/
static void SetMotor1(uint16_t ctrlVal);
static void SetMotor2(uint16_t ctrlVal);
static void SetMotor3(uint16_t ctrlVal);
static void SetMotor4(uint16_t ctrlVal);
static void MotorControlPrintSamplingTask(void const *argument);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes and configures the timer used to produce PWM output to control the ESC:s with.
 * @param  None.
 * @retval None.
 */
void MotorControlConfig(void) {
	/*##-1- Configure the TIM peripheral #######################################*/

	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef ocConfig;

	/* Initialize Motor TIM peripheral timebase */
	MotorControlTimHandle.Instance = TIM_MOTOR;
	MotorControlTimHandle.Init.Prescaler = SystemCoreClock / MOTOR_OUTPUT_COUNTER_CLOCK - 1;
	MotorControlTimHandle.Init.Period = MOTOR_OUTPUT_PERIOD;
	MotorControlTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	MotorControlTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_PWM_Init(&MotorControlTimHandle) != HAL_OK) {
		/* Capture initialization Error */
		ErrorHandler();
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	ocConfig.OCMode = TIM_OCMODE_PWM1;
	ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	ocConfig.OCFastMode = TIM_OCFAST_ENABLE;

	/* Set the pulse value for Motor 1 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&MotorControlTimHandle, &ocConfig, MOTOR1_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 2 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&MotorControlTimHandle, &ocConfig, MOTOR2_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 3 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&MotorControlTimHandle, &ocConfig, MOTOR3_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 4 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&MotorControlTimHandle, &ocConfig, MOTOR4_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/*##-3- Start PWM signals generation #######################################*/
	/* Start Motor 1 channel */
	if (HAL_TIM_PWM_Start(&MotorControlTimHandle, MOTOR1_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 2 channel */
	if (HAL_TIM_PWM_Start(&MotorControlTimHandle, MOTOR2_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 3 channel */
	if (HAL_TIM_PWM_Start(&MotorControlTimHandle, MOTOR3_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 4 channel */
	if (HAL_TIM_PWM_Start(&MotorControlTimHandle, MOTOR4_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motors 1-4
 * @param  ctrlValMotorX: value [0,65535] indicating amount of motor thrust. X == 1, 2, 3, 4
 * @retval None.
 */
void SetMotors(uint16_t ctrlValMotor1, uint16_t ctrlValMotor2, uint16_t ctrlValMotor3, uint16_t ctrlValMotor4) {
	SetMotor1(ctrlValMotor1);
	SetMotor2(ctrlValMotor2);
	SetMotor3(ctrlValMotor3);
	SetMotor4(ctrlValMotor4);
}

/*
 * @brief  Prints motor control signal values
 * @param  None.
 * @retval None.
 */
void PrintMotorControlValues(void) {
	static char motorCtrlString[MOTOR_CONTROL_PRINT_MAX_STRING_SIZE];

	snprintf((char*) motorCtrlString, MOTOR_CONTROL_PRINT_MAX_STRING_SIZE,
			"Motor control (uint16):\nM1: %u\nM2: %u\nM3: %u\nM4: %u\n\r\n", MotorControlValues.Motor1,
			MotorControlValues.Motor2, MotorControlValues.Motor3, MotorControlValues.Motor4);

	USBComSendString(motorCtrlString);
}

/*
 * @brief  Allocates RC receiver input directly to motor output
 * @param  None.
 * @retval None.
 */
void MotorAllocationRaw(void) {
	int32_t u1, u2, u3, u4, m1, m2, m3, m4;

	if (IsReceiverActive()) {
		u1 = (GetThrottleReceiverChannel()-INT16_MIN);
		u2 = u1*GetAileronReceiverChannel()/INT16_MAX;
		u3 = u1*GetElevatorReceiverChannel()/INT16_MAX;
		u4 = u1*GetRudderReceiverChannel()/INT16_MAX;

		// Motor 2 and 4 CW, Motor 1 and 3 CCW
		// TODO make define that specifies motor location on aircraft and rotational direction
		m1 = u1 + u2 - u3 + MOTOR_CHANNEL1_ROTATION_DIRECTION*u4;
		m2 = u1 - u2 - u3 + MOTOR_CHANNEL2_ROTATION_DIRECTION*u4;
		m3 = u1 - u2 + u3 + MOTOR_CHANNEL3_ROTATION_DIRECTION*u4;
		m4 = u1 + u2 + u3 + MOTOR_CHANNEL4_ROTATION_DIRECTION*u4;

		/* Check unsigned 16-bit overflow */
		if (!IS_NOT_GREATER_UINT16_MAX(m1))
			m1 = UINT16_MAX;
		if (!IS_NOT_GREATER_UINT16_MAX(m2))
			m2 = UINT16_MAX;
		if (!IS_NOT_GREATER_UINT16_MAX(m3))
			m3 = UINT16_MAX;
		if (!IS_NOT_GREATER_UINT16_MAX(m4))
			m4 = UINT16_MAX;

		/* Check unsigned 16-bit underflow (less than zero) */
		if (!IS_POS(m1))
			m1 = 0;
		if (!IS_POS(m2))
			m2 = 0;
		if (!IS_POS(m3))
			m3 = 0;
		if (!IS_POS(m4))
			m4 = 0;

		/* Set the motor signal values */
		SetMotors(m1, m2, m3, m4);
	} else {
		ShutdownMotors();
	}
}

/*
 * @brief  Turns off all the motors
 * @param  None.
 * @retval None.
 */
void ShutdownMotors(void) {
	/* Set the output compare pulses to zero width */
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR1_CHANNEL, 0);
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR2_CHANNEL, 0);
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR3_CHANNEL, 0);
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR4_CHANNEL, 0);

	/* Set the motor struct values to zero */
	MotorControlValues.Motor1 = 0;
	MotorControlValues.Motor2 = 0;
	MotorControlValues.Motor3 = 0;
	MotorControlValues.Motor4 = 0;
}

/*
 * @brief  Allocates the desired thrust force and moments to corresponding motor action. Data has been fitted to map
 * 		   thrust force [N] and roll/pitch/yaw moments [Nm] to motor output PWM widths [us] of each of the four motors.
 * @param  None.
 * @retval None.
 */
// TODO below is old code, but we need motor control allocated with mapping to thrust force and rotational moments
// void ControlAllocation(void) {
//	PWMMotorTimes.M1 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
//			- M_SQRT2 * BQ * CtrlSignals.Roll - M_SQRT2 * BQ * CtrlSignals.Pitch
//			- AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
//			/ ((float) 4 * AT * BQ * LENGTH_ARM);
//	PWMMotorTimes.M2 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
//			+ M_SQRT2 * BQ * CtrlSignals.Roll - M_SQRT2 * BQ * CtrlSignals.Pitch
//			+ AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
//			/ ((float) 4 * AT * BQ * LENGTH_ARM);
//	PWMMotorTimes.M3 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
//			+ M_SQRT2 * BQ * CtrlSignals.Roll + M_SQRT2 * BQ * CtrlSignals.Pitch
//			- AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
//			/ ((float) 4 * AT * BQ * LENGTH_ARM);
//	PWMMotorTimes.M4 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
//			- M_SQRT2 * BQ * CtrlSignals.Roll + M_SQRT2 * BQ * CtrlSignals.Pitch
//			+ AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
//			/ ((float) 4 * AT * BQ * LENGTH_ARM);

//  if (PWMMotorTimes.M1 > MAX_ESC_VAL)
//    PWMMotorTimes.M1 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M1 >= MIN_ESC_VAL)
//    PWMMotorTimes.M1 = PWMMotorTimes.M1;
//  else
//    PWMMotorTimes.M1 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M2 > MAX_ESC_VAL)
//    PWMMotorTimes.M2 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M2 >= MIN_ESC_VAL)
//    PWMMotorTimes.M2 = PWMMotorTimes.M2;
//  else
//    PWMMotorTimes.M2 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M3 > MAX_ESC_VAL)
//    PWMMotorTimes.M3 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M3 >= MIN_ESC_VAL)
//    PWMMotorTimes.M3 = PWMMotorTimes.M3;
//  else
//    PWMMotorTimes.M3 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M4 > MAX_ESC_VAL)
//    PWMMotorTimes.M4 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M4 >= MIN_ESC_VAL)
//    PWMMotorTimes.M4 = PWMMotorTimes.M4;
//  else
//    PWMMotorTimes.M4 = MIN_ESC_VAL;
// }

/*
 * @brief  Manual mode allocation - allocates raw RC input.
 * @param  None.
 * @retval None.
 */
//void ManualModeAllocation(void) {
//#ifdef TODO Manual mode allocation should perhaps also be mapped to thrust and rotational moments, but controlled
//				directly from RC rather than through control algorithm (is it really necessary?)
//	PWMMotorTimes.M1 = 0.9
//	* (PWMInputTimes.Throttle - 2 * (PWMInputTimes.Aileron - GetRCmid())
//			- 2 * (PWMInputTimes.Elevator - GetRCmid())
//			- 2 * (PWMInputTimes.Rudder - GetRCmid()));
//	PWMMotorTimes.M2 = 0.9
//	* (PWMInputTimes.Throttle + 2 * (PWMInputTimes.Aileron - GetRCmid())
//			- 2 * (PWMInputTimes.Elevator - GetRCmid())
//			+ 2 * (PWMInputTimes.Rudder - GetRCmid()));
//	PWMMotorTimes.M3 = 0.9
//	* (PWMInputTimes.Throttle + 2 * (PWMInputTimes.Aileron - GetRCmid())
//			+ 2 * (PWMInputTimes.Elevator - GetRCmid())
//			- 2 * (PWMInputTimes.Rudder - GetRCmid()));
//	PWMMotorTimes.M4 = 0.9
//	* (PWMInputTimes.Throttle - 2 * (PWMInputTimes.Aileron - GetRCmid())
//			+ 2 * (PWMInputTimes.Elevator - GetRCmid())
//			+ 2 * (PWMInputTimes.Rudder - GetRCmid()));
//
//  if (PWMMotorTimes.M1 > MAX_ESC_VAL)
//    PWMMotorTimes.M1 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M1 >= MIN_ESC_VAL)
//    PWMMotorTimes.M1 = PWMMotorTimes.M1;
//  else
//    PWMMotorTimes.M1 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M2 > MAX_ESC_VAL)
//    PWMMotorTimes.M2 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M2 >= MIN_ESC_VAL)
//    PWMMotorTimes.M2 = PWMMotorTimes.M2;
//  else
//    PWMMotorTimes.M2 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M3 > MAX_ESC_VAL)
//    PWMMotorTimes.M3 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M3 >= MIN_ESC_VAL)
//    PWMMotorTimes.M3 = PWMMotorTimes.M3;
//  else
//    PWMMotorTimes.M3 = MIN_ESC_VAL;
//
//  if (PWMMotorTimes.M4 > MAX_ESC_VAL)
//    PWMMotorTimes.M4 = MAX_ESC_VAL;
//  else if (PWMMotorTimes.M4 >= MIN_ESC_VAL)
//    PWMMotorTimes.M4 = PWMMotorTimes.M4;
//  else
//    PWMMotorTimes.M4 = MIN_ESC_VAL;
//#endif
//}

/*
 * @brief  Creates a task to sample print motor signal values over USB.
 * @param  sampleTime : Sets how often a sample should be printed.
 * @param  sampleDuration : Sets for how long sampling should be performed.
 * @retval MOTORCTRL_OK if thread started, else MOTORCTRL_ERROR.
 */
MotorControlErrorStatus StartMotorControlSamplingTask(const uint16_t sampleTime, const uint32_t sampleDuration) {
	if(sampleTime < MOTOR_CONTROL_PRINT_MINIMUM_SAMPLING_TIME)
		motorControlPrintSampleTime = MOTOR_CONTROL_PRINT_MINIMUM_SAMPLING_TIME;
	else
		motorControlPrintSampleTime = sampleTime;

	motorControlPrintSampleDuration = sampleDuration;

	/* Motor control signal value print sampling handler thread creation
	 * Task function pointer: MotorControlPrintSamplingTask
	 * Task name: MOTORCTRL_PRINT_SAMPL
	 * Stack depth: configMINIMAL_STACK_SIZE
	 * Parameter: NULL
	 * Priority: MOTOR_CONTROL_PRINT_SAMPLING_TASK_PRIO (0 to configMAX_PRIORITIES-1 possible)
	 * Handle: MotorControlPrintSamplingTaskHandle
	 * */
	if (pdPASS != xTaskCreate((pdTASK_CODE )MotorControlPrintSamplingTask, (signed portCHAR*)"MOTORCTRL_PRINT_SAMPL",
			configMINIMAL_STACK_SIZE, NULL, MOTOR_CONTROL_PRINT_SAMPLING_TASK_PRIO, &MotorControlPrintSamplingTaskHandle)) {
		ErrorHandler();
		return MOTORCTRL_ERROR;
	}

	return MOTORCTRL_OK;
}

/*
 * @brief  Stops motor control print sampling by deleting the task.
 * @param  None.
 * @retval MOTORCTRL_OK if task deleted, MOTORCTRL_ERROR if not.
 */
MotorControlErrorStatus StopMotorControlSamplingTask(void) {
	if(MotorControlPrintSamplingTaskHandle != NULL) {
		vTaskDelete(MotorControlPrintSamplingTaskHandle);
		MotorControlPrintSamplingTaskHandle = NULL;
		return MOTORCTRL_OK;
	}
	return MOTORCTRL_ERROR;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Task code handles motor control signal print sampling
 * @param  argument : Unused parameter
 * @retval None
 */
static void MotorControlPrintSamplingTask(void const *argument) {
	(void) argument;

	portTickType xLastWakeTime;
	portTickType xSampleStartTime;

	/* Initialise the xLastWakeTime variable with the current time */
	xLastWakeTime = xTaskGetTickCount();
	xSampleStartTime = xLastWakeTime;

	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, motorControlPrintSampleTime);

		PrintMotorControlValues();

		/* If sampling duration exceeded, delete task to stop sampling */
		if (xTaskGetTickCount() >= xSampleStartTime + motorControlPrintSampleDuration * configTICK_RATE_HZ)
			StopMotorControlSamplingTask();
	}
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 1
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor1(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR1_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor1 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 2
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor2(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR2_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor2 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 3
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor3(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR3_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor3 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 4
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor4(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR4_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor4 = ctrlVal;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
