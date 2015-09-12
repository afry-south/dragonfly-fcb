/*****************************************************************************
 * @file    motor_control.c
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

#include "fcb_error.h"
#include "receiver.h"
#include "common.h"
#include "dragonfly_fcb.pb.h"
#include "usb_com_cli.h"
#include "pb_encode.h"

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

typedef enum {
	MOTOR_NEG_ROTATION = -1,
	MOTOR_POS_ROTATION = 1
} MotorMomentSign;

typedef struct {
	MotorMomentSign rollDir;		// Depends on the motor location relative to aircraft CoG
	MotorMomentSign pitchDir;	// Depends on the motor location relative to aircraft CoG
	MotorMomentSign yawDir;		// Depends on the motor rotational direction (CCW or CW)
} MotorProperties_TypeDef;

/* Motor control value struct declaration*/
static MotorControlValues_TypeDef MotorControlValues;

static MotorProperties_TypeDef Motor1Properties;
static MotorProperties_TypeDef Motor2Properties;
static MotorProperties_TypeDef Motor3Properties;
static MotorProperties_TypeDef Motor4Properties;

/* Timer time base handler */
static TIM_HandleTypeDef MotorControlTimHandle;

/* Task handle for printing of sensor values task */
xTaskHandle MotorControlPrintSamplingTaskHandle = NULL;
static volatile uint16_t motorControlPrintSampleTime;
static volatile uint16_t motorControlPrintSampleDuration;
static SerializationType_TypeDef motorValuesPrintSerializationType;

/* Private function prototypes -----------------------------------------------*/
static void MotorControlPrintSamplingTask(void const *argument);
static void SetMotor1(const uint16_t ctrlVal);
static void SetMotor2(const uint16_t ctrlVal);
static void SetMotor3(const uint16_t ctrlVal);
static void SetMotor4(const uint16_t ctrlVal);
static void SaturateMotorSignalValues();

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes and configures the timer used to produce PWM output to control the ESC:s with.
 * @param  None.
 * @retval None.
 */
void MotorControlConfig(void) {
	/* Configure each of the motors' properties */
	Motor1Properties.rollDir = MOTOR_NEG_ROTATION;
	Motor1Properties.pitchDir = MOTOR_POS_ROTATION;
	Motor1Properties.yawDir = MOTOR_POS_ROTATION;

	Motor2Properties.rollDir = MOTOR_POS_ROTATION;
	Motor2Properties.pitchDir = MOTOR_POS_ROTATION;
	Motor2Properties.yawDir = MOTOR_NEG_ROTATION;

	Motor3Properties.rollDir = MOTOR_POS_ROTATION;
	Motor3Properties.pitchDir = MOTOR_NEG_ROTATION;
	Motor3Properties.yawDir = MOTOR_POS_ROTATION;

	Motor4Properties.rollDir = MOTOR_NEG_ROTATION;
	Motor4Properties.pitchDir = MOTOR_NEG_ROTATION;
	Motor4Properties.yawDir = MOTOR_NEG_ROTATION;

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
 * @brief  Sets the serialization type of printed motor signal values
 * @param  serializationType : Data serialization type enum
 * @retval None.
 */
void SetMotorPrintSamplingSerialization(const SerializationType_TypeDef serializationType) {
	motorValuesPrintSerializationType = serializationType;
}

/*
 * @brief  Prints motor control signal values
 * @param  None.
 * @retval None.
 */
void PrintMotorControlValues(const SerializationType_TypeDef serializationType) {
	static char motorCtrlString[MOTOR_CONTROL_PRINT_MAX_STRING_SIZE];

	if(serializationType == NO_SERIALIZATION) {
		snprintf((char*) motorCtrlString, MOTOR_CONTROL_PRINT_MAX_STRING_SIZE,
				"Motor control (uint16):\nM1: %u\nM2: %u\nM3: %u\nM4: %u\n\r\n", MotorControlValues.Motor1,
				MotorControlValues.Motor2, MotorControlValues.Motor3, MotorControlValues.Motor4);

		USBComSendString(motorCtrlString);
	}
	else if(serializationType == PROTOBUFFER_SERIALIZATION) {
		bool protoStatus;
		uint8_t serializedMotorData[MotorSignalValuesProto_size];
		MotorSignalValuesProto motorSignalValuesProto;
		uint32_t strLen;

		motorSignalValuesProto.has_M1 = true;
		motorSignalValuesProto.has_M2 = true;
		motorSignalValuesProto.has_M3 = true;
		motorSignalValuesProto.has_M4 = true;
		motorSignalValuesProto.M1 = MotorControlValues.Motor1;
		motorSignalValuesProto.M2 = MotorControlValues.Motor2;
		motorSignalValuesProto.M3 = MotorControlValues.Motor3;
		motorSignalValuesProto.M4 = MotorControlValues.Motor4;

		/* Create a stream that will write to our buffer and encode the data with protocol buffer */
		pb_ostream_t protoStream = pb_ostream_from_buffer(serializedMotorData, MotorSignalValuesProto_size);
		protoStatus = pb_encode(&protoStream, MotorSignalValuesProto_fields, &motorSignalValuesProto);

		/* Insert header to the sample string, then copy the data after that */
		snprintf(motorCtrlString, MOTOR_CONTROL_PRINT_MAX_STRING_SIZE, "%c %c ", MOTOR_VALUES_MSG_ENUM, protoStream.bytes_written);
		strLen = strlen(motorCtrlString);
		if(strLen + protoStream.bytes_written + strlen("\r\n") < MOTOR_CONTROL_PRINT_MAX_STRING_SIZE) {
			memcpy(&motorCtrlString[strLen], serializedMotorData, protoStream.bytes_written);
			memcpy(&motorCtrlString[strLen+protoStream.bytes_written], "\r\n", strlen("\r\n"));
		}

		if(protoStatus)
			USBComSendData((uint8_t*)motorCtrlString, strLen+protoStream.bytes_written+strlen("\r\n"));
		else
			ErrorHandler();
	}
}

/*
 * @brief  Allocates RC receiver input directly to motor output
 * @param  None.
 * @retval None.
 */
void MotorAllocationRaw(void) {
	int32_t u1, u2, u3, u4, m1, m2, m3, m4;

	/* Calculate raw control signals for throttle, roll, pitch, yaw */
	u1 = (GetThrottleReceiverChannel()-INT16_MIN); // Re-scale to uint16
	u2 = -u1*GetAileronReceiverChannel()/INT16_MAX;
	u3 = -u1*GetElevatorReceiverChannel()/INT16_MAX;
	u4 = -u1*GetRudderReceiverChannel()/INT16_MAX;

	/* Map raw control signals to motor output */
	m1 = u1 + Motor1Properties.rollDir*u2 + Motor1Properties.pitchDir*u3 + Motor1Properties.yawDir*u4;
	m2 = u1 + Motor2Properties.rollDir*u2 + Motor2Properties.pitchDir*u3 + Motor2Properties.yawDir*u4;
	m3 = u1 + Motor3Properties.rollDir*u2 + Motor3Properties.pitchDir*u3 + Motor3Properties.yawDir*u4;
	m4 = u1 + Motor4Properties.rollDir*u2 + Motor4Properties.pitchDir*u3 + Motor4Properties.yawDir*u4;

	/* Saturate motor signal values [0, UINT16_MAX] */
	SaturateMotorSignalValues(&m1, &m2, &m3, &m4);

	if (IsReceiverActive()) {
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
 * 		   thrust force [N] and roll/pitch/yaw moments [Nm] to motor output signal values of each of the four motors.
 * @param  u1 : thrust force [N]
 * @param  u2 : roll moment [Nm]
 * @param  u3 : pitch moment [Nm]
 * @param  u4 : yaw moment [Nm]
 * @retval None.
 */
void MotorAllocationPhysical(const float u1, const float u2, const float u3, const float u4) {
	int32_t m1, m2, m3, m4;

	/* Calculate physical motor control allocation. Remember that Z points down, so u1 will be negative. */
	m1 = (int32_t) (-THRUST_ALLOC_COEFF*u1 - THRUST_ALLOC_OFFSET - ROLLPITCH_ALLOC_COEFF*u2 + ROLLPITCH_ALLOC_COEFF*u3 + YAW_ALLOC_COEFF*u4);
	m2 = (int32_t) (-THRUST_ALLOC_COEFF*u1 - THRUST_ALLOC_OFFSET + ROLLPITCH_ALLOC_COEFF*u2 + ROLLPITCH_ALLOC_COEFF*u3 - YAW_ALLOC_COEFF*u4);
	m3 = (int32_t) (-THRUST_ALLOC_COEFF*u1 - THRUST_ALLOC_OFFSET + ROLLPITCH_ALLOC_COEFF*u2 - ROLLPITCH_ALLOC_COEFF*u3 + YAW_ALLOC_COEFF*u4);
	m4 = (int32_t) (-THRUST_ALLOC_COEFF*u1 - THRUST_ALLOC_OFFSET - ROLLPITCH_ALLOC_COEFF*u2 - ROLLPITCH_ALLOC_COEFF*u3 - YAW_ALLOC_COEFF*u4);

	/* Saturate motor signal values [0, UINT16_MAX] */
	SaturateMotorSignalValues(&m1, &m2, &m3, &m4);

	if (IsReceiverActive()) {
		/* Set the motor signal values */
		SetMotors(m1, m2, m3, m4);
	} else {
		ShutdownMotors();
	}
}

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

		PrintMotorControlValues(motorValuesPrintSerializationType);

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
static void SetMotor1(const uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR1_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor1 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 2
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor2(const uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR2_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor2 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 3
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor3(const uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR3_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor3 = ctrlVal;
}

/*
 * @brief  Sets the motor control PWM (sent to ESC) for motor 4
 * @param  ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval None.
 */
static void SetMotor4(const uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&MotorControlTimHandle, MOTOR4_CHANNEL, (uint16_t)ccrVal);
	MotorControlValues.Motor4 = ctrlVal;
}

static void SaturateMotorSignalValues(int32_t* m1, int32_t* m2, int32_t* m3, int32_t* m4)
{
	/* Check unsigned 16-bit overflow */
	if (!IS_NOT_GREATER_UINT16_MAX(*m1))
		*m1 = UINT16_MAX;
	if (!IS_NOT_GREATER_UINT16_MAX(*m2))
		*m2 = UINT16_MAX;
	if (!IS_NOT_GREATER_UINT16_MAX(*m3))
		*m3 = UINT16_MAX;
	if (!IS_NOT_GREATER_UINT16_MAX(*m4))
		*m4 = UINT16_MAX;

	/* Check unsigned 16-bit underflow (less than zero) */
	if (!IS_POS(*m1))
		*m1 = 0;
	if (!IS_POS(*m2))
		*m2 = 0;
	if (!IS_POS(*m3))
		*m3 = 0;
	if (!IS_POS(*m4))
		*m4 = 0;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
