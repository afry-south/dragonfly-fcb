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

/* Includes */
#include "motor_control.h"
#include "fcb_error.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer time base handler */
TIM_HandleTypeDef TimHandle;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/*
 * @brief	Initializes and configures the timer used to produce PWM output
 *              to control the ESC:s with.
 * @param	None.
 * @retval	None.
 */
void MotorControlConfig(void) {
	/*##-1- Configure the TIM peripheral #######################################*/

	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef ocConfig;

	/* Initialize Motor TIM peripheral timebase */
	TimHandle.Instance = TIM_MOTOR;
	TimHandle.Init.Prescaler = SystemCoreClock / MOTOR_OUTPUT_COUNTER_CLOCK - 1;
	TimHandle.Init.Period = MOTOR_OUTPUT_PERIOD;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
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
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &ocConfig, MOTOR1_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 2 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &ocConfig, MOTOR2_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 3 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &ocConfig, MOTOR3_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/* Set the pulse value for Motor 4 */
	ocConfig.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &ocConfig, MOTOR4_CHANNEL) != HAL_OK) {
		/* Configuration Error */
		ErrorHandler();
	}

	/*##-3- Start PWM signals generation #######################################*/
	/* Start Motor 1 channel */
	if (HAL_TIM_PWM_Start(&TimHandle, MOTOR1_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 2 channel */
	if (HAL_TIM_PWM_Start(&TimHandle, MOTOR2_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 3 channel */
	if (HAL_TIM_PWM_Start(&TimHandle, MOTOR3_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
	/* Start Motor 4 channel */
	if (HAL_TIM_PWM_Start(&TimHandle, MOTOR4_CHANNEL) != HAL_OK) {
		/* PWM Generation Error */
		ErrorHandler();
	}
}

/*
 * @brief       Sets the motor control PWM (sent to ESC) for motor 1
 * @param       ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval      None.
 */
void SetMotor1(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&TimHandle, MOTOR1_CHANNEL, (uint16_t )ccrVal);
}

/*
 * @brief       Sets the motor control PWM (sent to ESC) for motor 2
 * @param       ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval      None.
 */
void SetMotor2(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&TimHandle, MOTOR2_CHANNEL, ccrVal);
}

/*
 * @brief       Sets the motor control PWM (sent to ESC) for motor 3
 * @param       ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval      None.
 */
void SetMotor3(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&TimHandle, MOTOR3_CHANNEL, ccrVal);
}

/*
 * @brief       Sets the motor control PWM (sent to ESC) for motor 4
 * @param       ctrlVal: value [0,65535] indicating amount of motor thrust
 * @retval      None.
 */
void SetMotor4(uint16_t ctrlVal) {
	uint32_t ccrVal = ESC_MIN_OUTPUT + ctrlVal * (ESC_MAX_OUTPUT - ESC_MIN_OUTPUT) / UINT16_MAX;
	__HAL_TIM_SetCompare(&TimHandle, MOTOR4_CHANNEL, ccrVal);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
