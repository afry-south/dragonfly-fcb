/*****************************************************************************
 * @file    pid_control.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-31
 * @brief   File contains PID control algorithm implementation
 ******************************************************************************/

// TODO NOTE: Alot of code below is from the old project. It may be used as reference for further development.
// Eventually, it should be (re)moved and/or replaced or reimplemented. See real-time system implementation of PID.

/* Includes ------------------------------------------------------------------*/
#include "pid_control.h"

#include "flight_control.h"
#include "motor_control.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CONTROL_PERIOD	(float) FLIGHT_CONTROL_TASK_PERIOD/1000.0

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static PIDController_TypeDef AltCtrl;
static PIDController_TypeDef RollCtrl;
static PIDController_TypeDef PitchCtrl;
static PIDController_TypeDef YawCtrl;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief	Initializes the PID controllers, i.e. sets controller parameters.
 * @param	None.
 * @retval	None.
 */
void InitPIDControllers(void) {
	/* Initialize Altitude Controller */
	AltCtrl.K = K_VZ;
	AltCtrl.Ti = TI_VZ;
	AltCtrl.Td = TD_VZ;
	AltCtrl.Beta = BETA_VZ;
	AltCtrl.Gamma = GAMMA_VZ;
	AltCtrl.N = N_VZ;
	AltCtrl.P = 0.0;
	AltCtrl.I = 0.0;
	AltCtrl.D = 0.0;

	/* Initialize Roll Controller */
	RollCtrl.K = K_RP;
	RollCtrl.Ti = TI_RP;
	RollCtrl.Td = TD_RP;
	RollCtrl.Beta = BETA_RP;
	RollCtrl.Gamma = GAMMA_RP;
	RollCtrl.N = N_RP;
	RollCtrl.P = 0.0;
	RollCtrl.I = 0.0;
	RollCtrl.D = 0.0;

	/* Initialize Pitch Controller */
	PitchCtrl.K = K_RP;
	PitchCtrl.Ti = TI_RP;
	PitchCtrl.Td = TD_RP;
	PitchCtrl.Beta = BETA_RP;
	PitchCtrl.Gamma = GAMMA_RP;
	PitchCtrl.N = N_RP;
	PitchCtrl.P = 0.0;
	PitchCtrl.I = 0.0;
	PitchCtrl.D = 0.0;

	/* Initialize Yaw Controller */
	YawCtrl.K = K_YR;
	YawCtrl.Ti = TI_YR;
	YawCtrl.Td = TD_YR;
	YawCtrl.Beta = BETA_YR;
	YawCtrl.Gamma = GAMMA_YR;
	YawCtrl.N = N_YR;
	YawCtrl.P = 0.0;
	YawCtrl.I = 0.0;
	YawCtrl.D = 0.0;
}

/*
 * @brief	Controls the thrust force to achieve a desired vertical velocity
 * @param	None.
 * @retval	None.
 */
float AltitudeControl(void) {
	static float PreZVelocityRef = 0.0;
	static float PreZVelocity = 0.0;

	float AltitudeVelocityControlSignal;
	// float ZVelocity = GetZVelocity(); // TODO Get estimate from Kalman filter
	float ZVelocity = 0.0;
	float ZVelocityRef = GetZVelocityReferenceSignal();

	/* Calculate Proportional control part */
	AltCtrl.P = AltCtrl.K*(AltCtrl.Beta*ZVelocityRef - ZVelocity);

	/* Calculate Integral control part */
	if(AltCtrl.Ti > 0.001)
		AltCtrl.I = AltCtrl.I + AltCtrl.K*CONTROL_PERIOD/AltCtrl.Ti*(ZVelocityRef - ZVelocity);

	/* Calculate Derivative control part */
	AltCtrl.D = AltCtrl.Td / (AltCtrl.Td + AltCtrl.N * CONTROL_PERIOD)*AltCtrl.D
			+ AltCtrl.K * AltCtrl.Td * AltCtrl.N / (AltCtrl.Td + AltCtrl.N * CONTROL_PERIOD )
			* (AltCtrl.Gamma * (ZVelocityRef - PreZVelocityRef) - (ZVelocity - PreZVelocity));

	/* Calculate sum of P-I-D parts to obtain the control signal. Offset with gravitational acceleration and multiply
	 * with mass to obtain thrust force. Remember that z points downward so control signal should be negative */
	AltitudeVelocityControlSignal = (AltCtrl.P + AltCtrl.I + AltCtrl.D - G_ACC ) * MASS;

	/* Saturate controller output */
	if (AltitudeVelocityControlSignal > 0.0)
		AltitudeVelocityControlSignal = 0.0;
	else if (AltitudeVelocityControlSignal < -MAX_THRUST)
		AltitudeVelocityControlSignal = -MAX_THRUST;

	PreZVelocityRef = ZVelocityRef;
	PreZVelocity = ZVelocity;

	return AltitudeVelocityControlSignal;
}

/*
 * @brief	Controls the roll moment to achieve a desired roll angle
 * @param	None.
 * @retval	None.
 */
float RollControl(void) {
	static float PreRollAngleRef = 0.0;
	static float PreRollAngle = 0.0;

	float RollAngleControlSignal;
	// float RollAngle = GetRollAngle(); // TODO Get estimate from Kalman filter
	float RollAngle = 0.0;
	float RollAngleRef = GetRollAngleReferenceSignal();

	/* Calculate Proportional control part */
	RollCtrl.P = RollCtrl.K*(RollCtrl.Beta*RollAngleRef - RollAngle);

	/* Calculate Integral control part */
	if(RollCtrl.Ti > 0.001)
		RollCtrl.I = RollCtrl.I + RollCtrl.K*CONTROL_PERIOD/RollCtrl.Ti*(RollAngleRef - RollAngle);

	/* Calculate Derivative control part */
	RollCtrl.D = RollCtrl.Td / (RollCtrl.Td + RollCtrl.N * CONTROL_PERIOD)*RollCtrl.D
			+ RollCtrl.K * RollCtrl.Td * RollCtrl.N / (RollCtrl.Td + RollCtrl.N * CONTROL_PERIOD )
			* (RollCtrl.Gamma * (RollAngleRef - PreRollAngleRef) - (RollAngle - PreRollAngle));

	/* Calculate sum of P-I-D parts to obtain the control signal. Multiply with IXX moment of inertia to obtain roll moment. */
	RollAngleControlSignal = (RollCtrl.P + RollCtrl.I + RollCtrl.D) * IXX;

	/* Saturate controller output */
	if (RollAngleControlSignal < -MAX_ROLLPITCH_MOM)
		RollAngleControlSignal = -MAX_ROLLPITCH_MOM;
	else if (RollAngleControlSignal > MAX_ROLLPITCH_MOM)
		RollAngleControlSignal = MAX_ROLLPITCH_MOM;

	PreRollAngleRef = RollAngleRef;
	PreRollAngle = RollAngle;

	return RollAngleControlSignal;
}

/*
 * @brief	Controls the pitch moment to achieve a desired pitch angle
 * @param	None.
 * @retval	None.
 */
float PitchControl(void) {
	static float PrePitchAngleRef = 0.0;
	static float PrePitchAngle = 0.0;

	float PitchAngleControlSignal;
	// float PitchAngle = GetPitchAngle(); // TODO Get estimate from Kalman filter
	float PitchAngle = 0.0;
	float PitchAngleRef = GetPitchAngleReferenceSignal();

	/* Calculate Proportional control part */
	PitchCtrl.P = PitchCtrl.K*(PitchCtrl.Beta*PitchAngleRef - PitchAngle);

	/* Calculate Integral control part */
	if(PitchCtrl.Ti > 0.001)
		PitchCtrl.I = PitchCtrl.I + PitchCtrl.K*CONTROL_PERIOD/PitchCtrl.Ti*(PitchAngleRef - PitchAngle);

	/* Calculate Derivative control part */
	PitchCtrl.D = PitchCtrl.Td / (PitchCtrl.Td + PitchCtrl.N * CONTROL_PERIOD)*PitchCtrl.D
			+ PitchCtrl.K * PitchCtrl.Td * PitchCtrl.N / (PitchCtrl.Td + PitchCtrl.N * CONTROL_PERIOD )
			* (PitchCtrl.Gamma * (PitchAngleRef - PrePitchAngleRef) - (PitchAngle - PrePitchAngle));

	/* Calculate sum of P-I-D parts to obtain the control signal. Multiply with IYY moment of inertia to obtain pitch moment. */
	PitchAngleControlSignal = (PitchCtrl.P + PitchCtrl.I + PitchCtrl.D) * IYY;

	/* Saturate controller output */
	if (PitchAngleControlSignal < -MAX_ROLLPITCH_MOM)
		PitchAngleControlSignal = -MAX_ROLLPITCH_MOM;
	else if (PitchAngleControlSignal > MAX_ROLLPITCH_MOM)
		PitchAngleControlSignal = MAX_ROLLPITCH_MOM;

	PrePitchAngleRef = PitchAngleRef;
	PrePitchAngle = PitchAngle;

	return PitchAngleControlSignal;
}

/*
 * @brief	Controls the yaw moment to achieve desired yaw rate
 * @param	None.
 * @retval	None.
 */
float YawControl(void) {
	static float PreYawRateRef = 0.0;
	static float PreYawRate = 0.0;

	float YawRateControlSignal;
	// float YawRate = GetYawRate(); // TODO Get estimate from Kalman filter
	float YawRate = 0.0;
	float YawRateRef = GetYawAngularRateReferenceSignal();

	/* Calculate Proportional control part */
	YawCtrl.P = YawCtrl.K*(YawCtrl.Beta*YawRateRef - YawRate);

	/* Calculate Integral control part */
	if(YawCtrl.Ti > 0.001)
		YawCtrl.I = YawCtrl.I + YawCtrl.K*CONTROL_PERIOD/YawCtrl.Ti*(YawRateRef - YawRate);

	/* Calculate Derivative control part */
	YawCtrl.D = YawCtrl.Td / (YawCtrl.Td + YawCtrl.N * CONTROL_PERIOD)*YawCtrl.D
			+ YawCtrl.K * YawCtrl.Td * YawCtrl.N / (YawCtrl.Td + YawCtrl.N * CONTROL_PERIOD )
			* (YawCtrl.Gamma * (YawRateRef - PreYawRateRef) - (YawRate - PreYawRate));

	/* Calculate sum of P-I-D parts to obtain the control signal. Multiply with IYY moment of inertia to obtain pitch moment. */
	YawRateControlSignal = (YawCtrl.P + YawCtrl.I + YawCtrl.D) * IZZ;

	/* Saturate controller output */
	if (YawRateControlSignal < -MAX_YAW_MOM)
		YawRateControlSignal = -MAX_YAW_MOM;
	else if (YawRateControlSignal > MAX_YAW_MOM)
		YawRateControlSignal = MAX_YAW_MOM;

	PreYawRateRef = YawRateRef;
	PreYawRate = YawRate;

	return YawRateControlSignal;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
