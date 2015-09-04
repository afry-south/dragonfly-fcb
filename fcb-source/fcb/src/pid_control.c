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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PIDController_TypeDef AltCtrl;
PIDController_TypeDef RollCtrl;
PIDController_TypeDef PitchCtrl;
PIDController_TypeDef YawCtrl;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief	Initializes the PID controllers, i.e. sets controller parameters.
 * @param	None.
 * @retval	None.
 */
void InitPIDControllers() {
#ifdef TODO
	/* Initialize Altitude Controller */
	AltCtrl.K = K_VZ;
	AltCtrl.Ti = TI_VZ;
	AltCtrl.Td = TD_VZ;
	AltCtrl.B = BETA_VZ;
	AltCtrl.N = N_VZ;
	AltCtrl.P = 0.0;
	AltCtrl.I = 0.0;
	AltCtrl.D = 0.0;
	AltCtrl.PreState = 0.0;

	/* Initialize Roll Controller */
	RollCtrl.K = K_RP;
	RollCtrl.Ti = TI_RP;
	RollCtrl.Td = TD_RP;
	RollCtrl.B = BETA_RP;
	RollCtrl.N = N_RP;
	RollCtrl.P = 0.0;
	RollCtrl.I = 0.0;
	RollCtrl.D = 0.0;
	RollCtrl.PreState = 0.0; // TODO Set initial estimate

	/* Initialize Pitch Controller */
	PitchCtrl.K = K_RP;
	PitchCtrl.Ti = TI_RP;
	PitchCtrl.Td = TD_RP;
	PitchCtrl.B = BETA_RP;
	PitchCtrl.N = N_RP;
	PitchCtrl.P = 0.0;
	PitchCtrl.I = 0.0;
	PitchCtrl.D = 0.0;
	PitchCtrl.PreState = 0.0; // TODO Set initial estimate

	/* Initialize Yaw Controller */
	YawCtrl.K = K_YR;
	YawCtrl.Ti = TI_YR;
	YawCtrl.Td = TD_YR;
	YawCtrl.B = BETA_YR;
	YawCtrl.N = N_YR;
	YawCtrl.P = 0.0;
	YawCtrl.I = 0.0;
	YawCtrl.D = 0.0;
	YawCtrl.PreState = 0.0; // TODO Set initial estimate
#endif
}

/*
 * @brief	Controls the thrust force to achieve a desired vertical velocity
 * @param	None.
 * @retval	None.
 */
void AltitudeControl(void) {
#ifdef TODO
	float ZVelocity = GetZVelocity();

	AltCtrl.P = AltCtrl.K * (AltCtrl.B * RefSignals.ZVelocity - ZVelocity);

	// Backward difference, derivative part with zero set-point weighting
	AltCtrl.D = AltCtrl.Td / (AltCtrl.Td + AltCtrl.N * CONTROL_SAMPLE_PERTIOD)
			* AltCtrl.D
			- AltCtrl.K * AltCtrl.Td * AltCtrl.N
					/ (AltCtrl.Td + AltCtrl.N * CONTROL_SAMPLE_PERTIOD)
					* (ZVelocity - AltCtrl.PreState);

	CtrlSignals.Thrust = (AltCtrl.P + AltCtrl.I + AltCtrl.D + G_ACC ) * MASS;

	// Saturate controller output
	if (CtrlSignals.Thrust < 0)
		CtrlSignals.Thrust = 0;
	else if (CtrlSignals.Thrust > MAX_THRUST)
		CtrlSignals.Thrust = MAX_THRUST;

	// Forward difference, so updated after control
	if (AltCtrl.Ti != 0.0)
		AltCtrl.I = AltCtrl.I
				+ AltCtrl.K * CONTROL_SAMPLE_PERTIOD / AltCtrl.Ti
						* (RefSignals.ZVelocity - ZVelocity);

	AltCtrl.PreState = ZVelocity;
#endif
}

/*
 * @brief	Controls the roll moment to achieve a desired roll angle
 * @param	None.
 * @retval	None.
 */
void RollControl(void) {
#ifdef TODO
	float RollAngle = GetRoll();

	RollCtrl.P = RollCtrl.K * (RollCtrl.B * RefSignals.RollAngle - RollAngle);

	// Backward difference, derivative part with zero set-point weighting
	RollCtrl.D = RollCtrl.Td
			/ (RollCtrl.Td + RollCtrl.N * CONTROL_SAMPLE_PERTIOD) * RollCtrl.D
			- RollCtrl.K * RollCtrl.Td * RollCtrl.N
					/ (RollCtrl.Td + RollCtrl.N * CONTROL_SAMPLE_PERTIOD)
					* (RollAngle - RollCtrl.PreState);

	CtrlSignals.Roll = (RollCtrl.P + RollCtrl.I + RollCtrl.D) * IXX;

	// Saturate controller output
	if (CtrlSignals.Roll < -MAX_ROLLPITCH_MOM)
		CtrlSignals.Roll = -MAX_ROLLPITCH_MOM;
	else if (CtrlSignals.Roll > MAX_ROLLPITCH_MOM)
		CtrlSignals.Roll = MAX_ROLLPITCH_MOM;

	// Forward difference, so updated after control
	if (RollCtrl.Ti != 0.0)
		RollCtrl.I = RollCtrl.I
				+ RollCtrl.K * CONTROL_SAMPLE_PERTIOD / RollCtrl.Ti
						* (RefSignals.RollAngle - RollAngle);

	RollCtrl.PreState = RollAngle;
#endif
}

/*
 * @brief	Controls the pitch moment to achieve a desired pitch angle
 * @param	None.
 * @retval	None.
 */
void PitchControl(void) {
#ifdef TODO
	float PitchAngle = GetPitch();

	PitchCtrl.P = PitchCtrl.K
			* (PitchCtrl.B * RefSignals.PitchAngle - PitchAngle);

	// Backward difference, derivative part with zero set-point weighting
	PitchCtrl.D = PitchCtrl.Td
			/ (PitchCtrl.Td + PitchCtrl.N * CONTROL_SAMPLE_PERTIOD)
			* PitchCtrl.D
			- PitchCtrl.K * PitchCtrl.Td * PitchCtrl.N
					/ (PitchCtrl.Td + PitchCtrl.N * CONTROL_SAMPLE_PERTIOD)
					* (PitchAngle - PitchCtrl.PreState);

	CtrlSignals.Pitch = (PitchCtrl.P + PitchCtrl.I + PitchCtrl.D) * IYY;

	// Saturate controller output
	if (CtrlSignals.Pitch < -MAX_ROLLPITCH_MOM)
		CtrlSignals.Pitch = -MAX_ROLLPITCH_MOM;
	else if (CtrlSignals.Pitch > MAX_ROLLPITCH_MOM)
		CtrlSignals.Pitch = MAX_ROLLPITCH_MOM;

	// Forward difference, so updated after control
	if (PitchCtrl.Ti != 0.0)
		PitchCtrl.I = PitchCtrl.I
				+ PitchCtrl.K * CONTROL_SAMPLE_PERTIOD / PitchCtrl.Ti
						* (RefSignals.PitchAngle - PitchAngle);

	PitchCtrl.PreState = PitchAngle;
#endif
}

/*
 * @brief	Controls the yaw moment to achieve desired yaw rate
 * @param	None.
 * @retval	None.
 */
void YawControl(void) {
#ifdef TODO
	float YawRate = GetYawRate();

	YawCtrl.P = YawCtrl.K * (YawCtrl.B * RefSignals.YawRate - YawRate);

	// Backward difference
	YawCtrl.D = YawCtrl.Td / (YawCtrl.Td + YawCtrl.N * CONTROL_SAMPLE_PERTIOD)
			* YawCtrl.D
			- YawCtrl.K * YawCtrl.Td * YawCtrl.N
					/ (YawCtrl.Td + YawCtrl.N * CONTROL_SAMPLE_PERTIOD)
					* (YawRate - YawCtrl.PreState);

	CtrlSignals.Yaw = (YawCtrl.P + YawCtrl.I + YawCtrl.D) * IZZ;

	// Saturate controller output
	if (CtrlSignals.Yaw < -MAX_YAW_RATE)
		CtrlSignals.Yaw = -MAX_YAW_RATE;
	else if (CtrlSignals.Yaw > MAX_YAW_RATE)
		CtrlSignals.Yaw = MAX_YAW_RATE;

	// Forward difference, so updated after control
	if (YawCtrl.Ti != 0.0)
		YawCtrl.I = YawCtrl.I
				+ YawCtrl.K * CONTROL_SAMPLE_PERTIOD / YawCtrl.Ti
						* (RefSignals.YawRate - YawRate);

	YawCtrl.PreState = YawRate;
#endif
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
