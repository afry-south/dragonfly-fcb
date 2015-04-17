/******************************************************************************
 * @file    fcb/control.c
 * @author  ÅF Dragonfly:
 * Daniel Stenberg, Embedded Systems
 * @version v. 0.0.1
 * @date    2014-09-29
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *          File contains flight controller logic (input->control->output).
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "control.h"
#include "RCinput.h"
#include "sensors.h"
#include "motor_output.h"
#include "com.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
PWMRC_TimeTypeDef PWMInputTimes; // 6-channel PWM input width in seconds
CtrlSignals_TypeDef CtrlSignals; // Physical control signals
RefSignals_TypeDef RefSignals; // Control reference signals
PWMMotor_TimeTypeDef PWMMotorTimes; // Motor output PWM widths [s]

PIDController_TypeDef AltCtrl;
PIDController_TypeDef RollCtrl;
PIDController_TypeDef PitchCtrl;
PIDController_TypeDef YawCtrl;

/* Flight mode */
enum FlightControlMode flightMode = MANUAL_FLIGHT;
/*
 * @brief       Performs program duties with regular intervals.
 */
void UpdateControl(void)
{
  ReadSensors(); // Reads gyroscope, accelerometer and magnetometer
  GetPWMInputTimes(&PWMInputTimes); // Get 6 channel RC input pulse widths

  // Read/write USB Virtual COM
  //rwUSB();

  SetFlightMode();

  switch(flightMode)
  {
  case MANUAL_FLIGHT:
    STM_EVAL_LEDOff(LED7);
    STM_EVAL_LEDOff(LED8);
    STM_EVAL_LEDOff(LED9);
    STM_EVAL_LEDOff(LED10);

    UpdateStates(); // Updates state estimates using Kalman filtering of sensor readings
    // Set motor output to lowest
    ManualModeAllocation();
    SetMotors();
    return;

  case ATTITUDE_CONTROL:
    STM_EVAL_LEDOff(LED7);
    STM_EVAL_LEDOff(LED8);
    STM_EVAL_LEDOff(LED9);
    STM_EVAL_LEDOn(LED10);

    UpdateStates(); // Updates state estimates using Kalman filtering of sensor readings

    SetReferenceSignals();
    AltitudeControl();
    RollControl();
    PitchControl();
    YawControl();

    ControlAllocation();
    SetMotors();
    return;

  case SHUTDOWN_MOTORS:
    STM_EVAL_LEDOff(LED7);
    STM_EVAL_LEDOff(LED9);
    STM_EVAL_LEDOff(LED10);
    STM_EVAL_LEDOn(LED8);

    UpdateStates(); // Updates state estimates using Kalman filtering of sensor readings

    // Set motor output to lowest
    PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
    SetMotors();
    return;

  case INITIALIZE_STATES:
    STM_EVAL_LEDOn(LED9);
    ResetUserButton(); // Reset user button

    InitializeStateEstimation();

    // Set motor output to lowest
    PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
    SetMotors();
    return;

  case CALIBRATE_SENSORS:
    if (!GetMagCalibrated())
      {
        /* _COMPASS CALIBRATION INSTRUCTIONS_
         * Rotate the quadcopter around each of the positive and negative 3D axes (6 directions)
         * at least 360 deg (not too fast).
         * The alignment does not need to be exact and further arbitrary rotatation will
         * only be beneficial for the calibration.
         * It does not matter in which direction the quadcopter is rotated.
         * */

        STM_EVAL_LEDOn(LED3);
        ResetUserButton(); // Reset user button

        CalibrateMag();

        // Set motor output to lowest
        PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 =
            PWMMotorTimes.M4 = MIN_ESC_VAL;
        SetMotors();
      }
    else if (!GetAccCalibrated())
      {
        /* _ACCELEROMETER CALIBRATION INSTRUCTIONS_
         * Hold the quadcopter as still as possible for a few seconds in each of the following positions:
         * Level, upside-down, left side down, right side down, front down, rear down. (Does not need to be exact)
         * */

        STM_EVAL_LEDOn(LED5);
        ResetUserButton(); // Reset user button

        CalibrateAcc();

        // Set motor output to lowest
        PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 = PWMMotorTimes.M4 = MIN_ESC_VAL;
        SetMotors();
      }
    else if (!GetGyroCalibrated())
      {
        /* _GYROSCOPE CALIBRATION INSTRUCTIONS_
         * Hold the quadcopter completely still, put it on the ground or equivalent.
         * */

        STM_EVAL_LEDOn(LED7);
        ResetUserButton(); // Reset user button

        CalibrateGyro();

        // Set motor output to lowest
        PWMMotorTimes.M1 = PWMMotorTimes.M2 = PWMMotorTimes.M3 =
            PWMMotorTimes.M4 = MIN_ESC_VAL;
        SetMotors();
      }
    return;

  default:
    return;
  }
}

/* @SetFlightMode
 * @brief       Sets the Flight Mode - handles transitions between flight modes.
 * @param       None.
 * @retval      None.
 */
void SetFlightMode()
{
  if (CheckRCConnection())
    {
      if (PWMInputTimes.Gear >= GetRCmid())
        flightMode = ATTITUDE_CONTROL;
      else if (PWMInputTimes.Gear < GetRCmid() && PWMInputTimes.Gear >= 0.0)
        flightMode = MANUAL_FLIGHT;
    }
  else
    {
      flightMode = SHUTDOWN_MOTORS;
    }
}

/* @AltitudeControl
 * @brief	Controls the thrust force to achieve a desired vertical velocity
 * @param	None.
 * @retval	None.
 */
void AltitudeControl(void)
{
  float ZVelocity = GetZVelocity();

  AltCtrl.P = AltCtrl.K * (AltCtrl.B * RefSignals.ZVelocity - ZVelocity);

  // Backward difference, derivative part with zero set-point weighting
  AltCtrl.D = AltCtrl.Td / (AltCtrl.Td + AltCtrl.N * CONTROL_SAMPLE_PERTIOD) * AltCtrl.D
      - AltCtrl.K * AltCtrl.Td * AltCtrl.N / (AltCtrl.Td + AltCtrl.N * CONTROL_SAMPLE_PERTIOD)
          * (ZVelocity - AltCtrl.PreState);

  CtrlSignals.Thrust = (AltCtrl.P + AltCtrl.I + AltCtrl.D + G_ACC) * MASS;

  // Saturate controller output
  if (CtrlSignals.Thrust < 0)
    CtrlSignals.Thrust = 0;
  else if (CtrlSignals.Thrust > MAX_THRUST)
    CtrlSignals.Thrust = MAX_THRUST;

  // Forward difference, so updated after control
  if (AltCtrl.Ti != 0.0)
    AltCtrl.I = AltCtrl.I
        + AltCtrl.K * CONTROL_SAMPLE_PERTIOD / AltCtrl.Ti * (RefSignals.ZVelocity - ZVelocity);

  AltCtrl.PreState = ZVelocity;
}

/* @RollControl
 * @brief	Controls the roll moment to achieve a desired roll angle
 * @param	None.
 * @retval	None.
 */
void RollControl(void)
{
  float RollAngle = GetRoll();

  RollCtrl.P = RollCtrl.K * (RollCtrl.B * RefSignals.RollAngle - RollAngle);

  // Backward difference, derivative part with zero set-point weighting
  RollCtrl.D = RollCtrl.Td / (RollCtrl.Td + RollCtrl.N * CONTROL_SAMPLE_PERTIOD) * RollCtrl.D
      - RollCtrl.K * RollCtrl.Td * RollCtrl.N / (RollCtrl.Td + RollCtrl.N * CONTROL_SAMPLE_PERTIOD)
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
        + RollCtrl.K * CONTROL_SAMPLE_PERTIOD / RollCtrl.Ti * (RefSignals.RollAngle - RollAngle);

  RollCtrl.PreState = RollAngle;
}

/* @PitchControl
 * @brief	Controls the pitch moment to achieve a desired pitch angle
 * @param	None.
 * @retval	None.
 */
void PitchControl(void)
{
  float PitchAngle = GetPitch();

  PitchCtrl.P = PitchCtrl.K
      * (PitchCtrl.B * RefSignals.PitchAngle - PitchAngle);

  // Backward difference, derivative part with zero set-point weighting
  PitchCtrl.D = PitchCtrl.Td / (PitchCtrl.Td + PitchCtrl.N * CONTROL_SAMPLE_PERTIOD) * PitchCtrl.D
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
        + PitchCtrl.K * CONTROL_SAMPLE_PERTIOD / PitchCtrl.Ti * (RefSignals.PitchAngle - PitchAngle);

  PitchCtrl.PreState = PitchAngle;
}

/* @YawControl
 * @brief	Controls the yaw moment to achieve desired yaw rate
 * @param	None.
 * @retval	None.
 */
void YawControl(void)
{
  float YawRate = GetYawRate();

  YawCtrl.P = YawCtrl.K * (YawCtrl.B * RefSignals.YawRate - YawRate);

  // Backward difference
  YawCtrl.D = YawCtrl.Td / (YawCtrl.Td + YawCtrl.N * CONTROL_SAMPLE_PERTIOD) * YawCtrl.D
      - YawCtrl.K * YawCtrl.Td * YawCtrl.N / (YawCtrl.Td + YawCtrl.N * CONTROL_SAMPLE_PERTIOD)
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
        + YawCtrl.K * CONTROL_SAMPLE_PERTIOD / YawCtrl.Ti * (RefSignals.YawRate - YawRate);

  YawCtrl.PreState = YawRate;
}

/* SetControlSignals
 * @brief  Sets the reference values based on RC controller input
 * @param  None
 * @retval None
 */
void SetReferenceSignals(void)
{
  // Set velocity reference limits
  if (PWMInputTimes.Throttle >= GetRCmin()
      && PWMInputTimes.Throttle <= GetRCmax())
    RefSignals.ZVelocity = 2 * MAX_Z_VELOCITY * 1000
        * (PWMInputTimes.Throttle - GetRCmid());
  else
    RefSignals.ZVelocity = -MAX_Z_VELOCITY;

  // Set roll reference limits
  if (PWMInputTimes.Aileron >= GetRCmin()
      && PWMInputTimes.Aileron <= GetRCmax())
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
}

/* ControlAllocation
 * @brief  Allocates the desired thrust force and moments to corresponding motor action.
 * 		   Data has been fitted to map thrust force [N] and roll/pitch/yaw moments [Nm] to
 * 		   motor output PWM widths [us] of each of the four motors.
 * @param  None
 * @retval None
 */
void ControlAllocation(void)
{
  PWMMotorTimes.M1 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
      - M_SQRT2 * BQ * CtrlSignals.Roll - M_SQRT2 * BQ * CtrlSignals.Pitch
      - AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
      / ((float) 4 * AT * BQ * LENGTH_ARM);
  PWMMotorTimes.M2 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
      + M_SQRT2 * BQ * CtrlSignals.Roll - M_SQRT2 * BQ * CtrlSignals.Pitch
      + AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
      / ((float) 4 * AT * BQ * LENGTH_ARM);
  PWMMotorTimes.M3 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
      + M_SQRT2 * BQ * CtrlSignals.Roll + M_SQRT2 * BQ * CtrlSignals.Pitch
      - AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
      / ((float) 4 * AT * BQ * LENGTH_ARM);
  PWMMotorTimes.M4 = (BQ * LENGTH_ARM * CtrlSignals.Thrust
      - M_SQRT2 * BQ * CtrlSignals.Roll + M_SQRT2 * BQ * CtrlSignals.Pitch
      + AT * LENGTH_ARM * CtrlSignals.Yaw - 4 * BQ * CT * LENGTH_ARM)
      / ((float) 4 * AT * BQ * LENGTH_ARM);

  if (PWMMotorTimes.M1 > MAX_ESC_VAL)
    PWMMotorTimes.M1 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M1 >= MIN_ESC_VAL)
    PWMMotorTimes.M1 = PWMMotorTimes.M1;
  else
    PWMMotorTimes.M1 = MIN_ESC_VAL;

  if (PWMMotorTimes.M2 > MAX_ESC_VAL)
    PWMMotorTimes.M2 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M2 >= MIN_ESC_VAL)
    PWMMotorTimes.M2 = PWMMotorTimes.M2;
  else
    PWMMotorTimes.M2 = MIN_ESC_VAL;

  if (PWMMotorTimes.M3 > MAX_ESC_VAL)
    PWMMotorTimes.M3 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M3 >= MIN_ESC_VAL)
    PWMMotorTimes.M3 = PWMMotorTimes.M3;
  else
    PWMMotorTimes.M3 = MIN_ESC_VAL;

  if (PWMMotorTimes.M4 > MAX_ESC_VAL)
    PWMMotorTimes.M4 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M4 >= MIN_ESC_VAL)
    PWMMotorTimes.M4 = PWMMotorTimes.M4;
  else
    PWMMotorTimes.M4 = MIN_ESC_VAL;
}

/* ManualModeAllocation
 * @brief  Manual mode allocation - allocates raw RC input.
 * @param  None
 * @retval None
 */
void ManualModeAllocation(void)
{
  PWMMotorTimes.M1 = 0.9
      * (PWMInputTimes.Throttle - 2 * (PWMInputTimes.Aileron - GetRCmid())
          - 2 * (PWMInputTimes.Elevator - GetRCmid())
          - 2 * (PWMInputTimes.Rudder - GetRCmid()));
  PWMMotorTimes.M2 = 0.9
      * (PWMInputTimes.Throttle + 2 * (PWMInputTimes.Aileron - GetRCmid())
          - 2 * (PWMInputTimes.Elevator - GetRCmid())
          + 2 * (PWMInputTimes.Rudder - GetRCmid()));
  PWMMotorTimes.M3 = 0.9
      * (PWMInputTimes.Throttle + 2 * (PWMInputTimes.Aileron - GetRCmid())
          + 2 * (PWMInputTimes.Elevator - GetRCmid())
          - 2 * (PWMInputTimes.Rudder - GetRCmid()));
  PWMMotorTimes.M4 = 0.9
      * (PWMInputTimes.Throttle - 2 * (PWMInputTimes.Aileron - GetRCmid())
          + 2 * (PWMInputTimes.Elevator - GetRCmid())
          + 2 * (PWMInputTimes.Rudder - GetRCmid()));

  if (PWMMotorTimes.M1 > MAX_ESC_VAL)
    PWMMotorTimes.M1 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M1 >= MIN_ESC_VAL)
    PWMMotorTimes.M1 = PWMMotorTimes.M1;
  else
    PWMMotorTimes.M1 = MIN_ESC_VAL;

  if (PWMMotorTimes.M2 > MAX_ESC_VAL)
    PWMMotorTimes.M2 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M2 >= MIN_ESC_VAL)
    PWMMotorTimes.M2 = PWMMotorTimes.M2;
  else
    PWMMotorTimes.M2 = MIN_ESC_VAL;

  if (PWMMotorTimes.M3 > MAX_ESC_VAL)
    PWMMotorTimes.M3 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M3 >= MIN_ESC_VAL)
    PWMMotorTimes.M3 = PWMMotorTimes.M3;
  else
    PWMMotorTimes.M3 = MIN_ESC_VAL;

  if (PWMMotorTimes.M4 > MAX_ESC_VAL)
    PWMMotorTimes.M4 = MAX_ESC_VAL;
  else if (PWMMotorTimes.M4 >= MIN_ESC_VAL)
    PWMMotorTimes.M4 = PWMMotorTimes.M4;
  else
    PWMMotorTimes.M4 = MIN_ESC_VAL;
}

/* @InitPIDControllers
 * @brief	Initializes the PID controllers, i.e. sets controller parameters.
 * @param	None.
 * @retval	None.
 */
void InitPIDControllers()
{
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
}

/* @TIM6_Setup
 * @brief	Sets up the Timer 7 timebase. Timer 7 is responsible for
 * 			generating system interrupts at well-defined intervals used
 * 			to execute code at discrete time intervals.
 * @param	None.
 * @retval	None.
 */
void TIM7_Setup(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure7; // TIM7 init struct

  /* TIM7 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  /* TIM7 Time Base configuration */
  TIM_TimeBaseStructure7.TIM_Prescaler = SystemCoreClock / TIM7_FREQ - 1; // Scaling of system clock freq
  TIM_TimeBaseStructure7.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure7.TIM_Period = TIM7_FREQ / TIM7_CTRLFREQ - 1; // Counter reset value
  TIM_TimeBaseStructure7.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure7.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure7);

  /* TIM7 counter enable */
  TIM_Cmd(TIM7, ENABLE);
}

/* TIM7_SetupIRQ
 * @brief  Configures the Timer 7 IRQ Handler.
 * @param  None
 * @retval None
 */
void TIM7_SetupIRQ(void)
{

  /* Interrupt config */
  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  nvicStructure.NVIC_IRQChannelSubPriority = 0x00;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

/* @SetMotors
 * @brief       Sets the motor PWM, which is sent to the ESCs
 * @param       None.
 * @retval      None.
 */
void SetMotors()
{
  TIM_SetCompare1(TIM4, GetPWM_CCR(PWMMotorTimes.M1)); // To motor 1 (PD12)
  TIM_SetCompare2(TIM4, GetPWM_CCR(PWMMotorTimes.M2)); // To motor 2 (PD13)
  TIM_SetCompare3(TIM4, GetPWM_CCR(PWMMotorTimes.M3)); // To motor 3 (PD14)
  TIM_SetCompare4(TIM4, GetPWM_CCR(PWMMotorTimes.M4)); // To motor 4 (PD15)
}

/* @getPWM_CCR
 * @brief       Recalculates a time pulse width to number of TIM4 clock ticks.
 * @param       t is the pulse width in seconds.
 * @retval      TIM4 clock ticks to be written to TIM4 CCR output.
 */
uint16_t GetPWM_CCR(float t)
{
  return (uint16_t)((float) (t * SystemCoreClock / ((float) (TIM_GetPrescaler(TIM4) + 1))));
}
