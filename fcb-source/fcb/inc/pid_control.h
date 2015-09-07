/*****************************************************************************
 * @file    pid_control.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-31
 * @brief   Header file for PID control algorithm implementation
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_CONTROL_H_
#define __PID_CONTROL_H_

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

#include "arm_math.h"

/* Exported constants --------------------------------------------------------*/
/* Vertical control parameters */
#define K_VZ 				8.0
#define TI_VZ				0.0
#define TD_VZ				0.5
#define BETA_VZ				1.0			// Proportional set-point weighting
#define N_VZ				15.0		// Max derivative gain (often 10-20)
#define MAX_THRUST			4*AT*UINT16_MAX + 4*BT	// Maximal upward thrust from all four motors combined [N]

/* Roll/pitch control parameters */
#define K_RP				8.0			// Roll/pitch angle controller parameters
#define TI_RP				0.0
#define TD_RP				0.875
#define BETA_RP				1.0			// Proportional set-point weighting
#define N_RP				15.0		// Max derivative gain (often 10-20)
#define MAX_ROLLPITCH_MOM	MAX_THRUST*M_SQRT2/2*LENGTH_ARM		// Two motors full thrust, two motors no thrust [Nm]

/* Yaw control parameters */
#define K_YR 				2.0
#define TI_YR 				0.0
#define TD_YR				0.5
#define BETA_YR				1.0			// Proportional set-point weighting
#define N_YR				15.0		// Max derivative gain (often 10-20)
#define MAX_YAW_MOM			AQ*2*UINT16_MAX		// Two motors with same rot dir full thrust, other two motors no thrust [Nm]

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float K;		// PID gain parameter
  float Ti;		// PID integration time parameter
  float Td;		// PID derivative time parameter
  float B;		// Set-point weighting 0-1
  float N;		// Derivative action filter constant
  float P;		// Proportional control part
  float I;		// Integration control part
  float D;		// Derivative control part
  float PreState;	// Previous value of control variable state
}PIDController_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void AltitudeControl(void);
void RollControl(void);
void PitchControl(void);
void YawControl(void);
void InitPIDControllers(void);

#endif /* __PID_CONTROL_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
