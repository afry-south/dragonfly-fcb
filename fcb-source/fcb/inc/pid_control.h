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
#include "arm_math.h"

/* Exported constants --------------------------------------------------------*/
#define PID_USE_PARALLEL_FORM

/* Vertical control parameters */
#define K_VZ 				(float32_t) 2.0
#define TI_VZ				(float32_t) 0.0
#define TD_VZ				(float32_t) 0.5
#define BETA_VZ				(float32_t) 1.0			// Proportional set-point weighting
#define GAMMA_VZ			(float32_t) 0.0
#define N_VZ				(float32_t) 15.0		// Max derivative gain (often 10-20)
#define MAX_THRUST			(float32_t) 4*AT*UINT16_MAX + 4*BT	// Maximal upward thrust from all four motors combined [N]

/* Roll/pitch angle control parameters */
#define K_RP				(float32_t) 100.0
#define TI_RP				(float32_t) 0.0
#define TD_RP				(float32_t) 20.0
#define BETA_RP				(float32_t) 1.0			// Proportional set-point weighting
#define GAMMA_RP			(float32_t) 0.0
#define N_RP				(float32_t) 15.0		// Max derivative gain (often 10-20)
#define MAX_ROLLPITCH_MOM	(float32_t) MAX_THRUST/2*LENGTH_ARM/M_SQRT2	// Two motors full thrust, two motors no thrust [Nm]

/* Yaw angular rate control parameters */
#define K_YR 				(float32_t) 5.0
#define TI_YR 				(float32_t) 0.0
#define TD_YR				(float32_t) 0.0
#define BETA_YR				(float32_t) 1.0			// Proportional set-point weighting
#define GAMMA_YR			(float32_t) 0.0
#define N_YR				(float32_t) 15.0		// Max derivative gain (often 10-20)
#define MAX_YAW_MOM			(float32_t) AQ*2*UINT16_MAX		// Two motors with same rot dir full thrust, other two motors no thrust [Nm]

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float32_t Thrust;			// [N]
  float32_t RollMoment;		// [Nm]
  float32_t PitchMoment;	// [Nm]
  float32_t YawMoment;		// [Nm]
} CtrlSignals_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void InitPIDControllers(void);
void UpdatePIDControlSignals(CtrlSignals_TypeDef* ctrlSignals);

#endif /* __PID_CONTROL_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
