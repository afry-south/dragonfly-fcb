/******************************************************************************
 * @file    flight_control.h
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-12
 * @brief   Header file for flight control
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"

/* Exported constants --------------------------------------------------------*/

/* Physical properties of aircraft */
// TODO make physical properties struct
#define LENGTH_ARM 	        0.30		// Quadcopter arm length [m] // TODO use int values mm, g, g/(m^2)
#define MASS		        2.100		// Total mass of the quadcopter [kg]
#define IXX					0.030		// X-axis moment of inertia [kg/(m^2)]
#define IYY					0.030		// Y-axis moment of inertia [kg/(m^2)]
#define IZZ					0.060		// Z-axis moment of inertia [kg/(m^2)]

/* Reference signal ranges */
#define MAX_Z_VELOCITY			2.0			// Max vertical velocity (+/-) [m/s] // TODO use int values m/s, mrad, mrad/s
#define MAX_ROLLPITCH_ANGLE 	15*PI/180	// Max roll/pitch angle (+/-) [rad] (NOTE! Not in deg!)
#define MAX_YAW_RATE			30*PI/180	// Max yaw angle rate [rad/s] (NOTE! Not deg/s)

enum FlightControlMode {
	FLIGHT_CONTROL_IDLE,
	FLIGHT_CONTROL_RAW,
	FLIGHT_CONTROL_PID
};

/* Exported types ------------------------------------------------------------*/

typedef enum {
	FLIGHTCTRL_ERROR = 0, FLIGHTCTRL_OK = !FLIGHTCTRL_ERROR
} FlightControlErrorStatus;

typedef struct
{
  float ZVelocity;		// [m/s] // TODO use int values mm/s, mrad, mrad/s etc
  float RollAngle;		// [rad]
  float PitchAngle;		// [rad]
  float YawAngleRate;	// [rad/s]
} RefSignals_TypeDef;
// Type could be updated later on to include reference values of velocity and position XYZ

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void CreateFlightControlTask(void);
enum FlightControlMode GetFlightControlMode(void);
float GetZVelocityReferenceSignal(void);
float GetRollAngleReferenceSignal(void);
float GetPitchReferenceSignal(void);
float GetYawAngularRateReferenceSignal(void);

#endif /* __FLIGHT_CONTROL_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
