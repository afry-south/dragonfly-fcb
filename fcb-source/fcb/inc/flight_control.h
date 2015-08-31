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
/* Exported constants --------------------------------------------------------*/

/* Physical properties of aircraft */
#define LENGTH_ARM 	        0.30		// Quadcopter arm length [m]
#define MASS		        2.100		// Total mass of the quadcopter [kg]
#define IXX					0.030		// X-axis moment of inertia [kg/(m^2)]
#define IYY					0.030		// Y-axis moment of inertia [kg/(m^2)]
#define IZZ					0.060		// Z-axis moment of inertia [kg/(m^2)]

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
  float ZVelocity;		// [m/s]
  float RollAngle;		// [rad]
  float PitchAngle;		// [rad]
  float YawRate;		// [rad/s]
} RefSignals_TypeDef;
// Type could be updated later on to include reference values of velocity and position XYZ

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */

#endif /* __FLIGHT_CONTROL_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
