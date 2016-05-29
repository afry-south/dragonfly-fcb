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
#include "fcb_sensors.h"

/* Exported constants --------------------------------------------------------*/

#define FLIGHT_CONTROL_TASK_PERIOD		5 // [ms]

/* Physical properties of aircraft */

/* Quadcopter arm length [m] (measured) */
#define LENGTH_ARM 	            ((float32_t) 0.30)

/* Total mass of the quadcopter [kg]
 * # From CAD model w/o chassi : MASS = 2.1983808e+00 KILOGRAM
 * # From CAD model w/ chassi  : MASS = 2.3523396e+00 KILOGRAM
 * # NOTE: CAD model does not include masses of on-board electronics (e.g. PCBs and charger)
 */
#define MASS		            ((float32_t) 2.1983808)

/* Inertia tensor of the quadcopter [kg]
 *
 * # From CAD model w/o chassi :
 *
 * INERTIA at CENTER OF GRAVITY with respect to _QUADCOPTER_SAMMANSTALLNING coordinate frame:  (KILOGRAM * MM^2)
 * INERTIA TENSOR:
 * Ixx Ixy Ixz  4.0654730e+04  1.1429530e+03 -1.6499272e+01
 * Iyx Iyy Iyz  1.1429530e+03  4.0693168e+04  4.3305781e+01
 * Izx Izy Izz -1.6499272e+01  4.3305781e+01  7.4656405e+04
 *
 * # From CAD model w/ chassi  :
 *
 * INERTIA at CENTER OF GRAVITY with respect to _QUADCOPTER_SAMMANSTALLNING coordinate frame:  (KILOGRAM * MM^2)
 * INERTIA TENSOR:
 * Ixx Ixy Ixz  4.1942971e+04  1.1949577e+03 -1.8072975e+01
 * Iyx Iyy Iyz  1.1949577e+03  4.1981406e+04  4.0972106e+01
 * Izx Izy Izz -1.8072975e+01  4.0972106e+01  7.5746409e+04
 *
 * # NOTE: CAD model does not include masses/inertia contributions of on-board electronics (e.g. PCBs and charger)
 */

#define IXX					    ((float32_t) 0.04065473)		// X-axis moment of inertia [kg/(m^2)]
#define IYY					    ((float32_t) 0.040693168)		// Y-axis moment of inertia [kg/(m^2)]
#define IZZ					    ((float32_t) 0.074656405)		// Z-axis moment of inertia [kg/(m^2)]

#define G_ACC                   ((float32_t) 9.815)	/* Gravitational acceleration constant approx. 9.815 m/s^2 in
                                                     * Smygehuk, Sweden (according to Lantmateriet) */

#define COMPASS_DECLINATION	    ((float32_t) 3.226*PI/180.0)	/* For Malmoe, Sweden the compass declination is about 3.226 deg East
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * The total field strength is 50552 nT (505.52 mGauss) */
/* Default reference signal ranges */
#define DEFAULT_MAX_Z_VELOCITY          2.0			// Max vertical velocity (+/-) [m/s] // TODO use int values m/s, mrad, mrad/s
#define DEFAULT_MAX_ROLLPITCH_ANGLE     15*PI/180	// Max roll/pitch angle (+/-) [rad] (NOTE! Not in deg!)
#define DEFAULT_MAX_YAW_RATE            30*PI/180	// Max yaw angle rate [rad/s] (NOTE! Not deg/s)

#define RECEIVER_TO_REFERENCE_ZERO_PADDING	1800	// Sets how large an area around 0 receiver value the reference signal should be set to zero

enum FlightControlMode {
	FLIGHT_CONTROL_IDLE,
	FLIGHT_CONTROL_RAW,
	FLIGHT_CONTROL_PID,
	FLIGHT_CONTROL_AUTONOMOUS
};

/* Exported types ------------------------------------------------------------*/

typedef enum {
	FLIGHTCTRL_ERROR = 0, FLIGHTCTRL_OK = !FLIGHTCTRL_ERROR
} FlightControlErrorStatus;

typedef struct
{
  float32_t ZVelocity;		// [m/s]
  float32_t RollAngle;		// [rad]
  float32_t PitchAngle;		// [rad]
  float32_t YawAngleRate;	// [rad/s]
} RefSignals_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void CreateFlightControlTask(void);
enum FlightControlMode GetFlightControlMode(void);

float32_t GetZVelocityReferenceSignal(void);
float32_t GetRollAngleReferenceSignal(void);
float32_t GetPitchAngleReferenceSignal(void);
float32_t GetYawAngularRateReferenceSignal(void);

float32_t GetThrustControlSignal();
float32_t GetRollControlSignal();
float32_t GetPitchControlSignal();
float32_t GetYawControlSignal();

void ResetRefSignals(RefSignals_TypeDef* refSignals);

void SendFlightControlUpdateToFlightControl(void);
/**
 * This function sends a message to the flight control queue to indicate that a new prediction shall be calculated.
 */
void SendPredictionUpdateToFlightControl(void);

/**
 * This function sends a message to the flight control queue with new sensor values.
 *
 * @param sensorType see FcbSensorIndexType
 * @param deltaT time period to previous sensor drdy in ms
 * @param xyz a 3-array of XYZ sensor readings. See wiki page "Sensors"
 */
void SendCorrectionUpdateToFlightControl(FcbSensorIndexType sensorType, uint8_t deltaTms, float32_t xyz[3]);

void setMaxLimitForReferenceSignal(float32_t maxZVelocity, float32_t maxRollAngle, float32_t maxPitchAngle, float32_t maxYawAngleRate);
void getMaxLimitForReferenceSignal(float32_t* maxZVelocity, float32_t* maxRollAngle, float32_t* maxPitchAngle, float32_t* maxYawAngleRate);

#endif /* __FLIGHT_CONTROL_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
