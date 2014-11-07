/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

//typedef struct
//{
//	float P0;
//	float Q;
//	float R;
//	float K;
//}KalmanFilter_TypeDef; // TODO

/* Private define ------------------------------------------------------------*/
#define L3G_Sensitivity_250dps		(float)   	114.285f        /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     	(float)    	57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    	(float)    	14.285f	      	/*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
//#define PI                         (float)     3.14159265f
#define G_ACC						   	(float)	   	9.815		/* Gravitational acceleration constant */

#define LSM_Acc_Sensitivity_2g     	(float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     	(float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     	(float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    	(float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define GYRO_CALIBRATION_SAMPLES   	(int)	   	500
#define ACC_CALIBRATION_SAMPLES		(int)	   	500
#define MAG_CALIBRATION_SAMPLES    	(int)		3000

#define COMPASS_DECLINATION		   	(float)		3.226*PI/180.0	/* For Malmoe, Sweden the compass declination is about 3 deg East
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * The inclination angle is about 70 deg
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * The total field strength is 50552 nT (505.52 mGauss) */

/* Includes */
#include "stm32f30x.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery.h"
#include <stdio.h>
#include "math.h"
#include "arm_math.h"

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

void GyroConfig(void);
void CompassConfig(void);
void GyroReadAngRate(volatile float* pfData);
void CompassReadMag(volatile float* pfData);
void CompassReadAcc(volatile float* pfData);
void CalibrateGyro(void);
void CalibrateAcc(void);
void CalibrateMag(void);
char GetGyroCalibrated(void);
char GetAccCalibrated(void);
char GetMagCalibrated(void);
void GetBodyAttitude(float *pfData);
void GetBodyVelocity(float *pfData);
float GetHeading(void);
float GetYawRate(void);
void ReadSensors(void);

#endif /* __SENSORS_H */

/* **** END OF FILE ****/
