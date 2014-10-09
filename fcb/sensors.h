/* Private define ------------------------------------------------------------*/
#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */
#define PI                         (float)     3.14159265f
#define g						   (float)	   9.815		  /* Gravitational acceleration constant */

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define GYRO_CALIBRATION_SAMPLES   (int)	   500
#define ACC_CALIBRATION_SAMPLES    (int)	   500
#define MAG_CALIBRATION_SAMPLES    (int)	   500

/* Includes */
#include "stm32f30x.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include "stm32f3_discovery.h"
#include <stdio.h>
#include "math.h"

/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

void GyroConfig(void);
void CompassConfig(void);
void GyroReadAngRate(float* pfData);
void CompassReadMag(float* pfData);
void CompassReadAcc(float* pfData);
void CalibrateGyro(void);
void CalibrateAcc(void);
void CalibrateMag(void);
char GetGyroCalibrated(void);
char GetAccCalibrated(void);
void GetBodyAttitude(float *pfData, float h);
void GetBodyVelocity(float *pfData, float h);
float GetYawRate(void);
void ReadSensors(void);
