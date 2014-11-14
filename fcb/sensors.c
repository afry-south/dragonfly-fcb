/**
******************************************************************************
* @file    fcb/sensors.c
* @author  ÅF Dragonfly - Daniel Stenberg, Embedded Systems
* @version v. 0.0.1
* @date    2014-09-26
* @brief   Functions for reading and filtering the on-board MEMS sensor
* 		   utilities.
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "sensors.h"
#include "control.h"
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/

/* Sensor */
volatile float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f}, AccAttitudeBuffer[2] = {0.0f};

uint16_t GyroCalSample = 0;
double CalRollSum = 0.0, CalPitchSum = 0.0, CalYawSum = 0.0;
volatile char GyroCalibrated = 1; // TODO RESET TO 0

float AccOffsets[3] = {0.0f};
uint16_t AccCalSample = 0;
float CalAccXSum = 0.0, CalAccYSum = 0.0, CalAccZSum = 0.0;
char AccXCalibrated = 0, AccYCalibrated = 0, AccZCalibrated = 0;
volatile char AccCalibrated = 1; // TODO RESET TO 0

float MagOffsets[3] = {0.0f};
uint16_t MagCalSample = 0;
float CalMagXMax = 1.0, CalMagXMin = -1.0, CalMagYMax = 1.0, CalMagYMin = -1.0, CalMagZMax = 1.0, CalMagZMin = -1.0;
float MagXNorm = 1.0, MagYNorm = 1.0, MagZNorm = 1.0;
volatile char MagCalibrated = 1; // TODO RESET TO 0

float InitRollSum = 0.0, InitPitchSum = 0.0, InitYawSum = 0.0;
uint16_t InitSample = 0;
volatile char StatesInitialized = 0;

StateVector_TypeDef States;

KalmanFilter_TypeDef RollEstimator;
KalmanFilter_TypeDef PitchEstimator;
KalmanFilter_TypeDef YawRateEstimator;
KalmanFilter_TypeDef ZVelocityEstimator;

/* Data acquisition variables */
uint8_t ctrlx[2];
char ctrlxIsRead = 0;

uint8_t ctrlb = 0;
char ctrlbIsRead = 0;

/* Private functions -----------------------------------------------*/

/* GyroConfig
 * @brief  Configures the MEMS to gyroscope application
 * @param  None
 * @retval None
 */
void GyroConfig(void)
{
	  L3GD20_InitTypeDef L3GD20_InitStructure;
	  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

	  /* Configure MEMS L3GD20 (See datasheet for more info) */
	  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_3;
	  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
	  L3GD20_Init(&L3GD20_InitStructure);

	  L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
	  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void GyroReadAngRate(volatile float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i = 0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);

  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity = L3G_Sensitivity_250dps;
    break;

  case 0x10:
    sensitivity = L3G_Sensitivity_500dps;
    break;

  case 0x20:
    sensitivity = L3G_Sensitivity_2000dps;
    break;
  }

  /* Divide by sensitivity */
    pfData[0] = -(float)RawData[1]/sensitivity * PI/180;	// Output in radians/second
    pfData[1] = (float)RawData[0]/sensitivity * PI/180;		// Raw data index inverted and minus sign introduced
    pfData[2] = (float)RawData[2]/sensitivity * PI/180;		// to align gyroscope with board directions
}

/** CalibrateGyro
  * @brief  Calculate the Gyroscope sensor offset
  * @param  None.
  * @retval None.
  */
void CalibrateGyro(void)
{
	if(!GyroCalibrated)
	{
		CalRollSum += GyroBuffer[0];
		CalPitchSum += GyroBuffer[1];
		CalYawSum += GyroBuffer[2];
		GyroCalSample++;

		if(GyroCalSample >= GYRO_CALIBRATION_SAMPLES)
		{
			States.rollRateBias = CalRollSum/((float)GYRO_CALIBRATION_SAMPLES);
			States.pitchRateBias = CalPitchSum/((float)GYRO_CALIBRATION_SAMPLES);
			States.yawRateBias = CalYawSum/((float)GYRO_CALIBRATION_SAMPLES);
			GyroCalibrated = 1;
		}
	}
}

/** GetGyroCalibrated
  * @brief  Returns 1 if gyroscope has been calibrated
  * @param  None.
  * @retval None.
  */
char GetGyroCalibrated(void)
{
	return GyroCalibrated;
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate = LSM303DLHC_ODR_75_HZ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_1_3_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_DISABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer HPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_8;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
  LSM303DLHC_AccFilterCmd(DISABLE);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit representation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void CompassReadAcc(volatile float* pfData)
{
  int16_t pnRawData[3];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  if(ctrlxIsRead == 0)
  {
	  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);
	  ctrlxIsRead = 1;
  }

  /* Read acceleration data */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);

  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;

  /* check in the control register4 the data alignment*/
  if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
  }
  /* Read the register content */
  //LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);

  if(ctrlx[1]&0x40)
  {
    /* FIFO mode */
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }

  /* Obtain the mg (g for gravitational acceleration) value for the three axis */
    pfData[0] = (float)pnRawData[0]/LSM_Acc_Sensitivity / 1000 - AccOffsets[0];		// Output in g (grav. acc. const.)
    pfData[1] = (float)pnRawData[1]/LSM_Acc_Sensitivity / 1000 - AccOffsets[1];
    pfData[2] = (float)pnRawData[2]/LSM_Acc_Sensitivity / 1000 - AccOffsets[2];
}

/** AccAttitude
  * @brief  Calculates attitude (roll and pitch) estimate based on accelerometer reading
  * @param  pfData: pointer to the data out
  * @retval None
  */
void AccAttitude(volatile float* pfData)
{
	float SinAccRoll = -AccBuffer[1];
	float CosAccRoll = sqrtf(1-SinAccRoll*SinAccRoll);
	float SinAccPitch = AccBuffer[0]/CosAccRoll;

	pfData[0] = asinf(fSinAccRoll);
	pfData[1] = asinf(fSinAccPitch);
}

/** CalibrateAcc
  * @brief  Calculates the Accelerometer sensor offset and magnitude scaling error
  * @param  None.
  * @retval None.
  */
void CalibrateAcc(void)
{
	if(!AccCalibrated)
	{
		float accSamples[6][3] = {0.0f};
		float accSumSq[3] = {0.0f}; // TODO move
		float accSum[3] = {0.0f};

		// First ACC_MEANVAR_SAMPLES samples used to determine mean and variance
		if(GetUserButton() >= 0x01 && AccMeanVarSample <= ACC_MEANVAR_SAMPLES)
		{
			// Insert delay/wait/timeout 1-2 seconds before first measurement

			accSum[0] += AccBuffer[0];
			accSum[1] += AccBuffer[1];
			accSum[2] += AccBuffer[2];

			accSumSq[0] += AccBuffer[0]*AccBuffer[0];
			accSumSq[1] += AccBuffer[1]*AccBuffer[1];
			accSumSq[2] += AccBuffer[2]*AccBuffer[2];

			if(AccMeanVarSample >= ACC_CALIBRATION_SAMPLES)
			{
				accSamples[GetUserButton()-1][0] = accSum[0]/ACC_CALIBRATION_SAMPLES;
				accSamples[GetUserButton()-1][1] = accSum[1]/ACC_CALIBRATION_SAMPLES;
				accSamples[GetUserButton()-1][2] = accSum[2]/ACC_CALIBRATION_SAMPLES;
				if(GetUserButton() >= 0x06)
					AccCalSampled = 1;
			}
		}

		if(AccCalSampled)
		{
			if(change >= eps && iterations < MAX_ACCCAL_ITERATIONS)
			{
				CalcAccCalMatrices();
				GetDelta();

				AccOffsets[0] -= delta_acc[0];
				AccOffsets[1] -= delta_acc[1];
				AccOffsets[2] -= delta_acc[2];

				change = delta_acc[0]*delta_acc[0] + delta_acc[1]*delta_acc[1] + delta_acc[2]*delta_acc[2];
			}
		}

	}
}

/** GetAccCalibrated
  * @brief  Returns 1 if accelerometer has been calibrated
  * @param  None.
  * @retval None.
  */
char GetAccCalibrated(void)
{
	return AccCalibrated;
}

/** CompassReadMag
  * @brief  calculate the magnetic field Magn.
  * @param  pfData: pointer to the data out
  * @retval None
  */
void CompassReadMag(volatile float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;

  if(ctrlbIsRead == 0)
  {
	  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &ctrlb, 1);
	  ctrlbIsRead = 1;
  }

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 6);

  /* Switch the sensitivity set in the CRTLB*/
  switch(ctrlb & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  // Outputs data in g (Gauss), measurement of magnetic flux density
  pfData[0] = (float) (((int16_t)(((uint16_t)buffer[0] << 8) + buffer[1])*1000)/Magn_Sensitivity_XY - MagOffsets[0])/MagXNorm;
  pfData[1] = (float) (((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_XY - MagOffsets[1])/MagYNorm;
  pfData[2] = (float) (((int16_t)(((uint16_t)buffer[2] << 8) + buffer[3])*1000)/Magn_Sensitivity_Z - MagOffsets[2])/MagZNorm;
}

/** CalibrateMag
  * @brief  Calibrate for the hard-iron distortion offset
  * @param  None.
  * @retval None.
  */
void CalibrateMag(void)
{
	if(!MagCalibrated)
	{
		if(MagBuffer[0] > CalMagXMax)
			CalMagXMax = MagBuffer[0];
		if(MagBuffer[0] < CalMagXMin)
			CalMagXMin = MagBuffer[0];

		if(MagBuffer[1] > CalMagYMax)
			CalMagYMax = MagBuffer[1];
		if(MagBuffer[1] < CalMagYMin)
			CalMagYMin = MagBuffer[1];

		if(MagBuffer[2] > CalMagZMax)
			CalMagZMax = MagBuffer[2];
		if(MagBuffer[2] < CalMagZMin)
			CalMagZMin = MagBuffer[2];

		MagCalSample++;

		if(MagCalSample >= MAG_CALIBRATION_SAMPLES)
		{
			/* Hard-iron distortion calibration */
			MagOffsets[0] = (CalMagXMax + CalMagXMin)/2.0;
			MagOffsets[1] = (CalMagYMax + CalMagYMin)/2.0;
			MagOffsets[2] = (CalMagZMax + CalMagZMin)/2.0;

			/* Scale factor calibration */
			MagXNorm = (CalMagXMax - CalMagXMin)/2.0;
			MagYNorm = (CalMagYMax - CalMagYMin)/2.0;
			MagZNorm = (CalMagZMax - CalMagZMin)/2.0;

			MagCalibrated = 1;
		}
	}
}

/** GetMagCalibrated
  * @brief  Returns 1 if magnetometer has been calibrated
  * @param  None.
  * @retval None.
  */
char GetMagCalibrated(void)
{
	return MagCalibrated;
}

/** InitializeStates
  * @brief  Initializes the attitude states
  * @param  None.
  * @retval None.
  */
void InitializeStateEstimation(void)
{
	if(!StatesInitialized)
	{
		InitRollSum += AccAttitudeBuffer[0];
		InitPitchSum += AccAttitudeBuffer[1];
		InitYawSum += GetHeading();
		InitSample++;

		if(InitSample >= INIT_SAMPLES)
		{
			States.roll = InitRollSum/((float)INIT_SAMPLES);
			States.pitch = InitPitchSum/((float)INIT_SAMPLES);
			States.yaw = InitYawSum/((float)INIT_SAMPLES);

			// TODO Estimate variances, use with motor running to take vibrations into account?
			// Or do it offline somehow?
			InitRollEstimator();
			InitPitchEstimator();
			//InitYawEstimator();
			//InitZVelocityEstimator();

			StatesInitialized = 1;
		}
	}
}

/** GetStatesInitialized
  * @brief  Returns 1 if accelerometer has been calibrated
  * @param  None.
  * @retval None.
  */
char GetStatesInitialized(void)
{
	return StatesInitialized;
}

/* InitRollEstimator
 * @brief  Initializes the roll state Kalman estimator
 * @param  None
 * @retval None
 */
void InitRollEstimator(void)
{
	RollEstimator.p11 = 1.0;
	RollEstimator.p12 = 0.0;
	RollEstimator.p21 = 0.0;
	RollEstimator.p22 = 1.0;

	RollEstimator.q1 = 0.5;
	RollEstimator.q2 = 0.05;
	RollEstimator.r1 = 1.5;
}

/* UpdateRoll
 * @brief  Updates the roll state estimate using Kalman filtering
 * @param  None
 * @retval None
 */
void UpdateRollEstimate(void)
{
	/* Get roll rate from sensors: RollRate = p + (q*sinRoll+r*cosRoll)*tanPitch */
	float rollRateGyro = GyroBuffer[0] + (GyroBuffer[1]*sinf(States.roll) + GyroBuffer[1]*cosf(States.roll))*tanf(States.pitch);

	/* Prediction */
	States.roll = States.roll + H*rollRateGyro - H*States.rollRateBias;
	States.rollRateBias = States.rollRateBias;

	RollEstimator.p11 = RollEstimator.p11-RollEstimator.p21*H - H*(RollEstimator.p12-RollEstimator.p22*H) + RollEstimator.q1;
	RollEstimator.p12 = RollEstimator.p12 - RollEstimator.p22*H;
	RollEstimator.p21 = RollEstimator.p21 - RollEstimator.p22*H;
	RollEstimator.p22 = RollEstimator.p22 + RollEstimator.q2;

	/* Correction */
	RollEstimator.k1 = RollEstimator.p11/(RollEstimator.p11+RollEstimator.r1);
	RollEstimator.k2 = RollEstimator.p21/(RollEstimator.p11+RollEstimator.r1);

	States.roll = States.roll + RollEstimator.k1*(AccAttitudeBuffer[0]-States.roll);
	States.rollRateBias = States.rollRateBias + RollEstimator.k2*(AccAttitudeBuffer[0]-States.roll);

	RollEstimator.p11 = RollEstimator.p11*(1-RollEstimator.k1);
	RollEstimator.p12 = RollEstimator.p12*(1-RollEstimator.k1);
	RollEstimator.p21 = RollEstimator.p21 - RollEstimator.k2*RollEstimator.p11;
	RollEstimator.p22 = RollEstimator.p22 - RollEstimator.k2*RollEstimator.p12;
}

/* InitPitchEstimator
 * @brief  Initializes the pitch state Kalman estimator
 * @param  None
 * @retval None
 */
void InitPitchEstimator(void)
{
	PitchEstimator.p11 = 1.0;
	PitchEstimator.p12 = 0.0;
	PitchEstimator.p21 = 0.0;
	PitchEstimator.p22 = 1.0;

	PitchEstimator.q1 = 0.5;
	PitchEstimator.q2 = 0.05;
	PitchEstimator.r1 = 1.5;
}

/* UpdatePitch
 * @brief  Updates the pitch state estimate using Kalman filtering
 * @param  None
 * @retval None
 */
void UpdatePitchEstimate(void)
{
	/* Get pitch rate from sensors: PitchRate = q*cosRoll - r*sinRoll */
	float pitchRateGyro = GyroBuffer[1]*cosf(States.roll) - GyroBuffer[1]*sinf(States.roll);

	/* Prediction */
	States.pitch = States.pitch + H*pitchRateGyro - H*States.pitchRateBias;
	States.pitchRateBias = States.pitchRateBias;

	PitchEstimator.p11 = PitchEstimator.p11-PitchEstimator.p21*H - H*(PitchEstimator.p12-PitchEstimator.p22*H) + PitchEstimator.q1;
	PitchEstimator.p12 = PitchEstimator.p12 - PitchEstimator.p22*H;
	PitchEstimator.p21 = PitchEstimator.p21 - PitchEstimator.p22*H;
	PitchEstimator.p22 = PitchEstimator.p22 + PitchEstimator.q2;

	/* Correction */
	PitchEstimator.k1 = PitchEstimator.p11/(PitchEstimator.p11+PitchEstimator.r1);
	PitchEstimator.k2 = PitchEstimator.p21/(PitchEstimator.p11+PitchEstimator.r1);

	States.pitch = States.pitch + PitchEstimator.k1*(AccAttitudeBuffer[1]-States.pitch);
	States.pitchRateBias = States.pitchRateBias + PitchEstimator.k2*(AccAttitudeBuffer[1]-States.pitch);

	PitchEstimator.p11 = PitchEstimator.p11*(1-PitchEstimator.k1);
	PitchEstimator.p12 = PitchEstimator.p12*(1-PitchEstimator.k1);
	PitchEstimator.p21 = PitchEstimator.p21 - PitchEstimator.k2*RollEstimator.p11;
	PitchEstimator.p22 = PitchEstimator.p22 - PitchEstimator.k2*PitchEstimator.p12;
}

/* GetRoll
 * @brief  Gets the roll angle
 * @param  None
 * @retval Roll angle state
 */
float GetRoll(void)
{
	return States.roll;
}

/* GetPitch
 * @brief  Gets the pitch angle
 * @param  None
 * @retval Pitch angle state
 */
float GetPitch(void)
{
	return States.pitch;
}

void UpdateYawRateEstimate(void)
{
	States.yawRate = (GyroBuffer[1]*sinf(States.roll)+GyroBuffer[2]*cosf(States.roll))/cosf(States.pitch);
}

/* GetYawRate
 * @brief  Gets the yaw angular rate
 * @param  None
 * @retval Yaw angular rate state
 */

float GetYawRate(void)
{
	return States.yawRate;
}

/* UpdateZVelocity
 * @brief  Updates the Z velocity state
 * @param  None
 * @retval None
 */
void UpdateZVelocityEstimate(void)
{
	float ZAcc = (-AccBuffer[0]*sinf(States.pitch)+AccBuffer[1]*sinf(States.roll)*cosf(States.pitch)+AccBuffer[2]*cosf(States.roll)*cosf(States.pitch))*G_ACC - G_ACC;
	States.ZVelocity += ZAcc*H;
}

/* GetZVelocity
 * @brief  Gets the vertical z velocity
 * @param  None
 * @retval Z velocity state
 */
float GetZVelocity(void)
{
	return States.ZVelocity;
}

float GetHeading(void)
{
	float fTiltedX = MagBuffer[0]*cosf(States.pitch) + MagBuffer[1]*sinf(States.roll)*sinf(States.pitch) + MagBuffer[2]*cosf(States.roll)*sinf(States.pitch);
	float fTiltedY = MagBuffer[1]*cosf(States.roll) - MagBuffer[2]*sinf(States.roll);

	float HeadingValue = atan2f(fTiltedY, fTiltedX) - COMPASS_DECLINATION;

	if (HeadingValue < 0)
	{
	  HeadingValue = HeadingValue + 2.0*PI;
	}

	return HeadingValue;
}

/* ReadSensors
 * @brief  Updates the sensor buffers with the latest sensor values
 * @param  None
 * @retval None
 */
void ReadSensors(void)
{
	GyroReadAngRate(GyroBuffer);
	CompassReadAcc(AccBuffer);
	CompassReadMag(MagBuffer);
	AccAttitude(AccAttitudeBuffer);
}

/* UpdateStates
 * @brief  Updates the state estimates
 * @param  None
 * @retval None
 */
void UpdateStates(void)
{
	UpdateRollEstimate();
	UpdatePitchEstimate();
	UpdateYawRateEstimate();
	UpdateZVelocityEstimate();
}

/**
  * @brief  Basic management of the L3GD20 timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
	return 0;
}

/**
  * @brief  Basic management of the LSM303DLHC timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
	return 0;
}
