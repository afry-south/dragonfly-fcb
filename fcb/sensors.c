/**
******************************************************************************
* @file    fcb/sensors.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-26
* @brief   Functions for the on-board MEMS sensor utilities
* 		   NOTE: dps = degrees per second (angular velocity unit)
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "sensors.h"
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};	// Body-frame sensor readings arrays

float GyroOffsets[3] = {0.0f};
uint16_t GyroCalSample = 0;
float CalRollSum = 0.0, CalPitchSum = 0.0, CalYawSum = 0.0;
volatile char GyroCalibrated = 0;

float AccOffsets[3] = {0.0f};
uint16_t AccCalSample = 0;
float CalAccXSum = 0.0, CalAccYSum = 0.0, CalAccZSum = 0.0;
volatile char AccCalibrated = 0;

//float MagOffsets[3] = {0.0f};
//uint16_t MagCalSample = 0;
//float CalMagXSum = 0.0, CalMagYSum = 0.0, CalMagZSum = 0.0;
//volatile char MagCalibrated = 0;
//volatile float MagYawOffset = 0.0;

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

	  /* Configure Mems L3GD20 (See datasheet for more info) */
	  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_2;
	  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_2;
	  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
	  L3GD20_Init(&L3GD20_InitStructure);

	  L3GD20_FilterStructure.HighPassFilter_Mode_Selection = L3GD20_HPM_NORMAL_MODE_RES;
	  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_9;
	  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
}

/**
  * @brief  Calculate the angular Data rate Gyroscope.
  * @param  pfData : Data out pointer
  * @retval None
  */
void GyroReadAngRate(float* pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

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
    pfData[0] = -(float)RawData[1]/sensitivity * PI/180 - GyroOffsets[0];	// Output in radians/second
    pfData[1] = (float)RawData[0]/sensitivity * PI/180 - GyroOffsets[1];	// Raw data index inverted and minus sign introduced
    pfData[2] = (float)RawData[2]/sensitivity * PI/180 - GyroOffsets[2];	// to align gyroscope with board directions
}

/** CalibrateGyro
  * @brief  Calculate the Gyroscope sensor offset (Median, TODO make 3-point median and then run average)
  * @param  None.
  * @retval None.
  */
void CalibrateGyro(void)
{
	if(!GyroCalibrated)
	{
		GyroReadAngRate(GyroBuffer);
		CalRollSum += GyroBuffer[0];
		CalPitchSum += GyroBuffer[1];
		CalYawSum += GyroBuffer[2];
		GyroCalSample++;

		if(GyroCalSample >= GYRO_CALIBRATION_SAMPLES)
		{
			GyroOffsets[0] = CalRollSum/((float)GYRO_CALIBRATION_SAMPLES);
			GyroOffsets[1] = CalPitchSum/((float)GYRO_CALIBRATION_SAMPLES);
			GyroOffsets[2] = CalYawSum/((float)GYRO_CALIBRATION_SAMPLES);
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
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_4_0_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_100_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
  LSM303DLHCAcc_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_64;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

/**
* @brief Read LSM303DLHC output register, and calculate the acceleration ACC=(1/SENSITIVITY)* (out_h*256+out_l)/16 (12 bit representation)
* @param pnData: pointer to float buffer where to store data
* @retval None
*/
void CompassReadAcc(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;

  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
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
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);


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
    // pfData[i] = (float) pnRawData[i]/LSM_Acc_Sensitivity - AccOffsets[i];			// Output in mg (10^(-3) g)

    pfData[0] = (float)pnRawData[0]/LSM_Acc_Sensitivity / 1000 * g - AccOffsets[0];		// Output in m/(s^2)
    pfData[1] = (float)pnRawData[1]/LSM_Acc_Sensitivity / 1000 * g - AccOffsets[1];
    pfData[2] = (float)pnRawData[2]/LSM_Acc_Sensitivity / 1000 * g - AccOffsets[2];

}

/** CalibrateAcc
  * @brief  Calculate the Accelerometer sensor offset
  * @param  None.
  * @retval None.
  */
void CalibrateAcc(void)
{
	if(!AccCalibrated)
	{
		CompassReadAcc(AccBuffer);
		CalAccXSum += AccBuffer[0];
		CalAccYSum += AccBuffer[1];
		CalAccZSum += AccBuffer[2];
		AccCalSample++;

		if(AccCalSample >= ACC_CALIBRATION_SAMPLES)
		{
			AccOffsets[0] = CalAccXSum/((float)ACC_CALIBRATION_SAMPLES);
			AccOffsets[1] = CalAccYSum/((float)ACC_CALIBRATION_SAMPLES);
			AccOffsets[2] = CalAccZSum/((float)ACC_CALIBRATION_SAMPLES) - g;
			AccCalibrated = 1;
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

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void CompassReadMag(float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
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
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}

/** CalibrateMag
  * @brief  Calculate the Magnetometer sensor offset
  * @param  None.
  * @retval None.
  */
//void CalibrateMag(void)
//{
//	if(!MagCalibrated)
//	{
//		CompassReadMag(MagBuffer);
//		CalMagXSum += MagBuffer[0];
//		CalMagYSum += MagBuffer[1];
//		CalMagZSum += MagBuffer[2];
//		MagCalSample++;
//
//		if(MagCalSample >= MAG_CALIBRATION_SAMPLES)
//		{
//			MagOffsets[0] = CalMagXSum/((float)MAG_CALIBRATION_SAMPLES);
//			MagOffsets[1] = CalMagYSum/((float)MAG_CALIBRATION_SAMPLES);
//			MagOffsets[2] = CalMagZSum/((float)MAG_CALIBRATION_SAMPLES);
//
//		    MagYawOffset = (float) (atan2f((float)MagOffsets[1], (float)MagOffsets[0]));
//
//			MagCalibrated = 1;
//		}
//	}
//}

/** GetMagCalibrated
  * @brief  Returns 1 if magnetometer has been calibrated
  * @param  None.
  * @retval None.
  */
//char GetMagCalibrated(void)
//{
//	return MagCalibrated;
//}

/** GetMagYawOffset
  * @brief  Returns they yaw angle offset (magnetometer)
  * @param  None.
  * @retval None.
  */
//float GetMagYawOffset(void)
//{
//	return MagYawOffset;
//}

/* GetBodyAttitude
 * @brief  Discrete integration of angular rate
 * @param  None
 * @retval None
 */
void GetBodyAttitude(float *pfData, float h)
{
	/* Complementary sensor fusion filter. TODO: Possibly Kalman or Madgwick sensor filter */
	pfData[0] += 0.98*GyroBuffer[0]*h; + 0.02*atan2f(-AccBuffer[1], AccBuffer[2]);	// Roll angle
	pfData[1] += 0.98*GyroBuffer[1]*h; + 0.02*atan2f(AccBuffer[0], AccBuffer[2]);	// Pitch angle
	pfData[2] += GyroBuffer[2]*h; 													// Yaw angle
}

/* GetBodyVelocity
 * @brief  Discrete integration of linear acceleration
 * @param  None
 * @retval None
 */
void GetBodyVelocity(float *pfData, float h)
{
	pfData[0] += AccBuffer[0]*h;
	pfData[1] += AccBuffer[1]*h;
	pfData[2] += (AccBuffer[2]-g)*h;
}

float GetYawRate(void)
{
	return GyroBuffer[2];
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
}
