/*
 * bmp180.c
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#include "bmp180.h"
#include "stm32f3_discovery.h"

/***************************************************************/
/**\name	REGISTER ADDRESS DEFINITION       */
/***************************************************************/
/*register definitions */

#define BMP180_CALIB_PARAM_START_REG	(0xAA)
#define BMP180_CALIB_PARAM_DATA_LEN		(22)
//
#define BMP180_CHIP_ID_REG				(0xD0)
#define BMP180_VERSION_REG				(0xD1)
//
#define BMP180_CTRL_MEAS_REG			(0xF4)
#define BMP180_ADC_OUT_REG				(0xF6)
//#define BMP180_ADC_OUT_LSB_REG		(0xF7)
//
//#define BMP180_SOFT_RESET_REG		(0xE0)
//
/* temperature measurement */
#define BMP180_T_MEASURE			(0x2E)
/* pressure measurement*/
#define BMP180_P_MEASURE			(0x34)
///* TO be spec'd by GL or SB*/
//#define BMP180_TEMP_CONVERSION_TIME  (5)
//
//#define BMP180_PARAM_MG		(3038)
//#define BMP180_PARAM_MH		(-7357)
//#define BMP180_PARAM_MI		(3791)

/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
//#define	BMP180_TEMPERATURE_DATA_BYTES	(2)
//#define	BMP180_PRESSURE_DATA_BYTES		(3)
//#define	BMP180_TEMPERATURE_LSB_DATA		(1)
//#define	BMP180_TEMPERATURE_MSB_DATA		(0)
//#define	BMP180_PRESSURE_MSB_DATA		(0)
//#define	BMP180_PRESSURE_LSB_DATA		(1)
//#define	BMP180_PRESSURE_XLSB_DATA		(2)
//
//#define	BMP180_CALIB_DATA_SIZE			(22)
//#define	BMP180_CALIB_PARAM_AC1_MSB		(0)
//#define	BMP180_CALIB_PARAM_AC1_LSB		(1)
//#define	BMP180_CALIB_PARAM_AC2_MSB		(2)
//#define	BMP180_CALIB_PARAM_AC2_LSB		(3)
//#define	BMP180_CALIB_PARAM_AC3_MSB		(4)
//#define	BMP180_CALIB_PARAM_AC3_LSB		(5)
//#define	BMP180_CALIB_PARAM_AC4_MSB		(6)
//#define	BMP180_CALIB_PARAM_AC4_LSB		(7)
//#define	BMP180_CALIB_PARAM_AC5_MSB		(8)
//#define	BMP180_CALIB_PARAM_AC5_LSB		(9)
//#define	BMP180_CALIB_PARAM_AC6_MSB		(10)
//#define	BMP180_CALIB_PARAM_AC6_LSB		(11)
//#define	BMP180_CALIB_PARAM_B1_MSB		(12)
//#define	BMP180_CALIB_PARAM_B1_LSB		(13)
//#define	BMP180_CALIB_PARAM_B2_MSB		(14)
//#define	BMP180_CALIB_PARAM_B2_LSB		(15)
//#define	BMP180_CALIB_PARAM_MB_MSB		(16)
//#define	BMP180_CALIB_PARAM_MB_LSB		(17)
//#define	BMP180_CALIB_PARAM_MC_MSB		(18)
//#define	BMP180_CALIB_PARAM_MC_LSB		(19)
//#define	BMP180_CALIB_PARAM_MD_MSB		(20)
//#define	BMP180_CALIB_PARAM_MD_LSB		(21)

static uint8_t oss = 0;	//Oversampling ratio;

void BMP180_init(void) {
	I2Cbar_Init();

	uint8_t chipId;
	I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_CHIP_ID_REG, &chipId, 1);

	I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_VERSION_REG, &chipId, 1);
}

void BMP180_readCalibVals(BMP180CalibVals_t *calibVals) {
	I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_CALIB_PARAM_START_REG, (uint8_t *)calibVals, BMP180_CALIB_PARAM_DATA_LEN);
}

void BMP180_StartPressureMeasure(void) {
	I2Cbar_WriteData(BAROMETER_I2C_ADDRESS, BMP180_CTRL_MEAS_REG, BMP180_P_MEASURE | (oss << 6));
}

HAL_StatusTypeDef BMP180_ReadPressureValue(float * pData) {
    HAL_StatusTypeDef status = 0;
    uint8_t dataRead[3];
    uint32_t rawValue;

    status = I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_ADC_OUT_REG, dataRead, 3);
    rawValue = dataRead[0]<<(8+oss) | dataRead[1]<<(oss) | dataRead[2]>>(8-oss);

    *pData = rawValue;

    return status;
}
