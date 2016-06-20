/*
 * bmp180.c
 *
 *  Created on: Jun 14, 2016
 *      Author: a492478
 */

#include "bmp180.h"
#include "stm32f3_discovery.h"

/* Private define ------------------------------------------------------------*/

/***************************************************************/
/**\name	REGISTER ADDRESS DEFINITION       */
/***************************************************************/

#define BMP180_CALIB_PARAM_START_REG	(0xAA)
#define BMP180_CALIB_PARAM_DATA_LEN		(22)
//
#define BMP180_CTRL_MEAS_REG			(0xF4)
#define BMP180_ADC_OUT_REG				(0xF6)

/* temperature measurement */
#define BMP180_T_MEASURE			(0x2E)
/* pressure measurement*/
#define BMP180_P_MEASURE			(0x34)

/* Private typedef -----------------------------------------------------------*/

typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} BMP180CalibVals_t;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static BMP180CalibVals_t calibVals;
static uint8_t oss = 0;	// Over sampling ratio;
static uint32_t rawTempValue;

/* Private function prototypes -----------------------------------------------*/

static void ReadCalibVals(BMP180CalibVals_t *calibVals);
static int32_t CalculateRealPreassure(int32_t rawTemp, int32_t rawPressure);

/* Exported functions --------------------------------------------------------*/

void BMP180_init(void) {
	I2Cbar_Init();

    ReadCalibVals(&calibVals);
}

void BMP180_StartPressureMeasure(void) {
	I2Cbar_WriteData(BAROMETER_I2C_ADDRESS, BMP180_CTRL_MEAS_REG, BMP180_P_MEASURE | (oss << 6));
}

void BMP180_StartTemperatureMeasure(void) {
	I2Cbar_WriteData(BAROMETER_I2C_ADDRESS, BMP180_CTRL_MEAS_REG, BMP180_T_MEASURE);
}

HAL_StatusTypeDef BMP180_ReadPressureValue(float * pData) {
    HAL_StatusTypeDef status = 0;
    uint8_t dataRead[3];
    uint32_t rawValue;

    status = I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_ADC_OUT_REG, dataRead, 3);
    rawValue = dataRead[0]<<(8+oss) | dataRead[1]<<(oss) | dataRead[2]>>(8-oss);

    *pData = CalculateRealPreassure(rawTempValue, rawValue);

    return status;
}

HAL_StatusTypeDef BMP180_UpdateInternalTempValue() {
    HAL_StatusTypeDef status = 0;
    uint8_t dataRead[2];

    status = I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_ADC_OUT_REG, dataRead, 2);
    rawTempValue = dataRead[0]<<8 | dataRead[1];

    return status;
}

/* Private functions ---------------------------------------------------------*/

static void ReadCalibVals(BMP180CalibVals_t *calibVals) {
	uint8_t tmpData[22];
	I2Cbar_ReadDataLen(BAROMETER_I2C_ADDRESS, BMP180_CALIB_PARAM_START_REG, tmpData, BMP180_CALIB_PARAM_DATA_LEN);

	calibVals->AC1 = tmpData[0] << 8 | tmpData[1];
	calibVals->AC2 = tmpData[2] << 8 | tmpData[3];
	calibVals->AC3 = tmpData[4] << 8 | tmpData[5];
	calibVals->AC4 = tmpData[6] << 8 | tmpData[7];
	calibVals->AC5 = tmpData[8] << 8 | tmpData[9];
	calibVals->AC6 = tmpData[10] << 8 | tmpData[11];
	calibVals->B1 = tmpData[12] << 8 | tmpData[13];
	calibVals->B2 = tmpData[14] << 8 | tmpData[15];
	calibVals->MB = tmpData[16] << 8 | tmpData[17];
	calibVals->MC = tmpData[18] << 8 | tmpData[19];
	calibVals->MD = tmpData[20] << 8 | tmpData[21];
}

static int32_t CalculateRealPreassure(int32_t rawTemp, int32_t rawPressure) {
	int32_t X1, X2, X3, B3, B6, p;
	uint32_t B4, B7;

	X1 = (rawTemp - calibVals.AC6) * calibVals.AC5 / 32768;
	X2 = calibVals.MC * 2048 / (X1 + calibVals.MD);
	B6 = X1 + X2 - 4000;

	X1 = (calibVals.B2 * (B6 * B6 / 4096)) / 2048;
	X2 = calibVals.AC2 * B6 / 2048;
	X3 = X1 + X2;
	B3 = (((calibVals.AC1 * 4 + X3) << oss) + 2) / 4;

	X1 = calibVals.AC3 * B6 / 8192;
	X2 = (calibVals.B1 * ( B6 * B6 / 4096)) / 65536;
	X3 = ((X1 + X2) + 2) / 4;
	B4 = calibVals.AC4 * (uint32_t)(X3 + 32768) / 32768;
	B7 = ((uint32_t)rawPressure - B3) * (50000 >> oss);
	if (B7 < 0x80000000) {
		p = (B7 * 2) / B4;
	}
	else {
		p = (B7 / B4) * 2;
	}
	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038) / 65536;
	X2 = (-7357 * p) / 65536;
	p = p + (X1 + X2 +3791) / 16;

	return p;
}
