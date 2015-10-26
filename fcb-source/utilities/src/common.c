/******************************************************************************
 * @file    common.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Module contains common functions
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "fcb_error.h"

#include "stm32f3_discovery.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef CrcHandle;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes the Cyclic Redundancy Check peripheral
 * @param  None
 * @retval None
 */
void InitCRC(void)
{
	CrcHandle.Instance = CRC;

	/* The default polynomial is used */
	CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;

	/* The default init value is used */
	CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;

	/* The input data are not inverted */
	CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;

	/* The output data are not inverted */
	CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLED;

	/* The input data has 8 bits length */
	CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

	/* Initialize the CRC peripheral */
	if (HAL_CRC_Init(&CrcHandle) != HAL_OK) {
		/* Initialization Error */
		ErrorHandler();
	}
}

/*
 * @brief  Calculates Cyclic Redundancy Check (CRC) using the STM32 CRC Peripheral
 * @param  dataBuffer : Pointer to data buffer (byte array)
 * @param  dataBufferSize : byte size of dataBuffer
 * @retval CRC value
 */
uint32_t CalculateCRC(const uint8_t* dataBuffer, const uint32_t dataBufferSize) {

	uint32_t crcVal;

	/* Compute the CRC of dataBuffer */
	crcVal = HAL_CRC_Calculate(&CrcHandle, (uint32_t*) dataBuffer, dataBufferSize);

	return crcVal;
}

/**
 * @brief  Configures the Programmable Voltage Detection (PVD) resources.
 * @param  None
 * @retval None
 */
void ConfigPVD(void) {
	PWR_PVDTypeDef sConfigPVD;

	/*##-1- Enable Power Clock #################################################*/
	__PWR_CLK_ENABLE();

	/*##-2- Configure the NVIC for PVD #########################################*/
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_IRQn);

	/* Configure the PVD level to and generate an interrupt on falling
	 edges (Detection level set to 2.47V, refer to the electrical characteristics
	 of the device datasheet for more details) */
	sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;
	sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING;
	HAL_PWR_PVDConfig(&sConfigPVD);

	/* Enable the PVD Output */
	HAL_PWR_EnablePVD();
}

/*
 * @brief  Calculates the mean of the elements in an uint16_t buffer
 * @param  buffer : Pointer to uint16_t buffer
 * @param  length : number of uint16_t elements in buffer over which mean is calculated
 * @retval Mean value
 */
uint16_t UInt16Mean(const uint16_t* buffer, const uint16_t length) {
	uint32_t tmpInt = 0;
	uint16_t i;

	for (i = 0; i < length; i++)
		tmpInt += buffer[i];

	return (uint16_t) (tmpInt / length);
}

/**
 * @brief  Initializes the board's LEDs
 * @param  None
 * @retval None
 */
void InitLEDs(void) {
	/* Initialize LEDs and User Button available on STM32F3-Discovery board */
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);
	BSP_LED_Init(LED7);
	BSP_LED_Init(LED8);
	BSP_LED_Init(LED9);
	BSP_LED_Init(LED10);

	LEDsOff();
}

/**
 * @brief  Turns off all the LEDs
 * @param  None
 * @retval None
 */
void LEDsOff(void) {
	BSP_LED_Off(LED3);
	BSP_LED_Off(LED4);
	BSP_LED_Off(LED5);
	BSP_LED_Off(LED6);
	BSP_LED_Off(LED7);
	BSP_LED_Off(LED8);
	BSP_LED_Off(LED9);
	BSP_LED_Off(LED10);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
