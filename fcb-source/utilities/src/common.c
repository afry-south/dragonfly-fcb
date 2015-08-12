/******************************************************************************
 * @file    common.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Module contains common functions
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t UserButtonPressed;

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Calculates Cyclic Redundancy Check (CRC) using the STM32 CRC Peripheral
 * @param  dataBuffer : Pointer to data buffer (byte array)
 * @param  dataBufferSize : byte size of dataBuffer
 * @retval CRC value
 */
uint32_t CalculateCRC(const uint8_t* dataBuffer, const uint32_t dataBufferSize) {
	/*##-1- Configure the CRC peripheral #######################################*/
	/* CRC handler declaration */
	CRC_HandleTypeDef CrcHandle;

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
		Error_Handler();
	}

	/*##-2- Compute the CRC of "dataBuffer" ####################################*/
	uint32_t CrcVal = HAL_CRC_Calculate(&CrcHandle, (uint32_t*) dataBuffer,
			dataBufferSize);

	/* Deinitialize the CRC peripheral */
	HAL_CRC_DeInit(&CrcHandle);

	return CrcVal;
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

	return (uint16_t) (tmpInt /= length);
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

/**
 * @brief Toggles the LEDs based on User Button presses
 * @param None
 * @retval None
 */
void ToggleLEDs(void) {
	switch (UserButtonPressed) {
	case 0:
		LEDsOff();
		BSP_LED_On(LED3);
		break;

	case 1:
		LEDsOff();
		BSP_LED_On(LED4);
		break;

	case 2:
		LEDsOff();
		BSP_LED_On(LED5);
		break;

	case 3:
		LEDsOff();
		BSP_LED_On(LED6);
		break;

	case 4:
		LEDsOff();
		BSP_LED_On(LED7);
		break;

	case 5:
		LEDsOff();
		BSP_LED_On(LED8);
		break;

	case 6:
		LEDsOff();
		BSP_LED_On(LED9);
		break;

	case 7:
		LEDsOff();
		BSP_LED_On(LED10);
		break;

	default:
		break;
	}
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
