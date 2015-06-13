/******************************************************************************
 * @file    fcb/common.c
 * @author  ÅF Dragonfly
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
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Calculates Cyclic Redundancy Check (CRC) using the STM32 CRC Peripheral
 * @param  dataBuffer : Pointer to data buffer (byte array)
 * @param  dataBufferSize : byte size of dataBuffer
 * @retval CRC value
 */
uint32_t Calculate_CRC(const uint8_t* dataBuffer, const uint32_t dataBufferSize)
{
  /*##-1- Configure the CRC peripheral #######################################*/
  /* CRC handler declaration */
  CRC_HandleTypeDef CrcHandle;

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
  if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Compute the CRC of "dataBuffer" ####################################*/
  uint32_t CrcVal = HAL_CRC_Calculate(&CrcHandle, (uint32_t*)dataBuffer, dataBufferSize);

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
uint16_t UInt16_Mean(const uint16_t* buffer, const uint16_t length)
{
  uint32_t tmpInt;
  uint16_t i;

  for(i = 0; i < length; i++)
    tmpInt += buffer[i];

  return (uint16_t)(tmpInt /= length);
}

/* Private functions ---------------------------------------------------------*/


/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
