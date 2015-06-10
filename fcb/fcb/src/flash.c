/******************************************************************************
 * @file    fcb/flash.c
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Module contains flash reading/writing functions
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "common.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static FlashErrorStatus WriteFlashPage(const uint32_t* writeData, const uint16_t pageNbr);
static FlashErrorStatus WriteFlashDataWithCRC(const uint8_t* writeData, const uint32_t writeDataSize, const uint16_t pageNbr, const uint16_t pageOffset);
static FlashErrorStatus ReadFlashPage(uint32_t* readData, const uint16_t pageNbr);
static uint32_t ReadFlashWord(const uint32_t pageNbr, const uint32_t wordNbr);
static FlashErrorStatus ReadFlashBytes(uint8_t * readData, const uint32_t startAddr, const uint32_t nbrOfBytes);
static FlashErrorStatus IsValidFlashAddress(const uint32_t address);

/* Exported functions --------------------------------------------------------*/

FlashErrorStatus ReadCalibrationValuesFromFlash(Receiver_IC_PulseCalibrationValues_TypeDef* receiverCalibrationValues)
{
#ifdef TODO

#endif
  return FLASH_OK;
}

FlashErrorStatus WriteCalibrationValuesToFlash(const Receiver_IC_PulseCalibrationValues_TypeDef* receiverCalibrationValues)
{
  /* Read the page and store it in tmpPage */
  uint32_t tmpPage[FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE];  // Div by 4 to get word size
  memset(&tmpPage, 0xFF, sizeof(tmpPage));
  ReadFlashPage(&tmpPage[0], FLASH_RECEIVER_CALIBRATION_PAGE);

  /* Copy data to tmpPage at offset+1 location (CRC stored at first index) */
  memcpy(&tmpPage[FLASH_RECEIVER_CALIBRATION_DATA_OFFSET+1], receiverCalibrationValues, sizeof(Receiver_IC_PulseCalibrationValues_TypeDef));

  /* Take the CRC of tmp page, except for the first index that is reserved for the CRC itself */
  tmpPage[FLASH_RECEIVER_CALIBRATION_DATA_OFFSET] = Calculate_CRC((uint8_t*)receiverCalibrationValues, sizeof(Receiver_IC_PulseCalibrationValues_TypeDef));
  WriteFlashPage(&tmpPage[0], FLASH_RECEIVER_CALIBRATION_PAGE); // Write config to the config page

  return FLASH_OK;
}

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Writes one page (2048 bytes on STM32F303VC) of data to the flash
 * @param  writeData : pointer to data array, which is to be written to the flash page
 * @param  pageNbr : Specifies flash page
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus WriteFlashPage(const uint32_t* writeData, const uint16_t pageNbr)
{
  HAL_StatusTypeDef HALStatus = HAL_OK;

  /* Unlock the Flash Program Erase controller */
  HALStatus = HAL_FLASH_Unlock();

  /* Erase flash page and clear all pending flags before writing to it */
  FLASH_PageErase(FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE);

  /* Program Flash */
  uint32_t address = FLASH_BASE_ADDR + pageNbr*FLASH_PAGE_SIZE;
  uint32_t i = 0;

  while(i < FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE && HALStatus == HAL_OK && IsValidFlashAddress(address))
    {
      //if(writeData[i] != ReadFlashWord(pageNbr, i)) // To prevent unnecessary writing
      HALStatus = HAL_FLASH_Program(TYPEPROGRAM_WORD, address, writeData[i]); // Word size is 32 bits/4 bytes => 1 page = 2048 bytes = 512 words
      address += FLASH_WORD_BYTE_SIZE;
      i++;
    }

  if(HALStatus != HAL_OK)
    {
      HAL_FLASH_Lock();
      return FLASH_ERROR;
    }

  HALStatus = HAL_FLASH_Lock();

  if(HALStatus != HAL_OK)
    return FLASH_ERROR;

  return FLASH_OK;
}

/*
 * @brief  Reads one page (2048 bytes on STM32F303VC) of flash memory
 * @param  readData : pointer to the uint32_t array where the read data is stored
 * @param  pageNbr : the flash page number from which data is read
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus ReadFlashPage(uint32_t * readData, const uint16_t pageNbr)
{
  uint32_t i = 0;
  uint32_t address = FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE;

  if(!IsValidFlashAddress(address))
    return FLASH_ERROR;

  while(i < FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE)
    {
      readData[i] = *((uint32_t *) address);
      address += FLASH_WORD_BYTE_SIZE;
      i++;
    }

  return FLASH_OK;
}

/*
 * @brief  Reads one word (32 bits/4 bytes) of flash memory from flash
 * @param  pageNbr : flash page number
 * @param  wordNbr : word offset from pageNbr base
 * @retval word value as an unsigned 32-bit integer
 */
static uint32_t ReadFlashWord(const uint32_t pageNbr, const uint32_t wordNbr)
{
  uint32_t address = FLASH_BASE_ADDR + pageNbr*FLASH_PAGE_SIZE + FLASH_WORD_BYTE_SIZE*wordNbr;
  if(IsValidFlashAddress(address))
    return *((uint32_t *) address);
  else
    return 0xFFFFFFFF;
}

/*
 * @brief  Reads a specifiable amount of bytes from the flash memory starting from startAddr
 * @param  readData : pointer to the byte array where the read bytes are stored
 * @param  startAddr : start read address in flash
 * @param  nbrOfBytes : Number of bytes to be read
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus ReadFlashBytes(uint8_t * readData, const uint32_t startAddr, const uint32_t nbrOfBytes)
{
  uint32_t i = 0;
  uint32_t address = startAddr;

  while(i < nbrOfBytes && IsValidFlashAddress(address))
    {
      readData[i] = *((uint8_t *) address);
      address += 1;
      i++;
    }

  return FLASH_OK;
}

/*
 * @brief  Indicates if an entered address is a valid flash address
 * @param  address : The flash address
 * @retval FLASH_OK if flash address is within valid flash address range, else FLASH_ERROR
 */
static FlashErrorStatus IsValidFlashAddress(const uint32_t address)
{
  return (address >= FLASH_BASE_ADDR && address < FLASH_BASE_ADDR + FLASH_TOTAL_SIZE);
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
