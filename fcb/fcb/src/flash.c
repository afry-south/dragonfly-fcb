/******************************************************************************
 * @file    fcb/flash.c
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Module contains flash reading/writing functions
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static FlashErrorStatus WriteFlashPage(uint32_t* writeData, uint16_t pageNbr);
static FlashErrorStatus ReadFlashPage(uint32_t* readData, uint16_t pageNbr);
static uint32_t ReadFlashWord(uint32_t pageNbr, uint32_t wordNbr);

/* Exported functions --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/* WriteFlashPage
 * @brief  Writes one page (2048 bytes on STM32F303VC) of data to the flash
 * @param  writeData : pointer to data array, which is to be written to the flash page
 * @param  pageNbr : Specifies flash page
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus WriteFlashPage(uint32_t* writeData, uint16_t pageNbr)
{
  HAL_StatusTypeDef HALStatus = HAL_OK;

  /* Unlock the Flash Program Erase controller */
  HALStatus = HAL_FLASH_Unlock();

  /* Erase flash page and clear all pending flags before writing to it */
  FLASH_PageErase(FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE);

  /* Program Flash */
  uint32_t address = FLASH_BASE_ADDR + pageNbr*FLASH_PAGE_SIZE;
  uint32_t i = 0;

  while(i < FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE && HALStatus == HAL_OK)
    {
      //if(writeData[i] != ReadFlashWord(pageNbr, i)) // To prevent unnecessary writing
      HALStatus = HAL_FLASH_Program(TYPEPROGRAM_WORD, address, writeData[i]); // Word size is 32 bits/4 bytes => 1 page = 2048 bytes = 512 words
      address += FLASH_WORD_BYTE_SIZE;
      i++;
    }

  HALStatus = HAL_FLASH_Lock();

  if(HALStatus != HAL_OK)
    return FLASH_ERROR;

  return FLASH_OK;
}

/* ReadFlashPage
 * @brief  Reads one page (2048 bytes on STM32F303VC) of flash memory
 * @param  readData : pointer to the uint32_t array where the read data is stored
 * @param  pageNbr : the flash page number from which data is read
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus ReadFlashPage(uint32_t * readData, uint16_t pageNbr)
{
  uint32_t i = 0;
  uint32_t address = FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE;

  if(address < FLASH_BASE_ADDR || address >= FLASH_BASE_ADDR + FLASH_TOTAL_SIZE)
    return FLASH_ERROR;

  while(i < FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE)
    {
      readData[i] = *((uint32_t *) address);
      address += FLASH_WORD_BYTE_SIZE;
      i++;
    }

  return FLASH_OK;
}

/* ReadFlashWord
 * @brief  Reads one word (32 bits/4 bytes) of flash memory from flash
 * @param  pageNbr : flash page number
 * @param  wordNbr : word offset from pageNbr base
 * @retval word value as an unsigned 32-bit integer
 */
static uint32_t ReadFlashWord(uint32_t pageNbr, uint32_t wordNbr)
{
  uint32_t address = FLASH_BASE_ADDR + pageNbr*FLASH_PAGE_SIZE + FLASH_WORD_BYTE_SIZE*wordNbr;
  if(address >= FLASH_BASE_ADDR && address < FLASH_BASE_ADDR + FLASH_TOTAL_SIZE)
    return *((uint32_t *) address);
  else
    return 0xFFFFFFFF;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
