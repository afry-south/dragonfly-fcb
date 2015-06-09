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
static FlashErrorStatus WriteFlashPage(uint32_t * writeData, uint32_t pageNbr);

/* Exported functions --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/* WriteFlashPage
 * @brief Writes one page (2048 bytes on STM32F303VC) of data to the flash
 * @param writeData : pointer to data array, which is to be written to the flash page
 * @param pageNbr : Specifies flash page
 */
static FlashErrorStatus WriteFlashPage(uint32_t * writeData, uint32_t pageNbr)
{
  HAL_StatusTypeDef HALStatus = HAL_OK;

  /* Unlock the Flash Program Erase controller */
  HALStatus = HAL_FLASH_Unlock();

  /* Erase flash page and clear all pending flags before writing to it */
  FLASH_PageErase(FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE);

  /* Program Flash */
  uint32_t Address = FLASH_BASE_ADDR + pageNbr*FLASH_PAGE_SIZE;
  uint32_t i = 0;

  while((Address < FLASH_BASE_ADDR+pageNbr*FLASH_PAGE_SIZE + FLASH_PAGE_SIZE-1) && (HALStatus != HAL_OK) && i < FLASH_PAGE_SIZE/FLASH_WORD_BYTE_SIZE)
    {
      //if(writeData[i] != ReadFlashWord(pageNbr, i)) // To prevent unnecessary writing
      HALStatus = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, writeData[i]); // Word size is 32 bits/4 bytes => 1 page = 2048 bytes = 512 words
      Address += FLASH_WORD_BYTE_SIZE;
      i++;
    }

  HALStatus = HAL_FLASH_Lock();

  if(HALStatus != HAL_OK)
    return FLASH_ERROR;

  return FLASH_OK;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
