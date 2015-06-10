/******************************************************************************
 * @file    fcb/flash.h
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-09
 * @brief   Header file for flash reading/writing functions
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"
#include "receiver.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum
{
  FLASH_ERROR = 0,
  FLASH_OK = !FLASH_ERROR
} FlashErrorStatus;

/* Exported macro ------------------------------------------------------------*/

/* Flash definitions */
#define FLASH_BASE_ADDR                 0x08000000              // Flash base address of STM32
#define FLASH_PAGE_SIZE                 0x800                   // 2048 bytes (STM32F07x and STM32F09x)
#define FLASH_WORD_BYTE_SIZE            0x4                     // 4 byte/32-bit words on STM32
#define FLASH_TOTAL_SIZE                0x40000                 // 256 kb flash on STM32F091
#define FLASH_NBR_OF_PAGES              128                     // Flash pages numbered 0 to 127

/* Settings definitions */
#define FLASH_SETTINGS_START_ADDR       0x08038000                                                      // Make sure this area is not used by linker (see ROM range in .ld file).
#define FLASH_SETTINGS_START_PAGE       (FLASH_SETTINGS_START_ADDR - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE
#define FLASH_SETTINGS_BYTE_SIZE        FLASH_TOTAL_SIZE + FLASH_BASE_ADDR  - FLASH_SETTINGS_START_ADDR // 32 kB reserved for data storage
#define FLASH_SETTINGS_PAGE_SIZE        FLASH_SETTINGS_BYTE_SIZE / FLASH_PAGE_SIZE

/* NOTE: When adding settings to pages below (especially when storing multiple types of setting on the same page), make
 * sure that their addresses/data offsets do not overlap so that saving one setting partially (or completely) overwrites
 * something else. Settings should be saved with a CRC value stored in front of it.
 */

/* Receiver calibration values settings */
#define FLASH_RECEIVER_CALIBRATION_PAGE         FLASH_SETTINGS_START_PAGE       // Storage page (must be >= FLASH_SETTINGS_START_ADDR)
#define FLASH_RECEIVER_CALIBRATION_DATA_OFFSET  0                               // Storage byte offset from page base address (has to be word aligned)
#define FLASH_RECEIVER_CALIBRATION_SIZE         sizeof(Receiver_IC_PulseCalibrationValues_TypeDef) + HAL_CRC_LENGTH_32B/4       // Added room for CRC

/* Exported function prototypes --------------------------------------------- */
FlashErrorStatus ReadCalibrationValuesFromFlash(Receiver_IC_PulseCalibrationValues_TypeDef* receiverCalibrationValues);
FlashErrorStatus WriteCalibrationValuesToFlash(const Receiver_IC_PulseCalibrationValues_TypeDef* receiverCalibrationValues);

#endif /* __FLASH_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
