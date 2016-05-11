/******************************************************************************
 * @file    flash.h
 * @author  Dragonfly
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
#include "flight_control.h"

/* Exported constants --------------------------------------------------------*/
/* Flash definitions */
#define FLASH_BASE_ADDR                 0x08000000              // Flash base address of STM32
#define FLASH_PAGE_SIZE                 0x800                   // 2048 bytes (STM32F07x and STM32F09x)
#define FLASH_WORD_BYTE_SIZE            0x4                     // 4 byte/32-bit words on STM32
#define FLASH_TOTAL_SIZE                0x40000                 // 256 kb flash on STM32F091
#define FLASH_NBR_OF_PAGES              128                     // Flash pages numbered 0 to 127

/* Settings definitions */
#define FLASH_SETTINGS_START_ADDR       0x08038000                                                      // Make sure this area is not used by linker (see ROM range in .ld file).
#define	FLASH_SETTINGS_SIZE				0x8000
#define FLASH_SETTINGS_START_PAGE       (FLASH_SETTINGS_START_ADDR - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE
#define FLASH_SETTINGS_END_PAGE			(FLASH_SETTINGS_START_ADDR + FLASH_SETTINGS_SIZE - FLASH_BASE_ADDR) / FLASH_PAGE_SIZE
#define FLASH_SETTINGS_BYTE_SIZE        FLASH_TOTAL_SIZE + FLASH_BASE_ADDR  - FLASH_SETTINGS_START_ADDR // 32 kB reserved for data storage
#define FLASH_SETTINGS_PAGE_SIZE        FLASH_SETTINGS_BYTE_SIZE / FLASH_PAGE_SIZE

/* NOTE: When adding settings to pages below (especially when storing multiple types of setting on the same page), make
 * sure that their addresses/data offsets do not overlap so that saving one setting partially (or completely) overwrites
 * something else. Settings should be saved with a CRC value stored in front of it.
 */

/* Receiver calibration values settings */
#define FLASH_RECEIVER_CALIBRATION_PAGE         FLASH_SETTINGS_START_PAGE       // Storage page (must be >= FLASH_SETTINGS_START_ADDR)
#define FLASH_RECEIVER_CALIBRATION_DATA_OFFSET  0                               // Storage byte offset from page base address (has to be word aligned)
#define FLASH_RECEIVER_CALIBRATION_SIZE         sizeof(Receiver_CalibrationValues_TypeDef) + HAL_CRC_LENGTH_32B/4       // Added room for CRC
#define FLASH_RECEIVER_CALIBRATION_END			FLASH_RECEIVER_CALIBRATION_DATA_OFFSET + FLASH_RECEIVER_CALIBRATION_SIZE
/* Max reference signal limits settings */
#define FLASH_REFERENCE_MAX_LIMITS_PAGE         FLASH_SETTINGS_START_PAGE       // Storage page (must be >= FLASH_SETTINGS_START_ADDR)
#define FLASH_REFERENCE_MAX_LIMITS_DATA_OFFSET  FLASH_RECEIVER_CALIBRATION_END  // Storage byte offset from page base address (has to be word aligned)
#define FLASH_REFERENCE_MAX_LIMITS_SIZE         sizeof(RefSignals_TypeDef) + HAL_CRC_LENGTH_32B/4       // Added room for CRC
#define FLASH_REFERENCE_MAX_LIMITS_END          FLASH_REFERENCE_MAX_LIMITS_DATA_OFFSET + FLASH_REFERENCE_MAX_LIMITS_SIZE
/* Magnetometer calibration values */
#define FLASH_MAG_CALIBRATION_PAGE              FLASH_SETTINGS_START_PAGE       // Storage page (must be >= FLASH_SETTINGS_START_ADDR)
#define FLASH_MAG_CALIBRATION_DATA_OFFSET       FLASH_REFERENCE_MAX_LIMITS_END  // Storage byte offset from page base address (has to be word aligned)
#define FLASH_MAG_CALIBRATION_SIZE              sizeof(float32_t) * 6
#define FLASH_MAG_CALIBRATION_END               FLASH_MAG_CALIBRATION_DATA_OFFSET + FLASH_MAG_CALIBRATION_SIZE
/* Accelerometer calibration values */
#define FLASH_ACC_CALIBRATION_PAGE              FLASH_SETTINGS_START_PAGE       // Storage page (must be >= FLASH_SETTINGS_START_ADDR)
#define FLASH_ACC_CALIBRATION_DATA_OFFSET       FLASH_MAG_CALIBRATION_END  // Storage byte offset from page base address (has to be word aligned)
#define FLASH_ACC_CALIBRATION_SIZE              sizeof(float32_t) * 6
#define FLASH_ACC_CALIBRATION_END               FLASH_ACC_CALIBRATION_DATA_OFFSET + FLASH_ACC_CALIBRATION_SIZE

/* Exported types ------------------------------------------------------------*/
typedef enum {
	FLASH_ERROR = 0, FLASH_OK = !FLASH_ERROR
} FlashErrorStatus;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
FlashErrorStatus ReadCalibrationValuesFromFlash(volatile Receiver_CalibrationValues_TypeDef* receiverCalibrationValues);
FlashErrorStatus WriteCalibrationValuesToFlash(const Receiver_CalibrationValues_TypeDef* receiverCalibrationValues);
FlashErrorStatus ReadReferenceMaxLimitsFromFlash(RefSignals_TypeDef* receiverMaxLimits);
FlashErrorStatus WriteReferenceMaxLimitsToFlash( const RefSignals_TypeDef* receiverMaxLimits);
FlashErrorStatus ReadMagCalibrationValuesFromFlash(float32_t magCalibrationValues[6]);
FlashErrorStatus WriteMagCalibrationValuesToFlash(const float32_t magCalibrationValues[6]);
FlashErrorStatus ReadAccCalibrationValuesFromFlash(float32_t accCalibrationValues[6]);
FlashErrorStatus WriteAccCalibrationValuesToFlash(const float32_t accCalibrationValues[6]);

#endif /* __FLASH_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
