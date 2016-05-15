/******************************************************************************
 * @brief   Module contains flash reading/writing functions
 *
 * @license
 * Dragonfly FCB firmware to control the Dragonfly quadrotor UAV
 * Copyright (C) 2016  ÅF Technology South: Dragonfly Project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flash.h"
#include "common.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define IS_VALID_FLASH_ADDR(ADDR)	(((ADDR) >= FLASH_BASE_ADDR) && ((ADDR) < (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE)))

#define IS_VALID_SETTINGS_PAGE(PAGE)	(((PAGE) >= FLASH_SETTINGS_START_PAGE) && ((PAGE) <= FLASH_SETTINGS_END_PAGE))

#define IS_VALID_PAGE_OFFSET_SIZE(OFFSET,SIZE)	(((OFFSET) + (SIZE)) <= FLASH_PAGE_SIZE)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static FlashErrorStatus WriteSettingsToFlash(const uint8_t* writeSettingsData, const uint16_t writeSettingsDataSize,
		const uint8_t settingsPageNbr, const uint16_t settingsPageOffset);
static FlashErrorStatus ReadSettingsFromFlash(uint8_t* readSettingsData, const uint16_t readSettingsDataSize,
		const uint8_t settingsPageNbr, const uint16_t settingsPageOffset);

static FlashErrorStatus WriteFlashPage(const uint32_t* writeData,
		const uint8_t pageNbr);
static FlashErrorStatus ReadFlashPage(uint8_t * readData, const uint8_t pageNbr);
static uint32_t ReadFlashWord(const uint8_t pageNbr, const uint16_t pageOffset);
static FlashErrorStatus ReadFlashBytes(uint8_t * readData, const uint32_t startAddr, const uint32_t nbrOfBytes);

static uint32_t GetFlashPageOffsetAddress(const uint8_t pageNbr, const uint16_t pageOffset);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Reads previously stored receiver calibration values from flash memory
 * @param  receiverCalibrationValues : Pointer to receiver calibration values struct to which values will enter
 * @retval FLASH_OK if calibration values read succesfully from flash, else FLASH_ERROR
 */
FlashErrorStatus ReadCalibrationValuesFromFlash(volatile Receiver_CalibrationValues_TypeDef* receiverCalibrationValues) {
	FlashErrorStatus status = FLASH_OK;

	/* Read receiver calibration settings from flash, if valid data exists */
	status = ReadSettingsFromFlash((uint8_t*) receiverCalibrationValues, sizeof(Receiver_CalibrationValues_TypeDef),
			FLASH_RECEIVER_CALIBRATION_PAGE, FLASH_RECEIVER_CALIBRATION_DATA_OFFSET);

	return status;
}

/*
 * @brief  Writes the receiver calibration values to flash memory for persistent storage
 * @param  receiverCalibrationValues : Pointer to receiver calibration values struct to be saved
 * @retval FLASH_OK if calibration values written succesfully to flash, else FLASH_ERROR
 */
FlashErrorStatus WriteCalibrationValuesToFlash( const Receiver_CalibrationValues_TypeDef* receiverCalibrationValues) {
	FlashErrorStatus status = FLASH_OK;

	/* Write receiver calibration settings to flash */
	status = WriteSettingsToFlash((uint8_t*) receiverCalibrationValues, sizeof(Receiver_CalibrationValues_TypeDef),
			FLASH_RECEIVER_CALIBRATION_PAGE, FLASH_RECEIVER_CALIBRATION_DATA_OFFSET);

	return status;
}

/*
 * @brief  Reads previously stored reference max limits from flash memory
 * @param  referenceMaxLimits : Pointer to reference max limits struct to which values will enter
 * @retval FLASH_OK if values read succesfully from flash, else FLASH_ERROR
 */
FlashErrorStatus ReadReferenceMaxLimitsFromFlash(RefSignals_TypeDef* referenceMaxLimits) {
	FlashErrorStatus status = FLASH_OK;

	/* Read reference calibration settings from flash, if valid data exists */
	status = ReadSettingsFromFlash((uint8_t*) referenceMaxLimits, sizeof(RefSignals_TypeDef),
			FLASH_REFERENCE_MAX_LIMITS_PAGE, FLASH_REFERENCE_MAX_LIMITS_DATA_OFFSET);

	return status;
}

/*
 * @brief  Writes the reference max limits to flash memory for persistent storage
 * @param  referenceMaxLimits : Pointer to reference max limits struct to be saved
 * @retval FLASH_OK if calibration values written succesfully to flash, else FLASH_ERROR
 */
FlashErrorStatus WriteReferenceMaxLimitsToFlash( const RefSignals_TypeDef* referenceMaxLimits) {
	FlashErrorStatus status = FLASH_OK;

	/* Write reference calibration settings to flash */
	status = WriteSettingsToFlash((uint8_t*) referenceMaxLimits, sizeof(RefSignals_TypeDef),
			FLASH_REFERENCE_MAX_LIMITS_PAGE, FLASH_REFERENCE_MAX_LIMITS_DATA_OFFSET);

	return status;
}

/*
 * @brief  Reads previously stored magnetometer calibration values from flash memory
 * @param  magCalibrationValues : Pointer to magnetometer calibration values struct to which values will enter
 * @retval FLASH_OK if calibration values read succesfully from flash, else FLASH_ERROR
 */
FlashErrorStatus ReadMagCalibrationValuesFromFlash(float32_t magCalibrationValues[6]) {
	FlashErrorStatus status = FLASH_OK;

	/* Read receiver calibration settings from flash, if valid data exists */
	status = ReadSettingsFromFlash((uint8_t*) magCalibrationValues, FLASH_MAG_CALIBRATION_SIZE,
			FLASH_MAG_CALIBRATION_PAGE, FLASH_MAG_CALIBRATION_DATA_OFFSET);

	return status;
}

/*
 * @brief  Writes the magnetometer calibration values to flash memory for persistent storage
 * @param  magCalibrationValues : Pointer to magnetometer calibration values struct to be saved
 * @retval FLASH_OK if calibration values written succesfully to flash, else FLASH_ERROR
 */
FlashErrorStatus WriteMagCalibrationValuesToFlash(const float32_t magCalibrationValues[6]) {
	FlashErrorStatus status = FLASH_OK;

	/* Write receiver calibration settings to flash */
	status = WriteSettingsToFlash((uint8_t*) magCalibrationValues, FLASH_MAG_CALIBRATION_SIZE,
			FLASH_MAG_CALIBRATION_PAGE, FLASH_MAG_CALIBRATION_DATA_OFFSET);

	return status;
}

/*
 * @brief  Reads previously stored accelerometer calibration values from flash memory
 * @param  accCalibrationValues : Pointer to accelerometer calibration values to which values will enter
 * @retval FLASH_OK if calibration values read succesfully from flash, else FLASH_ERROR
 */
FlashErrorStatus ReadAccCalibrationValuesFromFlash(float32_t accCalibrationValues[6]) {
	FlashErrorStatus status = FLASH_OK;

	/* Read receiver calibration settings from flash, if valid data exists */
	status = ReadSettingsFromFlash((uint8_t*) accCalibrationValues, FLASH_ACC_CALIBRATION_SIZE,
			FLASH_ACC_CALIBRATION_PAGE, FLASH_ACC_CALIBRATION_DATA_OFFSET);

	return status;
}

/*
 * @brief  Writes the accelerometer calibration values to flash memory for persistent storage
 * @param  accCalibrationValues : Pointer to accelerometer calibration values to be saved
 * @retval FLASH_OK if calibration values written succesfully to flash, else FLASH_ERROR
 */
FlashErrorStatus WriteAccCalibrationValuesToFlash(const float32_t accCalibrationValues[6]) {
	FlashErrorStatus status = FLASH_OK;

	/* Write receiver calibration settings to flash */
	status = WriteSettingsToFlash((uint8_t*) accCalibrationValues, FLASH_ACC_CALIBRATION_SIZE,
			FLASH_ACC_CALIBRATION_PAGE, FLASH_ACC_CALIBRATION_DATA_OFFSET);

	return status;
}

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Writes the settings to flash memory for persistent storage while also adding a CRC value for the stored data
 * @param  writeSettingsData : uint8_t pointer to settings to be saved
 * @param  writeSettingsDataSize : writeSettingsData byte size
 * @retval FLASH_OK if settings written succesfully to flash, else FLASH_ERROR
 */
static FlashErrorStatus WriteSettingsToFlash(const uint8_t* writeSettingsData, const uint16_t writeSettingsDataSize,
		const uint8_t settingsPageNbr, const uint16_t settingsPageOffset) {
	/* Check so that page is valid and that there is enough space on page to store the settings together with CRC*/

	if (!IS_VALID_SETTINGS_PAGE(settingsPageNbr)
			|| !IS_VALID_PAGE_OFFSET_SIZE(settingsPageOffset, writeSettingsDataSize + FLASH_WORD_BYTE_SIZE))
		return FLASH_ERROR;

	/* Read the whole page and store it in tmpPage - required since when writing a page, its entire contents must first be erased */
	static uint8_t tmpPage[FLASH_PAGE_SIZE]; // Declared as static so stack/RTOS stack is not loaded with this
	memset(tmpPage, 0x00, sizeof(tmpPage));
	if (!ReadFlashPage(tmpPage, settingsPageNbr))
		return FLASH_ERROR;

	/* Copy data to tmpPage at offset+1 location (CRC stored at first index) */
	memcpy(&tmpPage[settingsPageOffset + FLASH_WORD_BYTE_SIZE], writeSettingsData,
			writeSettingsDataSize);

	/* Take the CRC of the data to be inserted into flash storage, except for the first index which is reserved for the CRC itself */
	uint32_t crcValue = CalculateCRC(&tmpPage[settingsPageOffset + FLASH_WORD_BYTE_SIZE],
			writeSettingsDataSize);
	memcpy(&tmpPage[settingsPageOffset], (uint8_t*) &crcValue, FLASH_WORD_BYTE_SIZE);
	if (!WriteFlashPage((uint32_t*) tmpPage, settingsPageNbr))
		return FLASH_ERROR;

	return FLASH_OK;
}

/*
 * @brief  Reads previously stored receiver calibration values from flash memory
 * @param  receiverCalibrationValues : Pointer to receiver calibration values struct to which values will enter
 * @retval FLASH_OK if calibration values read succesfully from flash, else FLASH_ERROR
 */
static FlashErrorStatus ReadSettingsFromFlash(uint8_t* readSettingsData, const uint16_t readSettingsDataSize,
		const uint8_t settingsPageNbr, const uint16_t settingsPageOffset) {

	uint8_t tmpSettings[readSettingsDataSize];
	uint32_t CRC_Calculated;

	/* Check so that page is valid and that space after offset is large enough on page to store the settings together with CRC */
	if (!IS_VALID_SETTINGS_PAGE(settingsPageNbr)
			|| !IS_VALID_PAGE_OFFSET_SIZE(settingsPageOffset, readSettingsDataSize + FLASH_WORD_BYTE_SIZE))
		return FLASH_ERROR;

	/* First, get the stored CRC, stored at first offset index */
	uint32_t CRC_Stored = ReadFlashWord(settingsPageNbr, settingsPageOffset);

	/* Read the stored settings data and store it in tmpSettings, added word size to the offset to skip pass the stored CRC */
	if (!ReadFlashBytes(tmpSettings,
			GetFlashPageOffsetAddress(settingsPageNbr, settingsPageOffset + FLASH_WORD_BYTE_SIZE),
			readSettingsDataSize))
		return FLASH_ERROR;

	/* Calculate CRC of loaded settings bytes */
	CRC_Calculated = CalculateCRC(tmpSettings, readSettingsDataSize);

	/* Check data integrity by comparing the calculated CRC with the one stored when setting were saved */
	if (CRC_Calculated == CRC_Stored) {
		memcpy(readSettingsData, tmpSettings, readSettingsDataSize);
		return FLASH_OK;
	}

	/* No valid settings values were loaded, perhaps settings has not been stored yet */
	return FLASH_ERROR;
}

/*
 * @brief  Writes one page (2048 bytes on STM32F303VC) of data to the flash
 * @param  writeData : pointer to data array, which is to be written to the flash page
 * @param  pageNbr : Specifies flash page
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus WriteFlashPage(const uint32_t* writeData, const uint8_t pageNbr) {
	HAL_StatusTypeDef HALStatus = HAL_OK;

	/* Unlock the Flash Program Erase controller */
	HALStatus = HAL_FLASH_Unlock();

	/* Erase flash page and clear all pending flags before writing to it */
	FLASH_PageErase(FLASH_BASE_ADDR + pageNbr * FLASH_PAGE_SIZE);

	CLEAR_BIT(FLASH->CR, FLASH_CR_PER); // Needed for flash writing to work (set when page is erased)

	/* Program Flash */
	uint32_t address = FLASH_BASE_ADDR + pageNbr * FLASH_PAGE_SIZE;
	uint32_t i = 0;

	while (i < FLASH_PAGE_SIZE / FLASH_WORD_BYTE_SIZE && HALStatus == HAL_OK && IS_VALID_FLASH_ADDR(address)) {
		if (writeData[i] != ReadFlashWord(pageNbr, i * FLASH_WORD_BYTE_SIZE)) // To prevent unnecessary writing (if byte values should == 0xFF anyway)
			HALStatus = HAL_FLASH_Program(TYPEPROGRAM_WORD, address, writeData[i]); // Word size is 32 bits/4 bytes => 1 page = 2048 bytes = 512 words

		address += FLASH_WORD_BYTE_SIZE;
		i++;
	}

	if (HALStatus != HAL_OK) {
		HAL_FLASH_Lock();
		return FLASH_ERROR;
	}

	HALStatus = HAL_FLASH_Lock();

	if (HALStatus != HAL_OK)
		return FLASH_ERROR;

	return FLASH_OK;
}

/*
 * @brief  Reads one page (2048 bytes on STM32F303VC) of flash memory
 * @param  readData : pointer to the byte array where the read data is stored
 * @param  pageNbr : the flash page number from which data is read
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus ReadFlashPage(uint8_t * readData, const uint8_t pageNbr) {
	uint32_t i = 0;
	uint32_t address = FLASH_BASE_ADDR + pageNbr * FLASH_PAGE_SIZE;

	if (!IS_VALID_FLASH_ADDR(address))
		return FLASH_ERROR;

	while (i < FLASH_PAGE_SIZE) {
		readData[i] = *((uint8_t *) address);
		address++;
		i++;
	}

	return FLASH_OK;
}

/*
 * @brief  Reads one word (32 bits/4 bytes) of flash memory from flash
 * @param  pageNbr : flash page number
 * @param  pageOffset : offset from pageNbr base (in bytes)
 * @retval word value as an unsigned 32-bit integer
 */
static uint32_t ReadFlashWord(const uint8_t pageNbr, const uint16_t pageOffset) {
	uint32_t address = FLASH_BASE_ADDR + pageNbr * FLASH_PAGE_SIZE + pageOffset;

	/* Check flash address validity */
	if (!IS_VALID_FLASH_ADDR(address))
		return 0xFFFFFFFF;

	return *((uint32_t *) address);
}

/*
 * @brief  Reads a specifiable amount of bytes from the flash memory starting from startAddr
 * @param  readData : pointer to the byte array where the read bytes are stored
 * @param  startAddr : start read address in flash
 * @param  nbrOfBytes : Number of bytes to be read
 * @retval FLASH_OK if flash operation successful, else FLASH_ERROR
 */
static FlashErrorStatus ReadFlashBytes(uint8_t * readData, const uint32_t startAddr, const uint32_t nbrOfBytes) {
	uint32_t i = 0;
	uint32_t address = startAddr;

	while (i < nbrOfBytes && IS_VALID_FLASH_ADDR(address)) {
		readData[i] = *((uint8_t *) address);
		address++;
		i++;
	}

	if (i < nbrOfBytes - 1)
		return FLASH_ERROR;

	return FLASH_OK;
}

/*
 * @brief  Calculates the flash address corresponding to input page and offset from page base
 * @param  pageNbr : page number
 * @param  pageOffset : byte offset from page base
 * @retval Flash address
 */
static uint32_t GetFlashPageOffsetAddress(const uint8_t pageNbr, const uint16_t pageOffset) {
	return FLASH_BASE_ADDR + pageNbr * FLASH_PAGE_SIZE + pageOffset;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
