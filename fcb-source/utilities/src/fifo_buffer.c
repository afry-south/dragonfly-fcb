/******************************************************************************
 * @file    fifo_buffer.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-06-24
 * @brief   Functions for handling circular FIFO buffers
 ******************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "fifo_buffer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes the circular FIFO buffer
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval None
 */
void FIFOBufferInit(volatile FIFOBuffer_TypeDef* buffer, uint8_t* bufferDataArray, uint16_t bufferDataArraySize) {
	buffer->bufferArray = bufferDataArray; // Pointer to data array used as the FIFO buffer
	buffer->bufferSize = bufferDataArraySize;     // Set size of data array
	buffer->count = 0;    // Initialize buffer current size count
	buffer->first = 0;    // Initialize first index
	buffer->last = 0;     // Initialize last index
}

/*
 * @brief  Puts a byte of data in to the buffer and updates indices
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 *                 putByte: The byte of data to be inserted in to the buffer
 * @retval SUCCESS if data put in buffer, ERROR if buffer was full
 */
ErrorStatus FIFOBufferPutByte(volatile FIFOBuffer_TypeDef* buffer, const uint8_t putByte) {
	// Check if buffer is full - if it is, return ERROR
	if (buffer->count == buffer->bufferSize) {
		return ERROR;
	}

	// Insert new byte at the updated last index
	buffer->bufferArray[buffer->last] = putByte;
	buffer->count++;

	// Update buffer last index
	buffer->last++;
	if (buffer->last >= buffer->bufferSize)
		buffer->last = 0;

	// Buffer updated - return SUCCESS
	return SUCCESS;
}

/*
 * @brief  Puts an array of data with a specified size in to the buffer and updates FIFO buffer indices
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @param  putDataPtr : Pointer to the data which is to be copied in to the FIFO buffer
 * @param  putDataSize : The size of the data which is to be copied in to the FIFO buffer
 * @retval SUCCESS if data put in buffer, ERROR if not enough empty space in buffer
 */
ErrorStatus FIFOBufferPutData(volatile FIFOBuffer_TypeDef* buffer, const uint8_t* putDataPtr,
		const uint16_t putDataSize) {
	uint16_t spaceLeftBeforeWraparound;

	// Check if there is enough unused space in buffer to store the data
	if (FIFOBufferGetAvailableDataSize(buffer) < putDataSize) {
		FIFOResetBuffer(buffer);  // If buffer becomes full, reset it
		return ERROR;
	}

	// Copy new data to the buffer last index location - take FIFO buffer wrap-around in to account
	spaceLeftBeforeWraparound = buffer->bufferSize - buffer->last;
	if (putDataSize <= spaceLeftBeforeWraparound) {
		memcpy(&buffer->bufferArray[buffer->last], &putDataPtr[0], putDataSize);
	} else {
		memcpy(&buffer->bufferArray[buffer->last], &putDataPtr[0],
				spaceLeftBeforeWraparound);
		memcpy(&buffer->bufferArray[0], &putDataPtr[spaceLeftBeforeWraparound],
				putDataSize - spaceLeftBeforeWraparound);
	}

	// Update buffer last index
	buffer->last = (buffer->last + putDataSize) % buffer->bufferSize;
	buffer->count += putDataSize;

	// Buffer updated - return SUCCESS
	return SUCCESS;
}

/*
 * @brief  Gets the first byte of data from the buffer and updates indices
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @param  getByte : Pointer to the byte where the buffer output is to be stored
 * @retval SUCCESS if data obtained from buffer, ERROR if buffer was empty
 */
ErrorStatus FIFOBufferGetByte(volatile FIFOBuffer_TypeDef* buffer, uint8_t* getByte) {
	// Check if buffer is empty - if it is, return ERROR
	if (buffer->count == 0)
		return ERROR;

	// Get data from buffer first index (FIFO)
	*getByte = buffer->bufferArray[buffer->first];
	buffer->count--;

	// Update buffer first index
	buffer->first++;
	if (buffer->first >= buffer->bufferSize)
		buffer->first = 0;

	return SUCCESS;
}

/*
 * @brief  Gets an array of data with a specified size in to the buffer and updates FIFO buffer indices.
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @param  getDataPtr : Pointer to the requested data stored in the FIFO buffer
 * @param  getDataSize : Requested data size
 * @retval Amount of data returned. May be less than requested if FIFO buffer wraps around. If so, this function
 *         should be called again (with the size difference) to receive a pointer to the rest of the requested data.
 */
uint16_t FIFOBufferGetData(volatile FIFOBuffer_TypeDef* buffer,
		uint8_t** getDataPtr, const uint16_t getDataSize) {
	uint16_t dataSize;
	uint16_t spaceLeftBeforeWraparound;

	// Check if amount of requested data in buffer is stored
	if (buffer->count < getDataSize) {
		return 0;
	}

	// Copy new data to the buffer last index location - take FIFO buffer wrap-around in to account
	spaceLeftBeforeWraparound = buffer->bufferSize - buffer->first;
	if (getDataSize <= spaceLeftBeforeWraparound)
		dataSize = getDataSize;
	else
		dataSize = spaceLeftBeforeWraparound;

	// Set the pointer to first index
	*getDataPtr = &buffer->bufferArray[buffer->first];

	// Update buffer last index
	buffer->first = (buffer->first + dataSize) % buffer->bufferSize;
	buffer->count -= dataSize;

	return dataSize;
}

/*
 * @brief  Deletes a specified amount of the last entered data bytes from FIFO buffer
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @param  dataSize : Amount of bytes to delete
 * @retval None
 */
void FIFOBufferDeleteLastEnteredBytes(volatile FIFOBuffer_TypeDef* buffer, const uint16_t dataSize) {
	if (buffer->count == dataSize) {
		buffer->count = 0;
		buffer->last = buffer->first;
	}

	if (buffer->last >= dataSize) {
		buffer->last = buffer->last - dataSize;
	} else {
		buffer->last += buffer->bufferSize - dataSize;
	}

	buffer->count -= dataSize;
}

/*
 * @brief  Checks if the buffer is empty
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval TRUE if buffer is empty, FALSE if it is NOT empty
 */
bool FIFOBufferIsEmpty(volatile FIFOBuffer_TypeDef* buffer) {
	if (buffer->count == 0)
		return true;
	return false;
}

/*
 * @brief  Checks if the buffer is full
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval TRUE if buffer is full, FALSE if it is NOT full
 */
bool FIFOBufferIsFull(volatile FIFOBuffer_TypeDef* buffer) {
	if (buffer->count == buffer->bufferSize)
		return true;
	return false;
}

/*
 * @brief  Calculates and returns the amount of free space in the FIFO buffer
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval free size in FIFO buffer
 */
uint16_t FIFOBufferGetAvailableDataSize(volatile FIFOBuffer_TypeDef* buffer) {
	return buffer->bufferSize - buffer->count;
}

/*
 * @brief  Resets the buffer
 * @param  buffer: Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval None
 */
void FIFOResetBuffer(volatile FIFOBuffer_TypeDef* buffer) {
	uint16_t i = 0;

	while (i < buffer->bufferSize)
		buffer->bufferArray[i++] = '\0';

	buffer->first = 0;
	buffer->last = 0;
	buffer->count = 0;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
