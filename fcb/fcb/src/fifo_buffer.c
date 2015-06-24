/******************************************************************************
* @file    fifo_buffer.c
* @author  ÅF Dragonfly
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
void FIFOBufferInit(volatile FIFOBuffer_TypeDef* buffer)
{
  buffer->count = 0;    // Initialize buffer current size
  buffer->first = 0;    // Initialize first index
  buffer->last = 0;     // Initialize last index
}

/*
 * @brief  Puts a byte of data in to the buffer and updates indices
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 *                 putByte: The byte of data to be inserted in to the buffer
 * @retval SUCCESS if data put in buffer, ERROR if buffer was full
 */
ErrorStatus BufferPut(volatile FIFOBuffer_TypeDef* buffer, const uint8_t putByte)
{
  // Check if buffer is full - if it is, return ERROR
  if(buffer->count == BUFFER_SIZE)
    {
      ResetBuffer(buffer);  // If buffer becomes full, reset it
      return ERROR;
    }

  // Insert new byte at the updated last index
  buffer->bufferArray[buffer->last] = putByte;
  buffer->count++;

  // Update buffer last index
  buffer->last++;
  if(buffer->last >= BUFFER_SIZE)
    buffer->last = 0;

  // Buffer updated - return SUCCESS
  return SUCCESS;
}

/*
 * @brief  Gets the first byte of data from the buffer and updates indices
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @param  getByte : Pointer to the byte where the buffer output is to be stored
 * @retval SUCCESS if data obtained from buffer, ERROR if buffer was empty
 */
ErrorStatus BufferGet(volatile FIFOBuffer_TypeDef* buffer, uint8_t* getByte)
{
  // Check if buffer is empty - if it is, return ERROR
  if(buffer->count == 0)
    return ERROR;

  // Get data from buffer first index (FIFO)
  *getByte = buffer->bufferArray[buffer->first];
  buffer->count--;

  // Update buffer first index
  buffer->first++;
  if(buffer->first >= BUFFER_SIZE)
    buffer->first = 0;

  return SUCCESS;
}

/*
 * @brief  Checks if the buffer is empty
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval TRUE if buffer is empty, FALSE if it is NOT empty
 */
bool BufferIsEmpty(volatile FIFOBuffer_TypeDef* buffer)
{
  if(buffer->count == 0)
    return true;
  return false;
}

/*
 * @brief  Checks if the buffer is full
 * @param  buffer : Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval TRUE if buffer is full, FALSE if it is NOT full
 */
bool BufferIsFull(volatile FIFOBuffer_TypeDef* buffer)
{
  if(buffer->count == BUFFER_SIZE)
    return true;
  return false;
}

/*
 * @brief  Resets the buffer
 * @param  buffer: Pointer to the declared FIFOBuffer_TypeDef buffer
 * @retval None
 */
void ResetBuffer(volatile FIFOBuffer_TypeDef* buffer)
{
  uint16_t i = 0;

  while(i < BUFFER_SIZE)
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
