/******************************************************************************
* @file    fifo_buffer.h
* @author  ÅF Dragonfly
* @version v. 1.0.0
* @date    2015-06-24
* @brief   Header file for handling circular FIFO buffers
******************************************************************************/

#ifndef __FIFO_BUFFER_H
#define __FIFO_BUFFER_H

/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct{
  uint8_t* bufferArray; // Pointer to buffer storage array
  uint16_t bufferSize;  // Buffer storage array size
  uint16_t first;       // Index for first byte inserted in to buffer
  uint16_t last;                        // Index for the next byte to be inserted in to buffer
  uint16_t count;                       // Number of bytes currently stored in buffer
}FIFOBuffer_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void FIFOBufferInit(volatile FIFOBuffer_TypeDef* buffer, uint8_t* bufferDataArray, const uint16_t bufferDataArraySize);
ErrorStatus BufferPutByte(volatile FIFOBuffer_TypeDef* buffer, const uint8_t putByte);
ErrorStatus BufferPutData(volatile FIFOBuffer_TypeDef* buffer, const uint8_t* putDataPtr, const uint16_t putDataSize);
ErrorStatus BufferGetByte(volatile FIFOBuffer_TypeDef* buffer, uint8_t* getByte);
uint16_t BufferGetData(volatile FIFOBuffer_TypeDef* buffer, uint8_t** getDataPtr, const uint16_t getDataSize);
void BufferDeleteLastEnteredBytes(volatile FIFOBuffer_TypeDef* buffer, const uint16_t dataSize);
bool BufferIsEmpty(volatile FIFOBuffer_TypeDef* buffer);
bool BufferIsFull(volatile FIFOBuffer_TypeDef* buffer);
uint16_t BufferGetAvailableDataSize(volatile FIFOBuffer_TypeDef* buffer);
void ResetBuffer(volatile FIFOBuffer_TypeDef* buffer);

#endif /* __FIFO_BUFFER_H */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
