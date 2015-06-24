/******************************************************************************
* @file    fifo_buffer.h
* @author  ÅF Dragonfly
* @version v. 1.0.0
* @date    2015-06-24
* @brief   Header file for handling circular FIFO buffers
******************************************************************************/

#ifndef FIFO_BUFFER_H_
#define FIFO_BUFFER_H_

/* Includes -----------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/
#define BUFFER_SIZE     2048            // Defines the maximum number of bytes the buffer can store

/* Exported types ------------------------------------------------------------*/
typedef struct{
  uint16_t first;                       // Index for first byte inserted in to buffer
  uint16_t last;                        // Index for the next byte to be inserted in to buffer
  uint16_t count;                       // Number of bytes currently stored in buffer
  uint8_t bufferArray[BUFFER_SIZE];     // Buffer storage array with size BUFFER_SIZE
}FIFOBuffer_TypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */
void FIFOBufferInit(volatile FIFOBuffer_TypeDef* buffer);
ErrorStatus BufferPut(volatile FIFOBuffer_TypeDef* buffer, const uint8_t putByte);
ErrorStatus BufferGet(volatile FIFOBuffer_TypeDef* buffer, uint8_t* getByte);
bool BufferIsEmpty(volatile FIFOBuffer_TypeDef* buffer);
bool BufferIsFull(volatile FIFOBuffer_TypeDef* buffer);
void ResetBuffer(volatile FIFOBuffer_TypeDef* buffer);

#endif /* FIFO_BUFFER_H_ */

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
