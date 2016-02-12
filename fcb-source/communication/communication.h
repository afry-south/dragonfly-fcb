/*****************************************************************************
 * @brief   Common communication header file
 ******************************************************************************/

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

/* Exported typedefs ---------------------------------------------------------*/
typedef enum {
    ARRAY_BUFFER,
    FIFO_BUFFER
} BufferType_TypeDef;

typedef enum {
  NO_SERIALIZATION,
  CALIBRATION_SERIALIZATION,
  PROTOBUFFER_SERIALIZATION
} SerializationType;

typedef struct {
    BufferType_TypeDef bufferType;
    uint16_t dataSize;
    void* bufferPtr;
    xSemaphoreHandle* BufferMutex;
} ComTxQueueItem_TypeDef;

enum ProtoMessageTypeEnum {
    RC_VALUES_MSG_ENUM = 1,
    MOTOR_VALUES_MSG_ENUM,
    SENSOR_SAMPLES_MSG_ENUM,
    FLIGHT_STATE_MSG_ENUM,
    PID_CTRLPARAMS_MSG_ENUM,
    CTRL_REFSIGNALS_MSG_ENUM,
    SIMULATED_STATES_ENUM
};

#endif /* COMMUNICATION_H_ */
