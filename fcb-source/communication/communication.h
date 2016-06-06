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

/* Exported constants --------------------------------------------------------*/

enum ProtoMessageTypeEnum {
	ERR_MSG_ENUM = 0,
    RC_VALUES_MSG_ENUM,
    MOTOR_VALUES_MSG_ENUM,
    SENSOR_SAMPLES_MSG_ENUM,
    FLIGHT_STATE_MSG_ENUM,
    PID_PARAMS_MSG_ENUM,
    REFSIGNALS_MSG_ENUM,
    SIMULATED_STATES_MSG_ENUM,
	CTRLSIGNALS_MSG_ENUM
};

#define	PROTO_HEADER_LEN	7

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

#endif /* COMMUNICATION_H_ */
