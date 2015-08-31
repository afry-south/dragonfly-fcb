#ifndef FCB_FSM_MSG_H
#define FCB_FSM_MSG_H

/*
 * This file contains message definitions for messages going
 * between FSM and FCB.
 */


/**
 * The first byte of an FCB-FSM message is its ID: enum fcb_fsm_msg
 */
enum { FCB_FSM_MSG_ID_INDEX = 0 };

/**
 * The next byte of a FCB-FSM message is its length, as uint8
 */
enum { FCB_FSM_MSG_LEN_INDEX = 1 };

/**
 * An identifier for messages.
 *
 * @see fcb-fsm.proto file for the actual messages.
 */
typedef enum fcb_fsm_msg {
    FLIGHT_SENSOR_SAMPLES = 0xa0,
    PROXIMITY_SENSOR_SAMPLES = 0xa1,
    QUADCOPTER_CFG = 0xa2,
    RADIO_CONTROL_CFG = 0xa3,
    RADIO_CONTROL_SAMPLES = 0xa4,
    MOTOR_SAMPLES = 0xa5,
    FLIGHT_CTRL_SYS_PARAMS = 0xa6,
    FLIGHT_CMDS = 0xa7,
    CURRENT_FLIGHT_CMD_SRC = 0xa8
} fcb_fsm_msg;

#endif /* FCB_FSM_MSG_H */
