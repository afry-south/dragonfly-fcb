/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.3 at Mon May 11 16:40:27 2015. */

#ifndef PB_FCB_FSM_PB_H_INCLUDED
#define PB_FCB_FSM_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
/* Struct definitions */
typedef struct _FlightControl {
    bool has_AltitudeCtrl_K;
    float AltitudeCtrl_K;
    bool has_AltitudeCtrl_Ti;
    float AltitudeCtrl_Ti;
    bool has_AltitudeCtrl_Td;
    float AltitudeCtrl_Td;
    bool has_RollPitchCtrl_K;
    float RollPitchCtrl_K;
    bool has_RollPitchCtrl_Ti;
    float RollPitchCtrl_Ti;
    bool has_RollPitchCtrl_Td;
    float RollPitchCtrl_Td;
    bool has_YawCtrl_K;
    float YawCtrl_K;
    bool has_YawCtrl_Ti;
    float YawCtrl_Ti;
    bool has_YawCtrl_Td;
    float YawCtrl_Td;
} FlightControl;

typedef struct _MotorSamples {
    bool has_Motor1Nanos;
    uint32_t Motor1Nanos;
    bool has_Motor2Nanos;
    uint32_t Motor2Nanos;
    bool has_Motor3Nanos;
    uint32_t Motor3Nanos;
    bool has_Motor4Nanos;
    uint32_t Motor4Nanos;
} MotorSamples;

typedef struct _QuadcopterConfig {
    bool has_Mass;
    float Mass;
    bool has_Ixx;
    float Ixx;
    bool has_Iyy;
    float Iyy;
    bool has_Izz;
    float Izz;
} QuadcopterConfig;

typedef struct _RCConfig {
    bool has_throttleMinMicros;
    uint32_t throttleMinMicros;
    bool has_throttleMaxMicros;
    uint32_t throttleMaxMicros;
    bool has_aileronMinMicros;
    uint32_t aileronMinMicros;
    bool has_aileronMaxMicros;
    uint32_t aileronMaxMicros;
    bool has_elevatorMinMicros;
    uint32_t elevatorMinMicros;
    bool has_elevatorMaxMicros;
    uint32_t elevatorMaxMicros;
    bool has_rudderMinMicros;
    uint32_t rudderMinMicros;
    bool has_rudderMaxMicros;
    uint32_t rudderMaxMicros;
    bool has_gearOnMicros;
    uint32_t gearOnMicros;
    bool has_gearOffMicros;
    uint32_t gearOffMicros;
    bool has_auxOnMicros;
    uint32_t auxOnMicros;
    bool has_auxOffMicros;
    uint32_t auxOffMicros;
} RCConfig;

typedef struct _RCSamples {
    bool has_throttleMicros;
    uint32_t throttleMicros;
    bool has_aileronMicros;
    uint32_t aileronMicros;
    bool has_elevatorMicros;
    uint32_t elevatorMicros;
    bool has_rudderMicros;
    uint32_t rudderMicros;
    bool has_gearMicros;
    uint32_t gearMicros;
    bool has_auxMicros;
    uint32_t auxMicros;
} RCSamples;

typedef struct _SensorSamples {
    bool has_gyroRoll;
    float gyroRoll;
    bool has_gyroPitch;
    float gyroPitch;
    bool has_gyroYaw;
    float gyroYaw;
    bool has_accX;
    float accX;
    bool has_accY;
    float accY;
    bool has_accZ;
    float accZ;
    bool has_magX;
    float magX;
    bool has_magY;
    float magY;
    bool has_magZ;
    float magZ;
} SensorSamples;

/* Default values for struct fields */

/* Initializer values for message structs */
#define SensorSamples_init_default               {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define QuadcopterConfig_init_default            {false, 0, false, 0, false, 0, false, 0}
#define RCConfig_init_default                    {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define RCSamples_init_default                   {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define MotorSamples_init_default                {false, 0, false, 0, false, 0, false, 0}
#define FlightControl_init_default               {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define SensorSamples_init_zero                  {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define QuadcopterConfig_init_zero               {false, 0, false, 0, false, 0, false, 0}
#define RCConfig_init_zero                       {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define RCSamples_init_zero                      {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define MotorSamples_init_zero                   {false, 0, false, 0, false, 0, false, 0}
#define FlightControl_init_zero                  {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define FlightControl_AltitudeCtrl_K_tag         1
#define FlightControl_AltitudeCtrl_Ti_tag        2
#define FlightControl_AltitudeCtrl_Td_tag        3
#define FlightControl_RollPitchCtrl_K_tag        4
#define FlightControl_RollPitchCtrl_Ti_tag       5
#define FlightControl_RollPitchCtrl_Td_tag       6
#define FlightControl_YawCtrl_K_tag              7
#define FlightControl_YawCtrl_Ti_tag             8
#define FlightControl_YawCtrl_Td_tag             9
#define MotorSamples_Motor1Nanos_tag             1
#define MotorSamples_Motor2Nanos_tag             2
#define MotorSamples_Motor3Nanos_tag             3
#define MotorSamples_Motor4Nanos_tag             4
#define QuadcopterConfig_Mass_tag                1
#define QuadcopterConfig_Ixx_tag                 2
#define QuadcopterConfig_Iyy_tag                 3
#define QuadcopterConfig_Izz_tag                 4
#define RCConfig_throttleMinMicros_tag           1
#define RCConfig_throttleMaxMicros_tag           2
#define RCConfig_aileronMinMicros_tag            3
#define RCConfig_aileronMaxMicros_tag            4
#define RCConfig_elevatorMinMicros_tag           5
#define RCConfig_elevatorMaxMicros_tag           6
#define RCConfig_rudderMinMicros_tag             7
#define RCConfig_rudderMaxMicros_tag             8
#define RCConfig_gearOnMicros_tag                9
#define RCConfig_gearOffMicros_tag               10
#define RCConfig_auxOnMicros_tag                 11
#define RCConfig_auxOffMicros_tag                12
#define RCSamples_throttleMicros_tag             1
#define RCSamples_aileronMicros_tag              2
#define RCSamples_elevatorMicros_tag             3
#define RCSamples_rudderMicros_tag               4
#define RCSamples_gearMicros_tag                 5
#define RCSamples_auxMicros_tag                  6
#define SensorSamples_gyroRoll_tag               1
#define SensorSamples_gyroPitch_tag              2
#define SensorSamples_gyroYaw_tag                3
#define SensorSamples_accX_tag                   4
#define SensorSamples_accY_tag                   5
#define SensorSamples_accZ_tag                   6
#define SensorSamples_magX_tag                   7
#define SensorSamples_magY_tag                   8
#define SensorSamples_magZ_tag                   9

/* Struct field encoding specification for nanopb */
extern const pb_field_t SensorSamples_fields[10];
extern const pb_field_t QuadcopterConfig_fields[5];
extern const pb_field_t RCConfig_fields[13];
extern const pb_field_t RCSamples_fields[7];
extern const pb_field_t MotorSamples_fields[5];
extern const pb_field_t FlightControl_fields[10];

/* Maximum encoded size of messages (where known) */
#define SensorSamples_size                       45
#define QuadcopterConfig_size                    20
#define RCConfig_size                            72
#define RCSamples_size                           36
#define MotorSamples_size                        24
#define FlightControl_size                       45

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define FCB_FSM_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
