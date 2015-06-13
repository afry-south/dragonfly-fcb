/******************************************************************************
 * @file    receiver.c
 * @author  ÅF Dragonfly
 * @version v. 1.0.0
 * @date    2015-05-28
 * @brief   File contains functionality for signal readubg from the Dragonfly
 *          RC receiver. The receiver model is Spektrum AR610, which uses DSMX
 *          frequency modulation technology. It outputs 6 channels: throttle,
 *          aileron, elevator, rudder, gear and aux1.
 *
 *          _RECEIVER CHANNEL ENCODING_
 *          Since a timer IC on STM32 only has up to 4 channels, two timers are
 *          needed to collect the receiver pulses. The pulses sent from the
 *          receiver are typically ~1-2 ms width with a period of ~22 ms.
 *
 *          It is the pulse width that encodes the received transmitter stick
 *          action and is therefore the most relevant quantity when reading the
 *          signals from the receiver. As example, the aileron channel outputs
 *          ~1 ms when the transmitter control stick is held in one direction and
 *          ~2 ms when it is held in the opposite direction.
 *
 *          _PERFORMING A CALIBRATION_
 *          To perform a calibration of the receiver channels, the function
 *          StartReceiverCalibration() must be called. The receiver channels
 *          will then be sampled for RECEIVER_CALIBRATION_DURATION during which
 *          the user must push all the sticks in to their top and bottom ranges
 *          cyclically as well as toggling the aux1 and gear switches a few
 *          times.
 *
 *          Naturally, the receiver must be actively reading from the transmitter
 *          so checking that these are binded and calling IsReceiverActive(),
 *          this can be assured.
 *
 *          During calibration, the UpdateReceiverCalibrationValues() should be
 *          called to finalize the calibration procedure, where the new values
 *          are taken into use (if they are valid) and written to persistent
 *          flash memory for future use.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "receiver.h"
#include "main.h"
#include "flash.h"
#include "common.h"
#include <stdbool.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  Pulse_State ThrottleInputState;
  Pulse_State AileronInputState;
  Pulse_State ElevatorInputState;
  Pulse_State RudderInputState;
  Pulse_State GearInputState;
  Pulse_State Aux1InputState;
}Receiver_Pulse_States_TypeDef;

typedef struct
{
  uint32_t PeriodCount;
  uint16_t RisingCount;
  uint16_t FallingCounter;
  uint16_t PreviousRisingCount;
  uint16_t PreviousRisingCountTimerPeriodCount;
  uint16_t PulseTimerCount;
  ReceiverErrorStatus IsActive;
}Receiver_IC_Values_TypeDef;

typedef struct
{
  uint16_t maxSamplesBuffer[RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE];
  uint16_t minSamplesBuffer[RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE];
  uint16_t channelCalibrationPulseSamples;
  ReceiverCalibrationState channelCalibrationState;
}Receiver_ChannelCalibrationSampling_TypeDef;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Timer time base handlers for each timer */
TIM_HandleTypeDef PrimaryReceiverTimHandle;
TIM_HandleTypeDef AuxReceiverTimHandle;

/* Timer IC init declarations for each timer */
static TIM_IC_InitTypeDef PrimaryReceiverICConfig;
static TIM_IC_InitTypeDef AuxReceiverICConfig;

/* Struct contaning HIGH/LOW state for each input channel pulse */
static Receiver_Pulse_States_TypeDef ReceiverPulseStates;

/* Structs for each channel's timer count values */
static volatile Receiver_IC_Values_TypeDef ThrottleICValues;
static volatile Receiver_IC_Values_TypeDef AileronICValues;
static volatile Receiver_IC_Values_TypeDef ElevatorICValues;
static volatile Receiver_IC_Values_TypeDef RudderICValues;
static volatile Receiver_IC_Values_TypeDef GearICValues;
static volatile Receiver_IC_Values_TypeDef Aux1ICValues;

/* Struct for all receiver channel's calibration values */
static volatile Receiver_CalibrationValues_TypeDef CalibrationValues;

/* Structs for sampling the receiver channels when calibrating */
static volatile Receiver_ChannelCalibrationSampling_TypeDef ThrottleCalibrationSampling;
static volatile Receiver_ChannelCalibrationSampling_TypeDef AileronCalibrationSampling;
static volatile Receiver_ChannelCalibrationSampling_TypeDef ElevatorCalibrationSampling;
static volatile Receiver_ChannelCalibrationSampling_TypeDef RudderCalibrationSampling;
static volatile Receiver_ChannelCalibrationSampling_TypeDef GearCalibrationSampling;
static volatile Receiver_ChannelCalibrationSampling_TypeDef Aux1CalibrationSampling;
static volatile uint32_t calibrationStartTime;

/* Timer reset counters for each timer */
static volatile uint16_t PrimaryReceiverTimerPeriodCount;
static volatile uint16_t AuxReceiverTimerPeriodCount;

/* Private function prototypes -----------------------------------------------*/
static ReceiverErrorStatus InitReceiverCalibrationValues(void);
static ReceiverErrorStatus LoadReceiverCalibrationValuesFromFlash(volatile Receiver_CalibrationValues_TypeDef* calibrationValues);
static void SetDefaultReceiverCalibrationValues(volatile Receiver_CalibrationValues_TypeDef* calibrationValues);
static ReceiverErrorStatus PrimaryReceiverInput_Config(void);
static ReceiverErrorStatus AuxReceiverInput_Config(void);

static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState, volatile Receiver_IC_Values_TypeDef* ChannelICValues,
    const uint32_t receiverChannel, volatile const uint16_t* ReceiverTimerPeriodCount, volatile Receiver_ChannelCalibrationSampling_TypeDef* ChannelCalibrationSampling);
static ReceiverErrorStatus UpdateChannelCalibrationSamples(volatile Receiver_ChannelCalibrationSampling_TypeDef* channelCalibrationSampling, const uint16_t channelPulseTimerCount);
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void);
static ReceiverErrorStatus UpdateReceiverAileronChannel(void);
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void);
static ReceiverErrorStatus UpdateReceiverRudderChannel(void);
static ReceiverErrorStatus UpdateReceiverGearChannel(void);
static ReceiverErrorStatus UpdateReceiverAux1Channel(void);

static int16_t GetSignedReceiverChannel(volatile const Receiver_IC_Values_TypeDef* ChannelICValues, volatile const Receiver_IC_ChannelCalibrationValues_TypeDef* ChannelCalibrationValues);
static uint16_t GetUnsignedReceiverChannel(volatile const Receiver_IC_Values_TypeDef* ChannelICValues, volatile const Receiver_IC_ChannelCalibrationValues_TypeDef* ChannelCalibrationValues);
static uint16_t GetReceiverChannelPulseMicros(volatile const Receiver_IC_Values_TypeDef* ChannelICValues);
static uint16_t GetReceiverChannelPeriodMicros(volatile const Receiver_IC_Values_TypeDef* ChannelICValues);

static ReceiverErrorStatus IsReceiverChannelActive(volatile Receiver_IC_Values_TypeDef* ChannelICValues, const uint16_t ReceiverTimerPeriodCount);
static ReceiverErrorStatus IsReceiverPulseValid(const uint16_t pulseTimerCount, const uint16_t currentPeriodCount, const uint16_t previousPeriodCount);
static ReceiverErrorStatus IsReceiverPeriodValid(const uint32_t periodTimerCount);
static ReceiverErrorStatus IsCalibrationValuesValid(volatile const Receiver_CalibrationValues_TypeDef* calibrationValues);
static ReceiverErrorStatus IsCalibrationMaxPulseValueValid(const uint16_t maxPulseValue);
static ReceiverErrorStatus IsCalibrationMinPulseValueValid(const uint16_t minPulseValue);

static void ResetCalibrationSamplingStatesToWaiting(void);

/* Exported functions --------------------------------------------------------*/

/*
 * @brief  Initializes timers in input capture mode to read the receiver input signals
 * @param  None
 * @retval None
 */
ReceiverErrorStatus ReceiverInput_Config(void)
{
  InitReceiverCalibrationValues();

  if(!PrimaryReceiverInput_Config())
    return RECEIVER_ERROR;

  if(!AuxReceiverInput_Config())
    return RECEIVER_ERROR;

  return RECEIVER_OK;
}

/*
 * @brief  Returns a normalized receiver throttle pulse value as an unsigned integer.
 * @param  None
 * @retval throttle value [0, 65535]
 */
uint16_t GetThrottleReceiverChannel(void)
{
  return GetUnsignedReceiverChannel(&ThrottleICValues, &CalibrationValues.ThrottleChannel);
}

/*
 * @brief  Returns a normalized receiver aileron pulse value as a signed integer.
 * @param  None
 * @retval aileron value [-32768, 32767]
 */
int16_t GetAileronReceiverChannel(void)
{
  return GetSignedReceiverChannel(&AileronICValues, &CalibrationValues.AileronChannel);
}

/*
 * @brief  Returns a normalized receiver elevator pulse value as a signed integer.
 * @param  None
 * @retval elevator value [-32768, 32767]
 */
int16_t GetElevatorReceiverChannel(void)
{
  return GetSignedReceiverChannel(&ElevatorICValues, &CalibrationValues.ElevatorChannel);
}

/*
 * @brief  Returns a normalized receiver rudder pulse value as a signed integer.
 * @param  None
 * @retval rudder value [-32768, 32767]
 */
int16_t GetRudderReceiverChannel(void)
{
  return GetSignedReceiverChannel(&RudderICValues, &CalibrationValues.RudderChannel);
}

/*
 * @brief  Returns a normalized receiver gear pulse value as a signed integer.
 * @param  None
 * @retval gear value [-32768, 32767]
 */
int16_t GetGearReceiverChannel(void)
{
  return GetSignedReceiverChannel(&GearICValues, &CalibrationValues.GearChannel);
}

/*
 * @brief  Returns a normalized receiver aux1 pulse value as a signed integer.
 * @param  None
 * @retval aux1 value [-32768, 32767]
 */
int16_t GetAux1ReceiverChannel(void)
{
  return GetSignedReceiverChannel(&Aux1ICValues, &CalibrationValues.Aux1Channel);
}

/*
 * @brief  Returns the last throttle pulse value in microseconds
 * @param  None
 * @retval throttle pulse value in microseconds
 */
uint16_t GetThrottleReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&ThrottleICValues);
}

/*
 * @brief  Returns the last aileron pulse value in microseconds
 * @param  None
 * @retval aileron pulse value in microseconds
 */
uint16_t GetAileronReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&AileronICValues);
}

/*
 * @brief  Returns the last elevator pulse value in microseconds
 * @param  None
 * @retval elevator pulse value in microseconds
 */
uint16_t GetElevatorReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&ElevatorICValues);
}

/*
 * @brief  Returns the last rudder pulse value in microseconds
 * @param  None
 * @retval rudder pulse value in microseconds
 */
uint16_t GetRudderReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&RudderICValues);
}

/*
 * @brief  Returns the last gear pulse value in microseconds
 * @param  None
 * @retval gear pulse value in microseconds
 */
uint16_t GetGearReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&GearICValues);
}

/*
 * @brief  Returns the last aux1 pulse value in microseconds
 * @param  None
 * @retval aux1 pulse value in microseconds
 */
uint16_t GetAux1ReceiverChannelPulseMicros(void)
{
  return GetReceiverChannelPulseMicros(&Aux1ICValues);
}

/*
 * @brief  Returns the last throttle period value in microseconds
 * @param  None
 * @retval throttle period value in microseconds
 */
uint16_t GetThrottleReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&ThrottleICValues);
}

/*
 * @brief  Returns the last aileron period value in microseconds
 * @param  None
 * @retval aileron period value in microseconds
 */
uint16_t GetAileronReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&AileronICValues);
}

/*
 * @brief  Returns the last elevator period value in microseconds
 * @param  None
 * @retval elevator period value in microseconds
 */
uint16_t GetElevatorReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&ElevatorICValues);
}

/*
 * @brief  Returns the last rudder period value in microseconds
 * @param  None
 * @retval rudder period value in microseconds
 */
uint16_t GetRudderReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&RudderICValues);
}

/*
 * @brief  Returns the last gear period value in microseconds
 * @param  None
 * @retval gear period value in microseconds
 */
uint16_t GetGearReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&GearICValues);
}

/*
 * @brief  Returns the last aux1 period value in microseconds
 * @param  None
 * @retval aux1 period value in microseconds
 */
uint16_t GetAux1ReceiverChannelPeriodMicros(void)
{
  return GetReceiverChannelPeriodMicros(&Aux1ICValues);
}


/*
 * @brief  Starts the receiver calibration procedure to identify stick/switch max and min values. During
 *         calibration, make sure the receiver and transmitter has connected and that the receiver is
 *         actively reading from the transmitter. Be sure to push the four sticks (throttle, aileron,
 *         elevator and rudder) to their top and bottom positions and toggle the gear and aux1 switches
 *         a few times.
 * @param  None.
 * @retval RECEIVER_OK if calibration could be started, RECEIVER_ERROR if calibration already in progress
 */
ReceiverErrorStatus StartReceiverCalibration(void)
{
  /* Reset the receiver channel's calibration sampling structs */
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&ThrottleCalibrationSampling, 0, sizeof(ThrottleCalibrationSampling));
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&ElevatorCalibrationSampling, 0, sizeof(ElevatorCalibrationSampling));
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&AileronCalibrationSampling, 0, sizeof(AileronCalibrationSampling));
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&RudderCalibrationSampling, 0, sizeof(RudderCalibrationSampling));
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&GearCalibrationSampling, 0, sizeof(GearCalibrationSampling));
  memset((Receiver_ChannelCalibrationSampling_TypeDef*)&Aux1CalibrationSampling, 0, sizeof(Aux1CalibrationSampling));

  /* Initialize the calibration states */
  ThrottleCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;
  AileronCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;
  ElevatorCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;
  RudderCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;
  GearCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;
  Aux1CalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_IN_PROGRESS;

  /* Set the calibration start time */
  calibrationStartTime = HAL_GetTick();
  return RECEIVER_OK;
}

/*
 * @brief  If receiver calibration has been performed, calling this function updates the channels' calibration
 *         states and writes calibration values to flash where they are stored for future use. It is necessary
 *         to call this function after calibration in order for the new calibration values to be set and written
 *         to flash.
 * @param  None.
 * @retval RECEIVER_OK if calibration finalized correctly, else RECEIVER_ERROR
 */
ReceiverErrorStatus UpdateReceiverCalibrationValues(void)
{
  if(ThrottleCalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED && AileronCalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED && ElevatorCalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED
      && RudderCalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED && GearCalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED && Aux1CalibrationSampling.channelCalibrationState == RECEIVER_CALIBRATION_FINISHED)
    {
      Receiver_CalibrationValues_TypeDef tmpCalibrationValues;

      /* Check so that each channel has collected enough pulse samples during calibration */
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;
      if(ThrottleCalibrationSampling.channelCalibrationPulseSamples < RECEIVER_CALIBRATION_MIN_PULSE_COUNT)
        return RECEIVER_ERROR;

      /* Calculate mean of max and mean sample buffers */
      tmpCalibrationValues.ThrottleChannel.ChannelMaxCount = UInt16_Mean((uint16_t*)&ThrottleCalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.ThrottleChannel.ChannelMinCount = UInt16_Mean((uint16_t*)&ThrottleCalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.AileronChannel.ChannelMaxCount = UInt16_Mean((uint16_t*)&AileronCalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.AileronChannel.ChannelMinCount = UInt16_Mean((uint16_t*)&AileronCalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.ElevatorChannel.ChannelMaxCount = UInt16_Mean((uint16_t*)&ElevatorCalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.ElevatorChannel.ChannelMinCount = UInt16_Mean((uint16_t*)&ElevatorCalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.RudderChannel.ChannelMaxCount = UInt16_Mean((uint16_t*)&RudderCalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.RudderChannel.ChannelMinCount = UInt16_Mean((uint16_t*)&RudderCalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.GearChannel.ChannelMaxCount = UInt16_Mean((uint16_t*)&GearCalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.GearChannel.ChannelMinCount = UInt16_Mean((uint16_t*)&GearCalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.Aux1Channel.ChannelMaxCount = UInt16_Mean((uint16_t*)&Aux1CalibrationSampling.maxSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);
      tmpCalibrationValues.Aux1Channel.ChannelMinCount = UInt16_Mean((uint16_t*)&Aux1CalibrationSampling.minSamplesBuffer[0], RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE);

      /* Check validity of calibration values */
      ReceiverErrorStatus calibrationValid = IsCalibrationValuesValid(&tmpCalibrationValues);
      if(!calibrationValid)
        return RECEIVER_ERROR;

      /* Write calibration values to flash for persistent storage */
      WriteCalibrationValuesToFlash(&tmpCalibrationValues);

      /* Copy values to used calibration values */
      CalibrationValues.ThrottleChannel.ChannelMaxCount = tmpCalibrationValues.ThrottleChannel.ChannelMaxCount;
      CalibrationValues.ThrottleChannel.ChannelMinCount = tmpCalibrationValues.ThrottleChannel.ChannelMinCount;
      CalibrationValues.AileronChannel.ChannelMaxCount = tmpCalibrationValues.AileronChannel.ChannelMaxCount;
      CalibrationValues.AileronChannel.ChannelMinCount = tmpCalibrationValues.AileronChannel.ChannelMinCount;
      CalibrationValues.ElevatorChannel.ChannelMaxCount = tmpCalibrationValues.ElevatorChannel.ChannelMaxCount;
      CalibrationValues.ElevatorChannel.ChannelMinCount = tmpCalibrationValues.ElevatorChannel.ChannelMinCount;
      CalibrationValues.RudderChannel.ChannelMaxCount = tmpCalibrationValues.RudderChannel.ChannelMaxCount;
      CalibrationValues.RudderChannel.ChannelMinCount = tmpCalibrationValues.RudderChannel.ChannelMinCount;
      CalibrationValues.GearChannel.ChannelMaxCount = tmpCalibrationValues.GearChannel.ChannelMaxCount;
      CalibrationValues.GearChannel.ChannelMinCount = tmpCalibrationValues.GearChannel.ChannelMinCount;
      CalibrationValues.Aux1Channel.ChannelMaxCount = tmpCalibrationValues.Aux1Channel.ChannelMaxCount;
      CalibrationValues.Aux1Channel.ChannelMinCount = tmpCalibrationValues.Aux1Channel.ChannelMinCount;


      /* Reset calibration states to waiting so a new calibration may be initiated */
      ResetCalibrationSamplingStatesToWaiting();

      return RECEIVER_OK;
    }

  /* If receiver is currently waiting for calibration or calibration in progress */
  return RECEIVER_ERROR;
}

/*
 * @brief  Checks if the RC transmission between transmitter and receiver is active.
 * @param  None.
 * @retval RECEIVER_OK if transmission is active, else RECEIVER_ERROR.
 */
ReceiverErrorStatus IsReceiverActive(void)
{
  ReceiverErrorStatus aileronChannelActive;
  ReceiverErrorStatus elevatorChannelActive;
  ReceiverErrorStatus rudderChannelActive;

  /* When transmission stops, the throttle channel on the Spektrum AR610 receiver keeps sending pulses on its
   * channel based on its last received throttle command. But the other channels go silents, so they can be
   * used to check if the transmission is down. */
  aileronChannelActive = IsReceiverChannelActive(&AileronICValues, PrimaryReceiverTimerPeriodCount);
  elevatorChannelActive = IsReceiverChannelActive(&ElevatorICValues, PrimaryReceiverTimerPeriodCount);
  rudderChannelActive = IsReceiverChannelActive(&RudderICValues, PrimaryReceiverTimerPeriodCount);

  return (aileronChannelActive && elevatorChannelActive && rudderChannelActive);
}

/**
 * @brief  Input Capture callback in non blocking mode
 * @param  htim : TIM IC handle
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==PRIMARY_RECEIVER_TIM)
    {
      if(htim->Channel == PRIMARY_RECEIVER_THROTTLE_ACTIVE_CHANNEL)
        UpdateReceiverThrottleChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_AILERON_ACTIVE_CHANNEL)
        UpdateReceiverAileronChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL)
        UpdateReceiverElevatorChannel();
      else if(htim->Channel == PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL)
        UpdateReceiverRudderChannel();
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      if(htim->Channel == AUX_RECEIVER_GEAR_ACTIVE_CHANNEL)
        UpdateReceiverGearChannel();
      else if(htim->Channel == AUX_RECEIVER_AUX1_ACTIVE_CHANNEL)
        UpdateReceiverAux1Channel();
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==PRIMARY_RECEIVER_TIM)
    {
      PrimaryReceiverTimerPeriodCount++;
    }
  else if (htim->Instance==AUX_RECEIVER_TIM)
    {
      AuxReceiverTimerPeriodCount++;
    }
}

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Initializes receiver input calibration values (max and min timer IC counts)
 * @param  None
 * @retval RECEIVER_OK if calibration values loaded, RECEIVER_ERROR if default values used
 */
static ReceiverErrorStatus InitReceiverCalibrationValues(void)
{
  if(!LoadReceiverCalibrationValuesFromFlash(&CalibrationValues))
    {
      SetDefaultReceiverCalibrationValues(&CalibrationValues);
      return RECEIVER_ERROR;
    }
  return RECEIVER_OK;
}

/*
 * @brief  Sets receiver calibration values to default
 * @param  calibrationValues : pointer to a calibration values struct
 * @retval None
 */
static void SetDefaultReceiverCalibrationValues(volatile Receiver_CalibrationValues_TypeDef* calibrationValues)
{
  calibrationValues->ThrottleChannel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->ThrottleChannel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  calibrationValues->AileronChannel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->AileronChannel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  calibrationValues->ElevatorChannel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->ElevatorChannel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  calibrationValues->RudderChannel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->RudderChannel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  calibrationValues->GearChannel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->GearChannel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;

  calibrationValues->Aux1Channel.ChannelMaxCount = RECEIVER_PULSE_DEFAULT_MAX_COUNT;
  calibrationValues->Aux1Channel.ChannelMinCount = RECEIVER_PULSE_DEFAULT_MIN_COUNT;
}

/*
 * @brief  Returns a normalized receiver channel value as a signed integer.
 * @param  ChannelICValues : Reference to channel's IC values struct
 * @param  ChannelCalibrationValues : Reference to channel's calibration values struct
 * @retval channel value [-32768, 32767]
 */
static int16_t GetSignedReceiverChannel(volatile const Receiver_IC_Values_TypeDef* ChannelICValues, volatile const Receiver_IC_ChannelCalibrationValues_TypeDef* ChannelCalibrationValues)
{
  if(ChannelICValues->PulseTimerCount < ChannelCalibrationValues->ChannelMinCount)
    return INT16_MIN;
  else if(ChannelICValues->PulseTimerCount > ChannelCalibrationValues->ChannelMaxCount)
    return INT16_MAX;
  else if(ChannelCalibrationValues->ChannelMaxCount > ChannelCalibrationValues->ChannelMinCount)
    return INT16_MIN + (((uint32_t) (ChannelICValues->PulseTimerCount-ChannelCalibrationValues->ChannelMinCount)*UINT16_MAX)/(ChannelCalibrationValues->ChannelMaxCount-ChannelCalibrationValues->ChannelMinCount));
  else
    return 0;
}

/*
 * @brief  Returns a normalized receiver channel value as an unsigned integer.
 * @param  ChannelICValues : Reference to channel's IC values struct
 * @param  ChannelCalibrationValues : Reference to channel's calibration values struct
 * @retval channel value [0, 65535]
 */
static uint16_t GetUnsignedReceiverChannel(volatile const Receiver_IC_Values_TypeDef* ChannelICValues, volatile const Receiver_IC_ChannelCalibrationValues_TypeDef* ChannelCalibrationValues)
{
  if(ChannelICValues->PulseTimerCount < ChannelCalibrationValues->ChannelMinCount)
    return 0;
  else if(ChannelICValues->PulseTimerCount > ChannelCalibrationValues->ChannelMaxCount)
    return UINT16_MAX;
  else if(ChannelCalibrationValues->ChannelMaxCount > ChannelCalibrationValues->ChannelMinCount)
    return ((uint32_t) (ChannelICValues->PulseTimerCount-ChannelCalibrationValues->ChannelMinCount)*UINT16_MAX)/(ChannelCalibrationValues->ChannelMaxCount-ChannelCalibrationValues->ChannelMinCount);
  else
    return 0;
}

/*
 * @brief  Returns the receiver channel pulse count in microseconds
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @retval Channel pulse timer count in microseconds
 */
static uint16_t GetReceiverChannelPulseMicros(volatile const Receiver_IC_Values_TypeDef* ChannelICValues)
{
  return (uint16_t)(((uint32_t)ChannelICValues->PulseTimerCount * 1000000) / RECEIVER_TIM_COUNTER_CLOCK);
}

/*
 * @brief  Returns the receiver channel period count in microseconds
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @retval Channel period timer count in microseconds
 */
static uint16_t GetReceiverChannelPeriodMicros(volatile const Receiver_IC_Values_TypeDef* ChannelICValues)
{
  /* The period count will never be more than RECEIVER_MAX_VALID_PERIOD_COUNT, take care so that 32-bit overflow does not occur below */
  return (uint16_t)(((uint32_t)ChannelICValues->PeriodCount * 100000) / RECEIVER_TIM_COUNTER_CLOCK * 10);
}

/*
 * @brief  Gets calibration values stored in flash after previously performed receiver calibration
 * @param  calibrationValues : pointer to calibration values struct
 * @retval true if valid calibration values has been loaded, else false.
 */
static ReceiverErrorStatus LoadReceiverCalibrationValuesFromFlash(volatile Receiver_CalibrationValues_TypeDef* calibrationValues)
{
  /* Load previously stored values into the calibrationValues struct */
  ReadCalibrationValuesFromFlash(calibrationValues);

  /* Check so that loaded values are within valid ranges */
  if(!IsCalibrationValuesValid(calibrationValues))
    return RECEIVER_ERROR;

  return RECEIVER_OK;
}

/*
 * @brief  Initializes reading from the receiver primary input channels, i.e. throttle aileron,
 *     elevator and rudder channels. The signals are encoded as pulses of ~1-2 ms.
 * @param  None.
 * @retval RECEIVER_OK if configured without errors, else RECEIVER_ERROR
 */
static ReceiverErrorStatus PrimaryReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /*##-1- Configure the Primary Receiver TIM peripheral ######################*/
  /* Set TIM instance */
  PrimaryReceiverTimHandle.Instance = PRIMARY_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver input period is ~22 ms) */
  PrimaryReceiverTimHandle.Init.Period = RECEIVER_COUNTER_PERIOD;
  PrimaryReceiverTimHandle.Init.Prescaler = SystemCoreClock/RECEIVER_TIM_COUNTER_CLOCK - 1;
  PrimaryReceiverTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  PrimaryReceiverTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&PrimaryReceiverTimHandle) != HAL_OK)
    {
      /* Initialization Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-2- Configure the Input Capture channels ###############################*/
  /* Common configuration */
  PrimaryReceiverICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  PrimaryReceiverICConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of elevator channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of rudder channel */
  PrimaryReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  PrimaryReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_THROTTLE_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_AILERON_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_ELEVATOR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&PrimaryReceiverTimHandle, PRIMARY_RECEIVER_RUDDER_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-4- Start the Time Base update interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&PrimaryReceiverTimHandle) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief    Initializes reading from the receiver aux input channels, i.e.
 *       Gear and Aux1. The signals are encoded as pulses of ~1-2 ms.
 * @param    None.
 * @retval   RECEIVER_OK if configured without errors, else RECEIVER_ERROR
 */
static ReceiverErrorStatus AuxReceiverInput_Config(void)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /*##-1- Configure the Aux Receiver TIM peripheral ##########################*/
  /* Set TIM instance */
  AuxReceiverTimHandle.Instance = AUX_RECEIVER_TIM;

  /* Initialize TIM peripheral to maximum period with suitable counter clocking (receiver input period is ~22 ms) */
  AuxReceiverTimHandle.Init.Period = RECEIVER_COUNTER_PERIOD;
  AuxReceiverTimHandle.Init.Prescaler = SystemCoreClock/RECEIVER_TIM_COUNTER_CLOCK - 1;
  AuxReceiverTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  AuxReceiverTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&AuxReceiverTimHandle) != HAL_OK)
    {
      /* Initialization Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-2- Configure the Input Capture channels ###############################*/
  /* Common configuration */
  AuxReceiverICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  AuxReceiverICConfig.ICFilter = 0;

  /* Configure the Input Capture of throttle channel */
  AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  AuxReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /* Configure the Input Capture of aileron channel */
  AuxReceiverICConfig.ICPolarity = TIM_ICPOLARITY_RISING;
  AuxReceiverICConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if(HAL_TIM_IC_ConfigChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-3- Start the Input Capture in interrupt mode ##########################*/
  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_GEAR_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  if(HAL_TIM_IC_Start_IT(&AuxReceiverTimHandle, AUX_RECEIVER_AUX1_CHANNEL) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  /*##-4- Start the Time Base update interrupt mode ##########################*/
  if(HAL_TIM_Base_Start_IT(&AuxReceiverTimHandle) != HAL_OK)
    {
      /* Starting Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief  Updates a receiver channel IC counts. The channel is specified by the function parameters.
 * @param  TimHandle : Reference to the TIM_HandleTypeDef struct used to read the channel's IC count
 * @param  TimIC : Reference to the TIM_IC_InitTypeDef struct used to configure the IC to count on rising/falling pulse flank
 * @param  channelInputState : The current channel pulse input state (PULSE_LOW or PULSE_HIGH)
 * @param  ChannelICValues : Reference to the channels IC value struct
 * @param  receiverChannel : TIM Channel
 * @param  ReceiverTimerPeriodCount : Reference to the timer period count variable (primary or aux)
 * @param  ChannelCalibrationSampling : Reference to channel's calibration sampling struct
 * @retval RECEIVER_OK if valid pulse or period detected, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverChannel(TIM_HandleTypeDef* TimHandle, TIM_IC_InitTypeDef* TimIC, Pulse_State* channelInputState, volatile Receiver_IC_Values_TypeDef* ChannelICValues,
    const uint32_t receiverChannel, volatile const uint16_t* ReceiverTimerPeriodCount, volatile Receiver_ChannelCalibrationSampling_TypeDef* ChannelCalibrationSampling)
{
  ReceiverErrorStatus errorStatus = RECEIVER_OK;

  /* Detected rising pulse edge */
  if ((*channelInputState) == PULSE_LOW)
    {
      uint32_t tempPeriodTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(TimHandle, receiverChannel);
      (*channelInputState) = PULSE_HIGH; // Set input state to high

      TimIC->ICPolarity = TIM_ICPOLARITY_FALLING; // Set IC polarity property to falling

      /* Set the rising timer IC counts */
      ChannelICValues->PreviousRisingCount = ChannelICValues->RisingCount;
      ChannelICValues->RisingCount = icValue;

      /* Calculate the period between pulses by computing the difference between current and previous rising edge timer counts.
       * Since the counter typically resets more often than a new pulse is triggered for the receiver, one must also use the
       * number of timer period resets since the last pulse. */
      if((*ReceiverTimerPeriodCount) > ChannelICValues->PreviousRisingCountTimerPeriodCount)
        tempPeriodTimerCount = ChannelICValues->RisingCount + UINT16_MAX - ChannelICValues->PreviousRisingCount
        + UINT16_MAX*((*ReceiverTimerPeriodCount)-ChannelICValues->PreviousRisingCountTimerPeriodCount-1);
      else
        tempPeriodTimerCount = ChannelICValues->RisingCount - ChannelICValues->PreviousRisingCount; // Short period value, likely incorrect

      if(IsReceiverPeriodValid(tempPeriodTimerCount))
        ChannelICValues->PeriodCount = tempPeriodTimerCount;
      else
        errorStatus = RECEIVER_ERROR;

      ChannelICValues->PreviousRisingCountTimerPeriodCount = (*ReceiverTimerPeriodCount);
    }
  /* Detected falling pulse edge */
  else if ((*channelInputState) == PULSE_HIGH)
    {
      uint16_t tempPulseTimerCount;

      /* Get the Input Capture value */
      uint32_t icValue = HAL_TIM_ReadCapturedValue(TimHandle, receiverChannel);
      (*channelInputState) = PULSE_LOW; // Set input state to low

      TimIC->ICPolarity = TIM_ICPOLARITY_RISING; // Set IC polarity property to rising

      /* Set the falling timer IC count */
      ChannelICValues->FallingCounter = icValue;

      /* Calculate the pulse of the 16-bit counter by computing the difference between falling and rising edges timer counts */
      tempPulseTimerCount = ChannelICValues->FallingCounter - ChannelICValues->RisingCount;

      /* Sanity check of pulse count before updating it */
      if(IsReceiverPulseValid(tempPulseTimerCount, (*ReceiverTimerPeriodCount), ChannelICValues->PreviousRisingCountTimerPeriodCount))
        {
          ChannelICValues->PulseTimerCount = tempPulseTimerCount;
          ChannelICValues->IsActive = RECEIVER_OK; // Set channel to active
          if(ChannelCalibrationSampling->channelCalibrationState == RECEIVER_CALIBRATION_IN_PROGRESS)
            UpdateChannelCalibrationSamples(ChannelCalibrationSampling, tempPulseTimerCount);
        }
      else
        errorStatus = RECEIVER_ERROR;
    }

  /* Toggle the IC Polarity */
  if(HAL_TIM_IC_ConfigChannel(TimHandle, TimIC, receiverChannel) != HAL_OK)
    {
      /* Configuration Error */
      errorStatus = RECEIVER_ERROR;
      Error_Handler();
    }

  return errorStatus;
}

/*
 * @brief  Updates the receiver channel's calibration samples
 * @param  channelCalibrationSampling : calibration sampling struct
 * @param  channelPulseTimerCount : The pulse count value
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateChannelCalibrationSamples(volatile Receiver_ChannelCalibrationSampling_TypeDef* channelCalibrationSampling, const uint16_t channelPulseTimerCount)
{
  static uint16_t tmpMaxIndex;
  static uint16_t tmpMaxBufferMinValue;
  static bool maxBufferUpdated;

  static uint16_t tmpMinIndex;
  static uint16_t tmpMinBufferMaxValue;
  static bool minBufferUpdated;

  uint16_t i;

  /* # If a new calibration has been started, reset the static variables used in this function */
  if(channelCalibrationSampling->channelCalibrationPulseSamples == 0)
    {
      tmpMaxIndex = 0;
      tmpMaxBufferMinValue = UINT16_MAX;
      maxBufferUpdated = true;

      tmpMinIndex = 0;
      tmpMinBufferMaxValue = 0;
      minBufferUpdated = true;
    }

  /* # Find the min value in the buffer containing the max values ########### */
  if(maxBufferUpdated)
    {
      for(i = 0; i < RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE; i++)
        {
          if(tmpMaxBufferMinValue > channelCalibrationSampling->maxSamplesBuffer[i])
            {
              tmpMaxBufferMinValue = channelCalibrationSampling->maxSamplesBuffer[i];
              tmpMaxIndex = i;
              maxBufferUpdated = false;
            }
        }
    }

  /* If the pulse value is larger than the min value of the max value buffer, replace it */
  if(channelPulseTimerCount > tmpMaxBufferMinValue)
    {
      channelCalibrationSampling->maxSamplesBuffer[tmpMaxIndex] = channelPulseTimerCount;
      maxBufferUpdated = true;
    }


  /* # Find the max value in the buffer containing the min values ########### */
  if(minBufferUpdated)
    {
      for(i = 0; i < RECEIVER_CALIBRATION_SAMPLES_BUFFER_SIZE; i++)
        {
          if(tmpMinBufferMaxValue < channelCalibrationSampling->minSamplesBuffer[i])
            {
              tmpMinBufferMaxValue = channelCalibrationSampling->minSamplesBuffer[i];
              tmpMinIndex = i;
              minBufferUpdated = false;
            }
        }
    }

  /* If the pulse value is larger than the min value of the max value buffer, replace it */
  if(channelPulseTimerCount < tmpMinBufferMaxValue)
    {
      channelCalibrationSampling->minSamplesBuffer[tmpMinIndex] = channelPulseTimerCount;
      minBufferUpdated = true;
    }

  /* Increase amount of channel calibration samples */
  channelCalibrationSampling->channelCalibrationPulseSamples++;

  /* # Check if max calibration time has been reached - stop calibration #### */
  if(HAL_GetTick() > RECEIVER_CALIBRATION_DURATION + calibrationStartTime)
    channelCalibrationSampling->channelCalibrationState = RECEIVER_CALIBRATION_FINISHED;

  return RECEIVER_OK;
}

/*
 * @brief  Handles the receiver input pulse measurements from the throttle channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverThrottleChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ThrottleInputState,
      &ThrottleICValues, PRIMARY_RECEIVER_THROTTLE_CHANNEL, &PrimaryReceiverTimerPeriodCount, &ThrottleCalibrationSampling);
}

/*
 * @brief  Handles the receiver input pulse measurements from the aileron channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverAileronChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.AileronInputState,
      &AileronICValues, PRIMARY_RECEIVER_AILERON_CHANNEL, &PrimaryReceiverTimerPeriodCount, &AileronCalibrationSampling);
}

/*
 * @brief  Handles the receiver input pulse measurements from the elevator channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverElevatorChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.ElevatorInputState,
      &ElevatorICValues, PRIMARY_RECEIVER_ELEVATOR_CHANNEL, &PrimaryReceiverTimerPeriodCount, &ElevatorCalibrationSampling);
}

/*
 * @brief  Handles the receiver input pulse measurements from the rudder channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverRudderChannel(void)
{
  return UpdateReceiverChannel(&PrimaryReceiverTimHandle, &PrimaryReceiverICConfig, &ReceiverPulseStates.RudderInputState,
      &RudderICValues, PRIMARY_RECEIVER_RUDDER_CHANNEL, &PrimaryReceiverTimerPeriodCount, &RudderCalibrationSampling);
}

/*
 * @brief  Handles the receiver input pulse measurements from the gear channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverGearChannel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.GearInputState,
      &GearICValues, AUX_RECEIVER_GEAR_CHANNEL, &AuxReceiverTimerPeriodCount, &GearCalibrationSampling);
}

/*
 * @brief  Handles the receiver input pulse measurements from the aux1 channel and updates pulse and frequency values.
 * @param  None.
 * @retval RECEIVER_OK if updated correctly, else RECEIVER_ERROR
 */
static ReceiverErrorStatus UpdateReceiverAux1Channel(void)
{
  return UpdateReceiverChannel(&AuxReceiverTimHandle, &AuxReceiverICConfig, &ReceiverPulseStates.Aux1InputState,
      &Aux1ICValues, AUX_RECEIVER_AUX1_CHANNEL, &AuxReceiverTimerPeriodCount, &Aux1CalibrationSampling);
}

/*
 * @brief  Checks if the RC transmission between transmitter and receiver is active for a specified channel.
 * @param  ChannelICValues : Reference to a channel's IC values struct
 * @param  ReceiverTimerPeriodCount : The period count value for the receiver counter
 * @retval RECEIVER_OK if transmission is active, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverChannelActive(volatile Receiver_IC_Values_TypeDef* ChannelICValues, const uint16_t ReceiverTimerPeriodCount)
{
  uint32_t periodsSinceLastChannelPulse;

  /* Check how many timer resets have been performed since the last rising pulse edge */
  periodsSinceLastChannelPulse = ReceiverTimerPeriodCount - ChannelICValues->PreviousRisingCountTimerPeriodCount;

  /* Set channel as inactive if too many periods have passed since last channel pulse update */
  if(periodsSinceLastChannelPulse > IS_RECEIVER_CHANNEL_INACTIVE_PERIODS_COUNT)
    ChannelICValues->IsActive = RECEIVER_ERROR;

  return ChannelICValues->IsActive;
}

/*
 * @brief  Checks if a receiver channel pulse count is within a valid range
 * @param  pulseTimerCount : pulse timer count value
 * @param  currentPeriodCount : The current period count
 * @param  previousPeriodCount : The previous period count
 * @retval RECEIVER_OK if receiver pulse count is valid, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverPulseValid(const uint16_t pulseTimerCount, const uint16_t currentPeriodCount, const uint16_t previousPeriodCount)
{
  /* Pulse count considered valid if it is within defined bounds and doesn't strech over more than 2 timer periods */
  return (pulseTimerCount <= RECEIVER_MAX_VALID_IC_PULSE_COUNT && pulseTimerCount >= RECEIVER_MIN_VALID_IC_PULSE_COUNT
      && currentPeriodCount - previousPeriodCount <= 1);
}

/*
 * @brief  Checks if a receiver channel period count is within a valid range
 * @param  periodTimerCount : period timer count value
 * @retval RECEIVER_OK if receiver period count is valid, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsReceiverPeriodValid(const uint32_t periodTimerCount)
{
  /* Period count considered valid if it is within defined bounds */
  return (periodTimerCount <= RECEIVER_MAX_VALID_PERIOD_COUNT && periodTimerCount >= RECEIVER_MIN_VALID_PERIOD_COUNT);
}

/*
 * @brief  Checks if receiver calibration values are valid
 * @param  calibrationValues : pointer to calibration values struct
 * @retval RECEIVER_OK if receiver calibration values are valid, else RECEIVER_ERROR.
 */
static ReceiverErrorStatus IsCalibrationValuesValid(volatile const Receiver_CalibrationValues_TypeDef* calibrationValues)
{
  /* Check throttle channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->ThrottleChannel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->ThrottleChannel.ChannelMinCount))
    return RECEIVER_ERROR;

  /* Check aileron channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->AileronChannel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->AileronChannel.ChannelMinCount))
    return RECEIVER_ERROR;

  /* Check elevator channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->ElevatorChannel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->ElevatorChannel.ChannelMinCount))
    return RECEIVER_ERROR;

  /* Check rudder channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->RudderChannel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->RudderChannel.ChannelMinCount))
    return RECEIVER_ERROR;

  /* Check gear channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->GearChannel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->GearChannel.ChannelMinCount))
    return RECEIVER_ERROR;

  /* Check aux1 channel calibration */
  if(!IsCalibrationMaxPulseValueValid(calibrationValues->Aux1Channel.ChannelMaxCount))
    return RECEIVER_ERROR;
  if(!IsCalibrationMinPulseValueValid(calibrationValues->Aux1Channel.ChannelMinCount))
    return RECEIVER_ERROR;

  return RECEIVER_OK;
}

/*
 * @brief  Checks if receiver pulse value is a valid max calibration value
 * @param  maxPulseValue : pulse value
 * @retval RECEIVER_OK if receiver pulse value valid for max calibration, else RECEIVER_ERROR
 */
static ReceiverErrorStatus IsCalibrationMaxPulseValueValid(const uint16_t maxPulseValue)
{
  if(maxPulseValue <= RECEIVER_MAX_CALIBRATION_MAX_PULSE_COUNT && maxPulseValue >= RECEIVER_MAX_CALIBRATION_MIN_PULSE_COUNT)
    return RECEIVER_OK;

  return RECEIVER_ERROR;
}

/*
 * @brief  Checks if receiver pulse value is a valid min calibration value
 * @param  minPulseValue : pulse value
 * @retval RECEIVER_OK if receiver pulse value valid for min calibration, else RECEIVER_ERROR
 */
static ReceiverErrorStatus IsCalibrationMinPulseValueValid(const uint16_t minPulseValue)
{
  if(minPulseValue <= RECEIVER_MIN_CALIBRATION_MAX_PULSE_COUNT && minPulseValue >= RECEIVER_MIN_CALIBRATION_MIN_PULSE_COUNT)
    return RECEIVER_OK;

  return RECEIVER_ERROR;
}

/*
 * @brief  Resets the calibration sampling states to waiting
 * @param  None
 * @retval None
 */
static void ResetCalibrationSamplingStatesToWaiting(void)
{
  ThrottleCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
  AileronCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
  ElevatorCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
  RudderCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
  GearCalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
  Aux1CalibrationSampling.channelCalibrationState = RECEIVER_CALIBRATION_WAITING;
}

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
