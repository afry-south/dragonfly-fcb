/**
 * @file fcb_accelerometer.c
 *
 * Implements fcb_accelerometer_magnetometer.h API
 *
 * This file does most of the GPIO & NVIC setup to receive the interrupts
 * from LSM303DLHC.
 *
 * The lsm303dlhc.c file does most of the configuration of the acc/magneto-meter
 * itself.
 *
 * This file also handles coordinate transform to translate magnetometer &
 * accelerometer data to the quadcopter x y z axes.
 *
 * @see fcb_accelerometer.h
 */
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensors.h"
#include "fcb_error.h"
#include "lsm303dlhc.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include "trace.h"

#include "FreeRTOS.h"

#include <stdint.h>
#include <stdio.h>

// #define FCB_ACCMAG_DEBUG

static float32_t sXYZDotDot[] = { 0, 0 , 0 };
static float32_t sXYZMagVector[] = { 0, 0 , 0 };

enum { X_IDX = 0 }; /* index into sGyroXYZDotDot & ditto Offset sXYZMagVectors */
enum { Y_IDX = 1 }; /* as above */
enum { Z_IDX = 2 }; /* as above */

enum { ACCMAG_SAMPLING_MAX_STRING_SIZE = 128 };
enum { ACCMAG_CALIBRATION_SAMPLES_N = 6 }; /* TODO increase */


/* static fcn declarations */

/* public fcn definitions */

uint8_t FcbInitialiseAccMagSensor(void) {
	uint8_t retVal = FCB_OK;

	/* configure STM32 interrupts & GPIO */
	ACCELERO_DRDY_GPIO_CLK_ENABLE(); /* GPIOE clock */

	/* STM32F3 doc UM1570 page 27/36. Accelerometer interrupt */
	FcbSensorsInitGpioPinForInterrupt(GPIOE, GPIO_PIN_4);

	/* STM32F3 doc UM1570 page 27/36. Magnetometer interrupt */
	FcbSensorsInitGpioPinForInterrupt(GPIOE, GPIO_PIN_2);

#ifdef FCB_ACCMAG_DEBUG
	{
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}

#endif

	HAL_NVIC_SetPriority(EXTI4_IRQn,
			configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn,
        configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

	LSM303DLHC_AccConfig();
	LSM303DLHC_MagInit();

    /* do a pre-read to get the DRDY interrupts going. Since we trig on
     * rising flank and the sensor has data from power-on, by the time we get
     * here the interrupt is already high. Reading the data trigs the
     * sensor to load a new set of values into its registers.
     */
	LSM303DLHC_AccReadXYZ(sXYZDotDot);
	LSM303DLHC_MagReadXYZ(sXYZMagVector);

	return retVal;
}

void FetchDataFromAccelerometer(void) {
  float acceleroMeterData[3] = { 0.0f, 0.0f, 0.0f };
#ifdef FCB_ACCMAG_DEBUG
	static uint32_t call_counter = 0;

	{
		if ((call_counter % 50) == 0) {
			BSP_LED_Toggle(LED8);
		}
		call_counter++;
	}

	/* measure duration with oscilloscope on pin PD9 */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
#endif

	LSM303DLHC_AccReadXYZ(acceleroMeterData);

    /* TODO apply calibration */

	/* from accelerometer to quadcopter coordinate axes
	 * see "Sensors" page in Wiki.
	 */
	sXYZDotDot[X_IDX] = acceleroMeterData[Y_IDX];
    sXYZDotDot[Y_IDX] = -acceleroMeterData[X_IDX];
    sXYZDotDot[Z_IDX] = -acceleroMeterData[Z_IDX];

#ifdef FCB_ACCMAG_DEBUG
	{
	  float xDotDot = (float)sXYZDotDot[X_IDX];

	  trace_post("xDotDot:%f", xDotDot);
	}
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
#endif
}

void BeginMagnetometerCalibration(void) {

}

void FetchDataFromMagnetometer(void) {
  float magnetoMeterData[3] = { 0.0f, 0.0f, 0.0f };
#ifdef FCB_ACCMAG_DEBUG
    static uint32_t call_counter = 0;

    {
        if ((call_counter % 75) == 0) {
            BSP_LED_Toggle(LED6);
        }
        call_counter++;
    }
    /* measure duration with oscilloscope on pin PD11 */
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);

#endif
    /* TODO possibly read the values into a temporary copy here ...
     * (contd below)
     */
	LSM303DLHC_MagReadXYZ(magnetoMeterData);
	sXYZMagVector[X_IDX] = magnetoMeterData[X_IDX];
	sXYZMagVector[Y_IDX] = - magnetoMeterData[Y_IDX];
	sXYZMagVector[Z_IDX] = - magnetoMeterData[Z_IDX];

	/* TODO ... and then copy them into a mutex-protected sXYZMagVector here */
#ifdef FCB_ACCMAG_DEBUG
	trace_post("sXYZMagVector[%f,%f,%f]");
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
#endif
}

void GetAcceleration(int16_t * xDotDot, int16_t * yDotDot, int16_t * zDotDot) {
	*xDotDot = sXYZDotDot[X_IDX];
	*yDotDot = sXYZDotDot[Y_IDX];
	*zDotDot = sXYZDotDot[Z_IDX];
}

void GetMagVector(float32_t * x, float32_t * y, float32_t * z) {
	*x = sXYZMagVector[X_IDX];
	*y = sXYZMagVector[Y_IDX];
	*z = sXYZMagVector[Z_IDX];
}


void PrintAccelerometerValues(void) {
    static char sampleString[ACCMAG_SAMPLING_MAX_STRING_SIZE];

    snprintf((char*) sampleString, ACCMAG_SAMPLING_MAX_STRING_SIZE,
            "Accelerometer readings [m/(s * s)]:\nAccX: %f\nAccY: %f\nAccZ: %f\n\r\n",
            sXYZDotDot[X_IDX],
            sXYZDotDot[Y_IDX],
            sXYZDotDot[Z_IDX]);

    USBComSendString(sampleString);
}

void PrintMagnetometerValues(void) {
    static char sampleString[ACCMAG_SAMPLING_MAX_STRING_SIZE];

    snprintf((char*) sampleString, ACCMAG_SAMPLING_MAX_STRING_SIZE,
            "Magnetometer readings [Gauss]:\nMagX: %1.6f\nMagY: %1.6f\nMagZ: %1.6f\n\r\n",
            sXYZMagVector[X_IDX],
            sXYZMagVector[Y_IDX],
            sXYZMagVector[Z_IDX]);

    USBComSendString(sampleString);
}
