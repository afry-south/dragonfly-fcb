/**
 * @file fcb_accelerometer.c
 *
 * Implements fcb_accelerometer_magnetometer.h API
 *
 *
 * @see fcb_accelerometer.h
 */
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensors.h"
#include "fcb_error.h"
#include "lsm303dlhc.h"
#include "usbd_cdc_if.h"
#include "trace.h"

#include "FreeRTOS.h"

#include <stdio.h>

// #define FCB_ACCMAG_DEBUG

float sXYZDotDot[] = { 0, 0 , 0 };
float sXYZMagVector[] = { 0, 0 , 0 };


enum { X_IDX = 0 }; /* index into sGyroXYZDotDot & ditto Offset sXYZMagVectors */
enum { Y_IDX = 1 }; /* as above */
enum { Z_IDX = 2 }; /* as above */

enum { ACCMAG_SAMPLING_MAX_STRING_SIZE = 128 };

/* static fcn declarations */

static void FcbInitialiseAccelerometer(void); /* configures LSM303DHLC accelerometer */


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

	FcbInitialiseAccelerometer();
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

static void FcbInitialiseAccelerometer(void) {
  {
      /* set up accelerometer */
      uint8_t ctrlReg1 = 0x00 |
          LSM303DLHC_NORMAL_MODE |
          LSM303DLHC_ODR_50_HZ |
          LSM303DLHC_AXES_ENABLE /* reg1 */;

      uint8_t ctrlReg3 = 0x00 | LSM303DLHC_IT1_DRY1 ;

      uint8_t ctrlReg4 = 0x00 |
          LSM303DLHC_FULLSCALE_2G |
          LSM303DLHC_BlockUpdate_Single |
          LSM303DLHC_BLE_LSB |
          LSM303DLHC_HR_ENABLE ;

      if (I_AM_LMS303DLHC != LSM303DLHC_AccReadID()) {
        ErrorHandler();
      }

      LSM303DLHC_AccRebootCmd();
      LSM303DLHC_AccInit(ctrlReg1, ctrlReg3, ctrlReg4);

      LSM303DLHC_AccFilterConfig(LSM303DLHC_HPM_NORMAL_MODE |
          LSM303DLHC_HPFCF_16 |
          LSM303DLHC_HPF_AOI1_DISABLE |
          LSM303DLHC_HPF_AOI2_DISABLE);
    }
}

void FetchDataFromAccelerometer(void) {
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

	LSM303DLHC_AccReadXYZ(sXYZDotDot);

#ifdef FCB_ACCMAG_DEBUG
	{
	  float xDotDot = (float)sXYZDotDot[X_IDX];

	  trace_post("xDotDot:%f", xDotDot);
	}
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
#endif
}

void FetchDataFromMagnetometer(void) {
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
	LSM303DLHC_MagReadXYZ(sXYZMagVector);

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

void GetMagVector(int16_t * x, int16_t * y, int16_t * z) {
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
