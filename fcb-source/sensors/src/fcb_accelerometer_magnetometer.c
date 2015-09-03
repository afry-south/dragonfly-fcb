/**
 * @file fcb_accelerometer.c
 *
 * Implements fcb_accelerometer.h API
 *
 * @see fcb_accelerometer.h
 */
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_sensors.h"
#include "fcb_error.h"
#include "stm32f3_discovery_accelerometer.h"
#include "trace.h"

#include "FreeRTOS.h"

// #define FCB_ACCMAG_DEBUG

int16_t sXYZDotDot[] = { 0, 0 , 0 };
float sXYZMagVector[] = { 0, 0 , 0 };


enum { X_IDX = 0 }; /* index into sGyroXYZDotDot & ditto Offset sXYZMagVectors */
enum { Y_IDX = 1 }; /* as above */
enum { Z_IDX = 2 }; /* as above */

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

	/* configure LSM303DHLC accelerometer */
	BSP_ACCELERO_Reset();

	if (ACCELERO_OK != BSP_ACCELERO_Init()) {
		ErrorHandler();
		retVal = FCB_ERR_INIT;
		goto Exit;
	}


	LSM303DLHC_MagInit();

    /* do a pre-read to get the DRDY interrupts going. Since we trig on
     * rising flank and the sensor has data from power-on, by the time we get
     * here the interrupt is already high. Reading the data trigs the
     * sensor to load a new set of values into its registers.
     */
	BSP_ACCELERO_GetXYZ(sXYZDotDot);
	LSM303DLHC_MagReadXYZ(sXYZMagVector);
Exit:
	return retVal;
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

	BSP_ACCELERO_GetXYZ(sXYZDotDot);

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
	LSM303DLHC_MagReadXYZ(sXYZMagVector);
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
