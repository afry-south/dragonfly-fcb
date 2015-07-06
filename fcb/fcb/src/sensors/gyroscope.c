#include "gyroscope.h"
#include "stm32f3_discovery_gyroscope.h"
#include "trace.h"
#include "fcb_error.h"
#include "FreeRTOS.h"

#define FCB_GYRO_DEBUG

/**
 * @todo in the ideal world we shouldn't read the gyro data
 * in the ISR itself, but in a dedicated thread.
 *
 * At the time of writing, giving a semaphore from GyroHandleDataReady
 * (which is called from an ISR) doesn't work, FreeRTOS hangs.
 *
 * So this is an interim solution.
 */
#define READ_GYRO_FROM_ISR

/**
 * Angular velocities, in degrees.
 *
 */
static float sGyroXYZAngleDot[3] = { 0.0, 0.0, 0.0 };
static uint8_t gyro_init_done = 0;

uint8_t InitialiseGyroscope(void) {
    uint8_t ret_val = FCB_OK;

    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    GYRO_CS_GPIO_CLK_ENABLE(); /* happens to be GPIOE */

    GPIO_InitStructure.Pin = GPIO_PIN_1; /* pin PE.01 see user manual */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    if (GYRO_OK != (ret_val = BSP_GYRO_Init())) {
        /* no log; BSP_GYRO_INIT returns either 0 (success) or 1 */
        fcb_error();
        ret_val = FCB_ERR_INIT;
    }


    gyro_init_done = 1;

    return ret_val;
}

/*
 * As settings are in BSP_GYRO_Init, the callback is called with a frequency
 * of 94.5 Hz according to oscilloscope.
 */
void GyroHandleDataReady(void) {
#ifdef FCB_GYRO_DEBUG
	static uint32_t count = 10;
	count++;
	BSP_LED_On(LED7);

	if ((count % 10) == 0) {
		BSP_LED_Toggle(LED5);
	}
#endif
	if (1) {
#ifdef READ_GYRO_FROM_ISR
    BSP_GYRO_GetXYZ(sGyroXYZAngleDot);
#else
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (pdTRUE != xSemaphoreGiveFromISR(sGyroDataReady,
                                        &higherPriorityTaskWoken)) {
        fcb_error();
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
#endif
	}
}
