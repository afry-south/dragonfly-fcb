#include "gyroscope.h"
#include "stm32f3_discovery_gyroscope.h"
#include "trace.h"
#include "fcb_error.h"
#include "FreeRTOS.h"

// #define FCB_GYRO_DEBUG

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

uint8_t InitialiseGyroscope(void) {
    uint8_t retVal = FCB_OK;

    GPIO_InitTypeDef GPIO_InitStructure;
    BSP_LED_On(LED4);

    /* configure GYRO DRDY (data ready) interrupt */
    GYRO_CS_GPIO_CLK_ENABLE(); /* happens to be GPIOE */

    GPIO_InitStructure.Pin = GPIO_PIN_1; /* STM32F3 doc UM1570 page 27/36 */
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(EXTI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    BSP_GYRO_Reset();

    if(BSP_GYRO_Init() != HAL_OK)
    {
        /* Initialization Error */
        fcb_error();
    }

    GetAngleDotFromGyro(); /* necessary so a fresh DRDY can be triggered */
    return retVal;
}


void GetAngleDotFromGyro(void) {

#ifdef FCB_GYRO_DEBUG
    static uint32_t call_counter = 0;
#endif
    BSP_GYRO_GetXYZ(sGyroXYZAngleDot);

#ifdef FCB_GYRO_DEBUG
    if (call_counter % 200) {
        TRACE_SYNC("tim:%u sum xyzdot:%1.1f, %1.1f, %1.1f\n",
                   (uint32_t) call_counter,
                   sGyroXYZAngleDot[0],
                   sGyroXYZAngleDot[1],
                   sGyroXYZAngleDot[2]);
    }

    call_counter++;
#endif
}
