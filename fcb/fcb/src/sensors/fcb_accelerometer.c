/**
 * @file fcb_accelerometer.c
 *
 * Implements fcb_accelerometer.h API
 *
 * @see fcb_accelerometer.h
 */
#include "fcb_accelerometer.h"
#include "fcb_error.h"
#include "stm32f3_discovery_accelerometer.h"

#include "FreeRTOS.h"

#define FCB_ACCMAG_DEBUG
#define FCB_USE_ACC_DRDY_INT1

int16_t sXYZDotDot[] = { 0, 0 , 0 };

enum { XDOTDOT_IDX = 0 }; /* index into sGyroXYZDotDot & ditto Offset */
enum { YDOTDOT_IDX = 1 }; /* as above */
enum { ZDOTDOT_IDX = 2 }; /* as above */


/* public fcn definitions */

uint8_t FcbInitialiseAccelerometer(void) {
    uint8_t retVal = FCB_OK;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* configure STM32 interrupts & GPIO */
    ACCELERO_DRDY_GPIO_CLK_ENABLE(); /* GPIOE clock */
#ifdef FCB_USE_ACC_DRDY_INT1
    GPIO_InitStructure.Pin = GPIO_PIN_4; /* STM32F3 doc UM1570 page 27/36 */
#else
    GPIO_InitStructure.Pin = GPIO_PIN_2; /* STM32F3 doc UM1570 page 27/36 */
#endif
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

#ifdef FCB_USE_ACC_DRDY_INT1
    HAL_NVIC_SetPriority(EXTI4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
#else
    HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);
#endif
    /* configure LSM303DHLC accelerometer */
    BSP_ACCELERO_Reset();

    if (0 != BSP_ACCELERO_Init()) {
        fcb_error();
        retVal = FCB_ERR_INIT;
        goto Exit;
    }


    BSP_ACCELERO_GetXYZ(sXYZDotDot);
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
#endif

    BSP_ACCELERO_GetXYZ(sXYZDotDot);
}


void GetAcceleration(int16_t * xDotDot, int16_t * yDotDot, int16_t * zDotDot) {
	*xDotDot = sXYZDotDot[XDOTDOT_IDX];
	*yDotDot = sXYZDotDot[YDOTDOT_IDX];
	*zDotDot = sXYZDotDot[ZDOTDOT_IDX];
}
