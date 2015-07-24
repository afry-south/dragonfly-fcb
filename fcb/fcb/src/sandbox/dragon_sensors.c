#define COMPILE_THIS

#include "fcb_error.h"
#include "trace.h"


#include "stm32f3xx.h"
#include "stm32f3_discovery.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3_discovery_gyroscope.h"
#include "usbd_cdc_if.h"



#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "stdio.h"


#ifdef COMPILE_THIS

#define SEM_VERSION

#define READ_GYRO_FROM_THR

#define ISR_DELEGATE_FCN

// #define READ_GYRO_FROM_ISR

#define LAUNCH_TASK_SCHEDULER

// #define LAUNCH_TIMER

// #define PRINT_SOMETHING


/*
 * Defining this makes a difference whether or not timer is launched.
 * When defined, then DRDY callbacks show on LEDs (timer launched)
 * When not defined DRDY callback doesn't show on LEDs. (timer not launched)
 *
 * (gyro read from ISR)
 *
 *   DRDYLED     UNE
 *              0   1
 *
 *   TIMER 0    0   1
 *         1    0   1
 *
 * Lighting LED10 if scheduler should exit never shows in any combination.
 */
#define USE_NEVER_EXIT

/*
 * trace variable via DAC output
 */
#define DAC_OUTPUT


static void dragon_timer_read_sensors(xTimerHandle xTimer ); /* tmrTIMER_CALLBACK */
#ifndef SEM_VERSION
static xTimerHandle xDragonTimer = NULL;
#endif

#ifdef SEM_VERSION
static xSemaphoreHandle sGyroDataReady = NULL;
static xTaskHandle hGyroDataRead;
#endif

enum { TIMER_INTERVAL_MS = 50 };
enum { DRAGON_TIMER_ID = 242 };

static volatile uint32_t cbk_gyro_counter = 0;
static volatile uint32_t cbk_counter = 0;
static volatile float gyro_xyz_dot_buf[3] = { 0.0, 0.0, 0.0 };
static volatile uint8_t sens_init_done = 0;

#endif

#ifdef DAC_OUTPUT
#include "stm32f3xx_hal_dac.h"

static DAC_HandleTypeDef sDacHandle;
static DAC_ChannelConfTypeDef sDacChannelOneConf = { DAC_TRIGGER_NONE, DAC_OUTPUTBUFFER_DISABLE };


void dragon_dac_init(void);
static uint32_t dac1_scaled = 0;

#define DAC_OUTPUT_XDOT_FRACTION

// #define DAC_OUTPUT_X_FRACTION

#endif

void dragon_gyro_init(void) {
    uint8_t tmpreg = 0;

    GPIO_InitTypeDef GPIO_InitStructure;
    BSP_LED_On(LED4);

    /* configure GYRO DRDY (data ready) interrupt */
    GYRO_CS_GPIO_CLK_ENABLE(); /* happens to be GPIOE */

    /*    GPIO_InitStructure.Pin = GYRO_INT2_PIN; */
    GPIO_InitStructure.Pin = GPIO_PIN_1;
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

    /* not sure if L3GD20_EnableIT says the right thing */
    GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG3_ADDR,1);
    TRACE_SYNC("L3GD20_CTRL_REG3:0x%02x", tmpreg);

#ifdef SEM_VERSION
    BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);
#endif

}

#ifdef COMPILE_THIS

/**
 *
 */

void dragon_sensors(void) {
    uint8_t tmpreg = 0;
    uint8_t ctrl3 = 0;
#ifdef USE_NEVER_EXIT
    xSemaphoreHandle sNeverExit = xSemaphoreCreateBinary();
#endif /* USE_NEVER_EXIT */
#ifdef SEM_VERSION
    portBASE_TYPE retVal = 0;
    sGyroDataReady = xSemaphoreCreateBinary();

    if (NULL == sGyroDataReady) {
        fcb_error();
    }

#endif

#ifdef DAC_OUTPUT
    dragon_dac_init();
#endif

#ifdef SEM_VERSION
    if (pdPASS != (retVal =
                   xTaskCreate((pdTASK_CODE)dragon_timer_read_sensors,
                               (signed portCHAR*)"tGyroRead",
                               4 * configMINIMAL_STACK_SIZE,
                               NULL /* parameter */,
                               1 /* priority */,
                               &hGyroDataRead))) {
        fcb_error();
    }
#else
    dragon_gyro_init();
    /* not sure if L3GD20_EnableIT says the right thing */
    GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG3_ADDR,1);
    TRACE_SYNC("L3GD20_CTRL_REG3:0x%02x", tmpreg);

#ifdef LAUNCH_TIMER
    if (0 == (xDragonTimer = xTimerCreate((const signed char*)"tmrDragon",
                                          TIMER_INTERVAL_MS / portTICK_RATE_MS,
                                          pdTRUE, /* uxAutoReload */
                                          (void*)DRAGON_TIMER_ID, /* pvTimerID */
                                          dragon_timer_read_sensors))) {
        fcb_error();
        return;
    }

    if (pdFAIL == xTimerStart(xDragonTimer, 0)) {
        fcb_error();
        return;
    }
#endif /* LAUNCH_TIMER */
    sens_init_done = 1;
#endif /* SEM_VERSION */

#ifdef LAUNCH_TASK_SCHEDULER
    /* Start scheduler
     *
     * since we use heap1.c, we must create all tasks and queues before the OS kernel
     * is started according to ST UM1722 manual section 1.6.
     */
    vTaskStartScheduler();
#ifdef USE_NEVER_EXIT
    /* We should never get here as control is now taken by the scheduler */
    if (pdTRUE != xSemaphoreTake(sNeverExit, portMAX_DELAY)) {
        // ERROR here
    }
#endif /* USE_NEVER_EXIT */
#endif /* LAUNCH_TASK_SCHEDULER */

}

#ifdef ISR_DELEGATE_FCN

extern void dragon_sensors_isr(void) {
    if ((cbk_gyro_counter % 48) == 0) {
        BSP_LED_Toggle(LED5);
    }

    if (0 == sens_init_done) {
        return;
    }

    cbk_gyro_counter++;
#ifdef SEM_VERSION
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (pdTRUE != xSemaphoreGiveFromISR(sGyroDataReady,
                                        &higherPriorityTaskWoken)) {
        fcb_error();
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
#endif /* SEM_VERSION */

#ifdef READ_GYRO_FROM_ISR
    BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);
#endif /* READ_GYRO_FROM_ISR */
}
#endif

static void dragon_timer_read_sensors(xTimerHandle xTimer ) {
    enum { SEND_BUF_LEN = 64 };
    uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
    static uint32_t pulse_counter = 0;
    int used_buf = 0;
    static float gyro_xyz_dot_mean_buf[3] = { 0 }; /* compensate zero-rate level */
    static float gyro_xyz[3] = { 0 }; /* compensate zero-rate level */
#ifdef SEM_VERSION
    int sample_num = 4000;
#else
    int sample_num = 1000 / TIMER_INTERVAL_MS * 60;
#endif
    uint8_t print_this = 0;
    uint8_t i = 0;
    uint8_t tmpreg = 0;
    static uint8_t xdot_or_x = 0; /* 0 = xdot, 1 = x */

#ifdef SEM_VERSION
    dragon_gyro_init();
    sens_init_done = 1;
    GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
    TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);


    while (1) {
#if 0
        GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
        TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);
#endif
        if (pdTRUE != xSemaphoreTake(sGyroDataReady, portMAX_DELAY)) {
            fcb_error();
        }
#endif /* SEM_VERSION */


#ifdef READ_GYRO_FROM_THR
        BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);
#endif


#ifdef SEM_VERSION
        if ((pulse_counter % (500)) == 0) {
            BSP_LED_Toggle(LED6);
            print_this = 1;
        } else {
            print_this = 0;
        }
#else
        if (pulse_counter % (2000 / TIMER_INTERVAL_MS) == 0) {
            BSP_LED_Toggle(LED6);

            print_this = 1;
        }
#endif


#if 0
        if (print_this) {
            GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
            TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);

            TRACE_SYNC("tim:%u cbkg:%u xdot:%1.1f\n",
                       (uint) pulse_counter,
                       cbk_gyro_counter,
                       gyro_xyz_dot_buf[0]);
        }
#else
        if (pulse_counter < sample_num) {
            gyro_xyz_dot_mean_buf[0] += gyro_xyz_dot_buf[0];
            gyro_xyz_dot_mean_buf[1] += gyro_xyz_dot_buf[1];
            gyro_xyz_dot_mean_buf[2] += gyro_xyz_dot_buf[2];

            if (print_this) {
#ifdef PRINT_SOMETHING
                TRACE_SYNC("tim:%u sum xyzdot:%1.1f, %1.1f, %1.1f\n",
                           (uint) pulse_counter,
                           gyro_xyz_dot_mean_buf[0],
                           gyro_xyz_dot_mean_buf[1],
                           gyro_xyz_dot_mean_buf[2]);
#endif /* PRINT_SOMETHING */

            }
        } else if (pulse_counter == sample_num) {
            for (i = 0; i < 3; i++) {
                gyro_xyz_dot_mean_buf[i] = gyro_xyz_dot_mean_buf[i] / sample_num ;
            }
        } else {
            float dt  = 0;

#ifdef LAUNCH_TIMER
            dt = TIMER_INTERVAL_MS / 1000;
#endif

#ifdef READ_GYRO_FROM_THR
            dt = (float)(1.0 / 189.5);
#endif /* READ_GYRO_FROM_THR */

#ifdef DAC_OUTPUT_XDOT_FRACTION

            {
                /* for L3GD20_SENSITIVITY_500DPS */

                float xdot_fsv_fraction = ((gyro_xyz_dot_buf[0] - gyro_xyz_dot_mean_buf[i]) + 500000) / 1000000;
                dac1_scaled = (xdot_fsv_fraction * 4096);
            }
#endif /* DAC_OUTPUT_XDOT_FRACTION */

            for (i = 0; i < 3; i++) {
                /* integrating the values */
                gyro_xyz[i] += ((gyro_xyz_dot_buf[i] - gyro_xyz_dot_mean_buf[i]) * dt);

                /* normalising the angle to +-180 circle */
                if (gyro_xyz[i] > 1800000) {
                    gyro_xyz[i] -= 360000;
                } else if (gyro_xyz[i] < -180000) {
                    gyro_xyz[i] += 360000;
                }

            }
#ifdef PRINT_SOMETHING
            if (print_this) {
                if (xdot_or_x == 0) {
                    xdot_or_x = 1;
                    TRACE_SYNC("tim:%u xdot:%0.1f ydot:%0.1f zdot:%0.1f\n",
                               (uint) pulse_counter,
                               gyro_xyz_dot_buf[0] - gyro_xyz_dot_mean_buf[0],
                               gyro_xyz_dot_buf[1] - gyro_xyz_dot_mean_buf[1],
                               gyro_xyz_dot_buf[2] - gyro_xyz_dot_mean_buf[2]);
                } else {
                    xdot_or_x = 0;
                    TRACE_SYNC("tim:%u x:%0.1f y:%0.1f z:%0.1f\n",
                               (uint) pulse_counter,
                               gyro_xyz[0],
                               gyro_xyz[1],
                               gyro_xyz[2]);
                }
            }
#endif /* PRINT_SOMETHING */
        }
#endif

#ifdef DAC_OUTPUT
        {
            HAL_StatusTypeDef halRetVal = HAL_OK;

#ifdef DAC_OUTPUT_X_FRACTION
            /* calculate the +-180 deg circle as a fraction of fsv */

            float x_fsv_fraction = (gyro_xyz[0] + 180000) / 360000;
            dac1_scaled = (x_fsv_fraction * 4096);
#endif /* DAC_OUTPUT_X_FRACTION */

            if (HAL_OK != (halRetVal = HAL_DAC_SetValue(&sDacHandle /* DAC_HandleTypeDef* hdac */,
                                                        DAC1_CHANNEL_1 /* uint32_t channel */,
                                                        DAC_ALIGN_12B_R /* uint32_t alignment */, /* 0 - 4095 */
                                                        dac1_scaled /* uint32_t data */))) {
                fcb_error();
            }
        }
#endif /* DAC_OUTPUT */

        ++pulse_counter;
#ifdef SEM_VERSION
    }
#endif
}

#ifdef DAC_OUTPUT
void dragon_dac_init(void) {
    HAL_StatusTypeDef halRetVal = HAL_OK;
    GPIO_InitTypeDef          GPIO_InitStruct;

    /* DAC init sequence borrowed from stm32f3xx_hal_dac.c */
    sDacHandle.Instance = DAC1;

    if ((halRetVal = HAL_DAC_Init(&sDacHandle)) != HAL_OK) {
        /* Initiliazation Error */
        fcb_error();
    }

    /* set PA4 to analog mode */

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO clock ****************************************/
    __GPIOA_CLK_ENABLE();
    /* DAC Periph clock enable */
    __DAC1_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* DAC Channel1 GPIO pin configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* returns HAL_StatusTypeDef */
    if (HAL_OK != (halRetVal = HAL_DAC_ConfigChannel(&sDacHandle /* DAC_HandleTypeDef* hdac */,
                                                     &sDacChannelOneConf /* DAC_ChannelConfTypeDef* sConfig */,
                                                     DAC1_CHANNEL_1 /* uint32_t channel */))) {
        fcb_error();
    }

    if (HAL_OK != (halRetVal = HAL_DAC_SetValue(&sDacHandle /* DAC_HandleTypeDef* hdac */,
                                                DAC1_CHANNEL_1 /* uint32_t channel */,
                                                DAC_ALIGN_12B_R /* uint32_t alignment */, /* 0 - 4095 */
                                                0xFFF /* uint32_t data */))) {
        fcb_error();
    }

    if (HAL_OK != (halRetVal = HAL_DAC_Start(&sDacHandle, DAC1_CHANNEL_1 /* uint32_t channel */))) {
        fcb_error();
    }

}
#endif /* DAC_OUTPUT */

#endif /* COMPILE_THIS */
