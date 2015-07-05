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

// #define SEM_VERSION

// #define READ_GYRO_FROM_THR
#define READ_GYRO_FROM_ISR

static void dragon_timer_read_sensors(xTimerHandle xTimer ); /* tmrTIMER_CALLBACK */
#ifndef SEM_VERSION
static xTimerHandle xDragonTimer = NULL;
#endif

#ifdef SEM_VERSION
static xSemaphoreHandle sGyroDataReady;
static xTaskHandle hGyroDataRead;
#endif

enum { TIMER_INTERVAL_MS = 50 };
enum { DRAGON_TIMER_ID = 242 };

static uint32_t cbk_gyro_counter = 0;
static uint32_t cbk_counter = 0;
static float gyro_xyz_dot_buf[3] = { 0.0, 0.0, 0.0 };
static volatile uint8_t sens_init_done = 0;


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
}

/**
 *
 */

void dragon_sensors(void) {
    uint8_t tmpreg = 0;
    uint8_t ctrl3 = 0;
    xSemaphoreHandle sNeverExit = xSemaphoreCreateBinary();
#ifdef SEM_VERSION
	portBASE_TYPE retVal = 0;
    vSemaphoreCreateBinary(sGyroDataReady);
#endif
    dragon_gyro_init();
    /* not sure if L3GD20_EnableIT says the right thing */
    GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG3_ADDR,1);
    TRACE_SYNC("L3GD20_CTRL_REG3:0x%02x", tmpreg);

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
    sens_init_done = 1;
#endif


    /* Start scheduler
     *
     * since we use heap1.c, we must create all tasks and queues before the OS kernel
     * is started according to ST UM1722 manual section 1.6.
     */
    vTaskStartScheduler();


    /* We should never get here as control is now taken by the scheduler */
    if (pdTRUE != xSemaphoreTake(sNeverExit, portMAX_DELAY)) {
        // ERROR here
    }
}

#if 1
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    cbk_counter++;
#error BANG
    if ((cbk_counter % 48) == 0) {
    	BSP_LED_Toggle(LED5);
    }

    if (0 == sens_init_done) {
    	return;
    }
    if (GPIO_PIN_1 == GPIO_Pin) {
    	cbk_gyro_counter++;
#ifdef SEM_VERSION
        portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
    	cbk_gyro_counter++;

        if (pdTRUE != xSemaphoreGiveFromISR(sGyroDataReady,
        		&higherPriorityTaskWoken)) {
            fcb_error();
        }
       	portYIELD_FROM_ISR(higherPriorityTaskWoken);
#endif
#ifdef READ_GYRO_FROM_ISR
    	BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);
#endif
    }
}
#endif

static void dragon_timer_read_sensors(xTimerHandle xTimer ) {
    enum { SEND_BUF_LEN = 64 };
    uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
    static uint8_t flipflop = 0;
    static uint32_t pulse_counter = 0;
    int used_buf = 0;
    static float gyro_xyz_dot_mean_buf[3] = { 0 }; /* compensate zero-rate level */
    static float gyro_xyz[3] = { 0 }; /* compensate zero-rate level */
    int sample_num = 1000 / TIMER_INTERVAL_MS * 60;
    uint8_t print_this = 0;
    uint8_t i = 0;
    uint8_t tmpreg = 0;
    static uint8_t xdot_or_x = 0; /* 0 = xdot, 1 = x */

#ifdef SEM_VERSION
    sens_init_done = 1;
    GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
    TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);

	BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);

    while (1) {
#if 0
        GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
        TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);
#endif
        if (pdTRUE != xSemaphoreTake(sGyroDataReady, portMAX_DELAY)) {
            fcb_error();
        }
#endif

#ifdef READ_GYRO_FROM_THR
    	BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);
#endif

    	if (flipflop) {
        BSP_LED_Off(LED6);
        flipflop = 0;
    } else {
        BSP_LED_On(LED6);
        flipflop = 1;
    }

#ifdef SEM_VERSION
    if ((pulse_counter % (500)) == 0) {
       print_this = 1;
    }
#else
    if (pulse_counter % (2000 / TIMER_INTERVAL_MS) == 0) {
        print_this = 1;
    }
#endif

#if 1
	  if (print_this) {
//          GYRO_IO_Read(&tmpreg,L3GD20_STATUS_REG_ADDR,1);
//		  TRACE_SYNC("L3GD20_STATUS_REG:0x%02x", tmpreg);

	      used_buf += snprintf((char*)(send_buf+used_buf),
                             SEND_BUF_LEN,
                             "tim:%u cbk:%u cbkg:%u xyzdot:%1.1f, %1.1f, %1.1f\n",
                             (uint) pulse_counter,
                             cbk_counter,
                             cbk_gyro_counter,
                             gyro_xyz_dot_buf[0],
                             gyro_xyz_dot_buf[1],
                             gyro_xyz_dot_buf[2]);
	  }
#else

    if (pulse_counter < sample_num) {
        gyro_xyz_dot_mean_buf[0] += gyro_xyz_dot_buf[0];
        gyro_xyz_dot_mean_buf[1] += gyro_xyz_dot_buf[1];
        gyro_xyz_dot_mean_buf[2] += gyro_xyz_dot_buf[2];

        if (print_this) {
            used_buf += snprintf((char*)(send_buf+used_buf),
                                 SEND_BUF_LEN,
                                 "tim:%u sum xyzdot:%1.1f, %1.1f, %1.1f\n",
                                 (uint) pulse_counter,
                                 gyro_xyz_dot_mean_buf[0],
                                 gyro_xyz_dot_mean_buf[1],
                                 gyro_xyz_dot_mean_buf[2]);
        }
    } else if (pulse_counter == sample_num) {
        for (i = 0; i < 3; i++) {
            gyro_xyz_dot_mean_buf[i] = gyro_xyz_dot_mean_buf[i] / sample_num ;
        }
    } else {

        for (i = 0; i < 3; i++) {
            /* integrating the values */
            gyro_xyz[i] += ((gyro_xyz_dot_buf[i] - gyro_xyz_dot_mean_buf[i]) * TIMER_INTERVAL_MS / 1000);
        }

        if (print_this) {
            if (xdot_or_x == 0) {
                xdot_or_x = 1;
                used_buf += snprintf((char*)(send_buf+used_buf),
                                     SEND_BUF_LEN,
                                     "tim:%u xdot:%0.1f ydot:%0.1f zdot:%0.1f\n",
                                     (uint) pulse_counter,
                                     gyro_xyz_dot_buf[0] - gyro_xyz_dot_mean_buf[0],
                                     gyro_xyz_dot_buf[1] - gyro_xyz_dot_mean_buf[1],
                                     gyro_xyz_dot_buf[2] - gyro_xyz_dot_mean_buf[2]);
            } else {
                xdot_or_x = 0;
                used_buf += snprintf((char*)(send_buf+used_buf),
                                     SEND_BUF_LEN,
                                     "tim:%u x:%0.1f y:%0.1f z:%0.1f\n",
                                     (uint) pulse_counter,
                                     gyro_xyz[0],
                                     gyro_xyz[1],
                                     gyro_xyz[2]);

            }
        }
    }
#endif

    if (print_this) {
        CDC_Transmit_FS(send_buf, (used_buf >= SEND_BUF_LEN) ? SEND_BUF_LEN : used_buf + 1);
    }
    ++pulse_counter;
#ifdef SEM_VERSION
    }
#endif
}

#if 0
void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName ) {
	fcb_error();
}
#endif
