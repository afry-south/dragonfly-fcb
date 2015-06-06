#include "df_error.h"


#include "stm32f3xx.h"
#include "stm32f3_discovery.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3_discovery_gyroscope.h"
#include "usbd_cdc_if.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "stdio.h"

static GPIO_InitTypeDef  GPIO_InitStruct;


static void dragon_timer_read_sensors(xTimerHandle xTimer ); /* tmrTIMER_CALLBACK */
static xTimerHandle xDragonTimer = NULL;

enum { TIMER_INTERVAL_MS = 100 };
enum { DRAGON_TIMER_ID = 242 };

/**
 *
 */

void dragon_sensors(void) {
	xSemaphoreHandle sNeverExit = xSemaphoreCreateBinary();
	BSP_LED_On(LED3);

	/* configure pin PD9 for GPIO toggling */

	 __GPIOD_CLK_ENABLE();


   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   if(BSP_GYRO_Init() != HAL_OK)
   {
       /* Initialization Error */
       df_error();
   }


	if (0 == (xDragonTimer = xTimerCreate((const signed char*)"tmrDragon",
			TIMER_INTERVAL_MS / portTICK_RATE_MS,
			pdTRUE, /* uxAutoReload */
			(void*)DRAGON_TIMER_ID, /* pvTimerID */
			dragon_timer_read_sensors))) {
		BSP_LED_On(LED10);
		return;
	}

	if (pdFAIL == xTimerStart(xDragonTimer, 0)) {
		BSP_LED_On(LED10);
		return;
	}

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


static void dragon_timer_read_sensors(xTimerHandle xTimer ) {
	enum { SEND_BUF_LEN = 64 };
	uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
	static uint8_t flipflop = 0;
	static uint32_t pulse_counter = 0;
	int used_buf = 0;
  float gyro_xyz_dot_buf[3] = { 0.0, 0.0, 0.0 }; /* mdps, mdps, mdps */
  static float gyro_xyz_dot_mean_buf[3] = { 0 }; /* compensate zero-rate level */
  static float gyro_xyz[3] = { 0 }; /* compensate zero-rate level */
  int sample_num = 1000 / TIMER_INTERVAL_MS * 60;
  uint8_t print_this = 0;
  uint8_t i = 0;
  static uint8_t xdot_or_x = 0; /* 0 = xdot, 1 = x */

	if (flipflop) {
		BSP_LED_Off(LED6);
		flipflop = 0;
	} else {
		BSP_LED_On(LED6);
		flipflop = 1;
	}

  BSP_GYRO_GetXYZ(gyro_xyz_dot_buf);

  if (pulse_counter % (2000 / TIMER_INTERVAL_MS) == 0) {
	  print_this = 1;
  }

  if (pulse_counter < sample_num) {
	  gyro_xyz_dot_mean_buf[0] += gyro_xyz_dot_buf[0] / sample_num;
	  gyro_xyz_dot_mean_buf[1] += gyro_xyz_dot_buf[1] / sample_num;
	  gyro_xyz_dot_mean_buf[2] += gyro_xyz_dot_buf[2] / sample_num;

	  if (print_this) {
	      used_buf += snprintf((char*)(send_buf+used_buf),
	                       SEND_BUF_LEN,
	                       "tim:%u mean xyzdot:%1.1f, %1.1f, %1.1f\n",
	                       (uint) pulse_counter,
	                       gyro_xyz_dot_mean_buf[0],
	                       gyro_xyz_dot_mean_buf[1],
	                       gyro_xyz_dot_mean_buf[2]);
	  }
  } else {

      for (i = 0; i < 3; i++) {
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

  if (print_this) {
      CDC_Transmit_FS(send_buf, (used_buf >= SEND_BUF_LEN) ? SEND_BUF_LEN : used_buf + 1);
  }
  ++pulse_counter;
}
