#include "df_error.h"
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

static void take_isr_semaphore(xTimerHandle xTimer ); /* tmrTIMER_CALLBACK */

static xSemaphoreHandle sUsrBtnPressed;
static xTaskHandle hTakeIsrSem;

enum { TIMER_INTERVAL_MS = 50 };
enum { DRAGON_TIMER_ID = 242 };

static uint32_t cbk_btn_counter = 0;
static uint32_t cbk_counter = 0;
static volatile uint8_t sens_init_done = 0;

#if 1
/**
  * @brief  Configures EXTI Line9-5 (connected to PE6 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI0_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __GPIOA_CLK_ENABLE();

  /* Configure User Button, connected to PE6 IOs in External Interrupt Mode with Rising edge trigger detection. */
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);


//  HAL_NVIC_SetPriority(EXTI0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
#endif


/**
 *
 */

void dragon_isr_semaphore(void) {
    uint8_t tmpreg = 0;
    uint8_t ctrl3 = 0;

#if 0
    EXTI0_Config();
#endif
    xSemaphoreHandle sNeverExit = xSemaphoreCreateBinary();

    portBASE_TYPE retVal = 0;
    vSemaphoreCreateBinary(sUsrBtnPressed);

    BSP_LED_On(LED5);

    if (pdPASS != (retVal =
                   xTaskCreate((pdTASK_CODE)take_isr_semaphore,
                               (signed portCHAR*)"tGyroRead",
                               4 * configMINIMAL_STACK_SIZE,
                               NULL /* parameter */,
                               1 /* priority */,
                               &hTakeIsrSem))) {
        df_error();
    }

    /* Start scheduler
     *
     * since we use heap1.c, we must create all tasks and queues before the OS kernel
     * is started according to ST UM1722 manual section 1.6.
     */
//    HAL_NVIC_SetPriorityGroup(NVIC_PRIORITYGROUP_4);
   // NVIC_PriorityGroupConfig(NVIC_PRIORITYGROUP_4);
    vTaskStartScheduler();


    /* We should never get here as control is now taken by the scheduler */
    if (pdTRUE != xSemaphoreTake(sNeverExit, portMAX_DELAY)) {
        // ERROR here
    }
}

#if 1

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    cbk_counter++;
    __IO uint32_t i =0;

    if (USER_BUTTON_PIN == GPIO_Pin) {
    	  if(GPIO_Pin == USER_BUTTON_PIN)
    	  {
    	    /* Delay */
    	    for(i=0; i<0x7FFFF; i++);
    	    /* Toggle LD3 */
    	    BSP_LED_Toggle(LED3);
    	    /* Toggle LD6 */
//    	    BSP_LED_Toggle(LED6);
    	    /* Toggle LD7 */
//    	    BSP_LED_Toggle(LED7);
    	    /* Toggle LD10 */
    	    BSP_LED_Toggle(LED10);
    	  }

        portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
        cbk_btn_counter++;

        if (0 == sens_init_done) {
            return;
        }
//        return; // when commented in, ISR doesn't get called repeatedly.
        if (pdTRUE != xSemaphoreGiveFromISR(sUsrBtnPressed,
                                            &higherPriorityTaskWoken)) {
            df_error();
        }
//        if (pdTRUE == higherPriorityTaskWoken) {
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
//        }
    }
}

#endif

static void take_isr_semaphore(xTimerHandle xTimer ) {
    enum { SEND_BUF_LEN = 64 };
    uint8_t send_buf[SEND_BUF_LEN] = { (uint8_t) '_'};
    static uint8_t flipflop = 0;
    static uint32_t pulse_counter = 0;
    int used_buf = 0;

    sens_init_done = 1;

    while (1) {
        if (pdTRUE != xSemaphoreTake(sUsrBtnPressed, portMAX_DELAY)) {
            df_error();
        }

        if (flipflop) {
            BSP_LED_Off(LED6);
            flipflop = 0;
        } else {
            BSP_LED_On(LED6);
            flipflop = 1;
        }

        TRACE_SYNC("usr btn flipflop:%u pulse_cnt:%u cbk_btn:%u", flipflop, pulse_counter,cbk_btn_counter);

        ++pulse_counter;
    }
}

void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed char *pcTaskName ) {
	df_error();
}

