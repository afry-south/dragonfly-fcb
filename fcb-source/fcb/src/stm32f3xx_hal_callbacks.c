/******************************************************************************
 * @file    stm32f3xx_hal_callbacks.c
 * @author  Dragonfly
 * @version v. 1.0.0
 * @date    2015-08-12
 * @brief   This module should implement various callback functions (such as
 * 			those triggered by different interrupts) from the STM32F3xx HAL
 * 			Driver library.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "receiver.h"
#include "fcb_sensors.h"
#include "fcb_gyroscope.h"
#include "fcb_accelerometer_magnetometer.h"
#include "fcb_error.h"
#include "usbd_cdc_if.h"
#include "uart.h"
#include "state_estimation.h"

#include "stm32f3_discovery.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "task_status.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint8_t UserButtonPressed;

/* Private function prototypes -----------------------------------------------*/

/* Called every system tick to drive the RTOS */
extern void xPortSysTickHandler(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch(GPIO_Pin) {
    case USER_BUTTON_PIN:
        UserButtonPressed++;
        if (UserButtonPressed > 0x7) {
            BSP_LED_Toggle(LED7);
            UserButtonPressed = 0x0;
        }
        break;
    case GPIO_GYRO_DRDY:
        FcbSendSensorMessageFromISR(FCB_SENSOR_GYRO_DATA_READY);
        break;
    case GPIO_ACCELEROMETER_DRDY:
        FcbSendSensorMessageFromISR(FCB_SENSOR_ACC_DATA_READY);
        break;
    case GPIO_MAGNETOMETER_DRDY:
        FcbSendSensorMessageFromISR(FCB_SENSOR_MAGNETO_DATA_READY);
        break;
    default:
        break;
    }
}

/**
 * @brief  PWR PVD interrupt callback
 * @param  none
 * @retval none
 */
void HAL_PWR_PVDCallback(void) {
	USBComSendString("Error: PVD\n");

	/* Voltage drop detected - Go to error handler */
	ErrorHandler();
}

/**
 * @brief  SYSTICK callback
 * @param  None
 * @retval None
 */
void HAL_SYSTICK_Callback(void) {
#if 1

	static uint32_t kicks = 0;
	kicks++;

	if ((kicks % 1000) == 0) {
		BSP_LED_Toggle(LED9);
	}
#endif
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
		xPortSysTickHandler();
	}

	HAL_IncTick();
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		PrimaryReceiverTimerPeriodCountIncrement();
	} else if (htim->Instance == AUX_RECEIVER_TIM) {
		AuxReceiverTimerPeriodCountIncrement();
	} else if (htim->Instance == TASK_STATUS_TIM){
		IncreaseTaskStatusTimerPeriodCount();
	} else if (htim->Instance == STATE_ESTIMATION_UPDATE_TIM){
	    StateEstimationTimeEventCallback();
    }
}

/**
 * @brief  Input Capture callback in non blocking mode
 * @param  htim : TIM IC handle
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == PRIMARY_RECEIVER_TIM) {
		if (htim->Channel == PRIMARY_RECEIVER_THROTTLE_ACTIVE_CHANNEL)
			UpdateReceiverThrottleChannel();
		else if (htim->Channel == PRIMARY_RECEIVER_AILERON_ACTIVE_CHANNEL)
			UpdateReceiverAileronChannel();
		else if (htim->Channel == PRIMARY_RECEIVER_ELEVATOR_ACTIVE_CHANNEL)
			UpdateReceiverElevatorChannel();
		else if (htim->Channel == PRIMARY_RECEIVER_RUDDER_ACTIVE_CHANNEL)
			UpdateReceiverRudderChannel();
	} else if (htim->Instance == AUX_RECEIVER_TIM) {
		if (htim->Channel == AUX_RECEIVER_GEAR_ACTIVE_CHANNEL)
			UpdateReceiverGearChannel();
		else if (htim->Channel == AUX_RECEIVER_AUX1_ACTIVE_CHANNEL)
			UpdateReceiverAux1Channel();
	}
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance == UART) {
        HandleUartTxCallback(UartHandle);
    }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance == UART) {
        HandleUartRxCallback(UartHandle);
    }
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    if(UartHandle->Instance == UART) {
        HandleUartErrorCallback(UartHandle);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/


