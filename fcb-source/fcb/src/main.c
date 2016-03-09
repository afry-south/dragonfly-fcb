/******************************************************************************
 * @file    main.c
 * @brief   Flight Control program for the Dragonfly quadcopter
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f3_discovery.h"
#include "common.h"
#include "motor_control.h"
#include "flight_control.h"
#include "rotation_transformation.h"
#include "pid_control.h"
#include "receiver.h"
#include "task_status.h"
#include "usbd_cdc_if.h"
#include "com_cli.h"
#include "fcb_error.h"
#include "fcb_retval.h"
#include "fcb_sensors.h"
#include "fcb_gyroscope.h"
#include "state_estimation.h"
#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void InitSystem(void);
static void InitRTOS(void);
static void ConfigSystemClock(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void) {
	/* At this stage the microcontroller clock setting is already configured,
	 * this is done through SystemInit() function which is called from startup
	 * file (startup_stm32f303.c) before to branch to application main.
	 * To reconfigure the default setting of SystemInit() function, refer to
	 * system_stm32f30x.c file
	 */

	/* Init system at low level */
	InitSystem();

	/* Initialize RTOS tasks */
	InitRTOS();

	while (1);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initializes system at low level, setting up clocks and configuring
 *         peripheral operation.
 * @param  None
 * @retval None
 */
static void InitSystem(void) {
	/* STM32F3xx HAL library initialization:
	 - Configure the Flash prefetch
	 - Systick timer is configured by default as source of time base, but user
	 can eventually implement his proper time base source (a general purpose
	 timer for example or other time source), keeping in mind that Time base
	 duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
	 handled in milliseconds basis.
	 - Set NVIC Group Priority to 4
	 - Low Level Initialization
	 */
	HAL_Init();

	/* Initialize the CRC peripheral */
	InitCRC();

	/* Initialize Programmable Voltage Detection (PVD) */
	ConfigPVD();

	/* Configure the system clock to 72 Mhz */
	ConfigSystemClock();

	/* Initialize Command Line Interface for USB communication */
	RegisterCLICommands();

	/* Configure UART */
	UartConfig();

	/* Init on-board LEDs */
	InitLEDs();

	/* Init User button */
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Init sensor reading */
	if (FCB_OK != FcbSensorsConfig()) {
		ErrorHandler();
	}

	/* Init the states for the Kalman filter */
	InitStatesXYZ();

	/* Init the rotation matrix */
	InitRotationMatrix();

	/* Initialize PID control variables */
	InitPIDControllers();

	/* Setup motor output timer */
	MotorControlConfig();

	/* Setup receiver timers for receiver input */
	ReceiverInputConfig();

	/* Setup timer for task status command*/
	InitMonitoring();
}

/**
 * @brief  Initializes the Real-time operating system (RTOS) and starts the
 *         OS kernel
 * @param  None
 * @retval None
 */
static void InitRTOS(void) {
	/* # CREATE THREADS ####################################################### */
	CreateFlightControlTask();
#if defined(USE_USB_COM)
	CreateUSBComTasks();
#endif
	CreateUARTComTasks();

	/* # CREATE QUEUES ######################################################## */
#if defined(USE_USB_COM)
	CreateUSBComQueues();
#endif
	CreateUARTComQueues();

	/* # CREATE SEMAPHORES #################################################### */
	CreateCLISemaphores();
#if defined(USE_USB_COM)
	CreateUSBComSemaphores();
#endif
	CreateUARTComSemaphores();

	/* # Start the RTOS scheduler #############################################
	 * Since we use heap1.c, we must create all tasks and queues before the OS kernel
	 * is started according to ST UM1722 manual section 1.6.
	 */
	vTaskStartScheduler();
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV                     = 1
 *            PLLMUL                         = RCC_PLL_MUL9 (9)
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
static void ConfigSystemClock(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Configures the USB clock */
	HAL_RCCEx_GetPeriphCLKConfig(&RCC_PeriphClkInit);
	RCC_PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5; // 72/1.5 = 48 MHz USB clock
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	 clocks dividers */
	RCC_ClkInitStruct.ClockType =
			(RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1 is limited to 36 MHz according to reference manual
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */
/*****END OF FILE****/
