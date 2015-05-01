/******************************************************************************
 * @file    fcb/main.c
 * @author  ÅF Dragonfly
 * Daniel Stenberg, Embedded Systems
 * @version v. 1.0.0
 * @date    2015-05-01
 * @brief   Dragonfly template project demonstration. Makes use of the board
 *          user push button and LED circle
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef hUSBDDevice;
__IO uint8_t UserButtonPressed;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Init_System(void);
static void Init_LEDs(void);
static void LEDs_Off(void);
static void ToggleLEDs();

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
  /* At this stage the microcontroller clock setting is already configured,
   * this is done through SystemInit() function which is called from startup
   * file (startup_stm32f303.c) before to branch to application main.
   * To reconfigure the default setting of SystemInit() function, refer to
   * system_stm32f30x.c file
   */

  Init_System();

  HAL_Delay(1000); // A 1 second delay

  // Infinite loop keeps the program alive.
  while (1)
    {
      ToggleLEDs();
    }
}

static void Init_System(void)
{
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

  /* Configure the system clock to 72 Mhz */
  SystemClock_Config();

  /* Init on-board LEDs */
  Init_LEDs();

  /* Init User button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

static void Init_LEDs(void)
{
  /* Initialize LEDs and User Button available on STM32F3-Discovery board */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);

  LEDs_Off();
}

/**
  * @brief  Turns off all the LEDs
  * @param  None
  * @retval None
  */
static void LEDs_Off(void)
{
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
  BSP_LED_Off(LED7);
  BSP_LED_Off(LED8);
  BSP_LED_Off(LED9);
  BSP_LED_Off(LED10);
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
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphClkInit;

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
  RCC_PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;   // 72/1.5 = 48 MHz USB clock
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;     // APB1 is limited to 36 MHz according to reference manual
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief Toggles the LEDs based on User Button presses
  * @param None
  * @retval None
  */
void ToggleLEDs(void)
{
  switch(UserButtonPressed)
  {
  case 0:
    LEDs_Off();
    BSP_LED_On(LED3);
    break;

  case 1:
    LEDs_Off();
    BSP_LED_On(LED4);
    break;

  case 2:
    LEDs_Off();
    BSP_LED_On(LED5);
    break;

  case 3:
    LEDs_Off();
    BSP_LED_On(LED6);
    break;

  case 4:
    LEDs_Off();
    BSP_LED_On(LED7);
    break;

  case 5:
    LEDs_Off();
    BSP_LED_On(LED8);
    break;

  case 6:
    LEDs_Off();
    BSP_LED_On(LED9);
    break;

  case 7:
    LEDs_Off();
    BSP_LED_On(LED10);
    break;

  default:
    break;
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
   UserButtonPressed++;
   if (UserButtonPressed > 0x7)
    {
      UserButtonPressed = 0x0;
    }
  }
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
