/******************************************************************************
 * @file    fcb/main.c
 * @author  ÅF Dragonfly
 * Daniel Nilsson, Embedded Systems
 * Daniel Stenberg, Embedded Systems
 * @version v. 0.0.2
 * @date    2015-04-12
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor_control.h"
#include "flight_control.h"
#include "sensors.h"
#include "RCinput.h"
#include "com.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Button_TypeDef UserButton;

/* Private functions -----------------------------------------------*/
static void Init_System(void);
static void Init_LEDs(void);

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

  HAL_Init();
  Init_System();

  // Infinite loop keeps the program alive.
  while (1)
    {
      BSP_LED_On(LED3);
      BSP_LED_On(LED6);
    }
}

static void Init_System(void)
{
  /* Init on-board LEDs */
  Init_LEDs();

  /* Init User button */
  BSP_PB_Init(UserButton, BUTTON_MODE_EXTI);

  /* Setup sensors */
  GyroConfig();
  CompassConfig();
  InitPIDControllers();

  /* Init USB com */
  //initUSB();
//
//  /* TIM GPIO configuration */
//  TIM4_IOconfig();
//
//  /* Setup Timer 4 (used for PWM output)*/
//  TIM4_Setup();
//  /* Setup Timer 4 OC registers (for PWM output) */
//  TIM4_SetupOC();
//
//  /* Setup Timers 2 and 3 (used for PWM input) */
//  TIM2_Setup();
//  TIM3_Setup();
//  /* Setup and start PWM input (GPIO, NVIC settings) */
//  PWM_In_Setup();
//
//  /* Setup Timer 7 (used for program periodic execution) */
//  TIM7_Setup();
//  /* Setup and start Timer 7 for interrupt generation */
//  TIM7_SetupIRQ(); // NEEDS TO BE STARTED AFTER SENSOR CONFIG
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

  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED6);
  BSP_LED_Off(LED7);
  BSP_LED_Off(LED8);
  BSP_LED_Off(LED9);
  BSP_LED_Off(LED10);
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
