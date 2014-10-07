/**
******************************************************************************
* @file    fcb/main.c
* @author  ÅF Dragonfly - Embedded Systems
* @version v. 0.0.1
* @date    2014-09-26
* @brief   Flight Control program for the ÅF Dragonfly quadcopter
******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor_output.h"
#include "control.h"
#include "sensors.h"
#include "RCinput.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions -----------------------------------------------*/


static void Init(void);

/**
* @brief  Main program.
* @param  None
* @retval None
*/
int main(void)
{
	/* At this stage the microcontroller clock setting is already configured,
	 * this is done through SystemInit() function which is called from startup
	 * file (startup_stm32f30x.s) before to branch to application main.
	 * To reconfigure the default setting of SystemInit() function, refer to
	 * system_stm32f30x.c file
	 */

	Init();

	// Infinite loop keeps the program alive.
	while (1);
}

static void Init(void)
{
	/* Setup sensors */
	GyroConfig();
	CompassConfig();
	// TODO Calibrate sensors (calculate drift while standing still and level)

	/* TIM GPIO configuration */
	TIM4_IOconfig();

	/* Setup Timer 4 (used for PWM output)*/
	TIM4_Setup();
	/* Setup Timer 4 OC registers (for PWM output) */
	TIM4_SetupOC();

	/* Setup Timer 2 (used for PWM input) */
	TIM2_Setup();
	/* Setup and start PWM input (GPIO, NVIC settings) */
	PWM_In_Setup();
	// TODO Calibrate RC input (min, max, midpoint) and map to according position and angle references
	// TODO Failsafe - what happens when no RC signal is received?

	/* Setup Timer 3 (used for program periodic execution) */
	TIM3_Setup();
	/* Setup and start Timer 3 for interrupt generation */
	TIM3_SetupIRQ(); // NEEDS TO BE STARTED AFTER SENSOR CONFIG
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
