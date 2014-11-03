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
#include "com.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions -----------------------------------------------*/
static void Init(void);
static void InitLEDs(void);

/* TODO bodyZVelocity calc? Rotate from roll/pitch/yaw estimates */
/* TODO Refine sensor settings and algorithm (extended Kalman? Kalman? Quaternions?) */
/* TODO Port to STM32F4 (it has SPI (faster than I2C!) for accelerometer) and faster CPU - maybe later */
/* TODO Accelerometer calibration using g and axis rotation, use mean function and reset if deviation too large (+/- 2%?) */
/* TODO dynamic h / dt in sensor integration and controller? (measure with GetCounter()) */
/* TODO Calibrate RC input (min, max, midpoint) and map to according position and angle references */
/* TODO Failsafe - what happens when no RC signal is received? Check receiver PWM - does it keep outputting the same after transmitter shutdown? */
/* TODO Use on-board LEDs to indicate calibration, warnings, modes etc. */
/* TODO PWM input chan 5 chan 6 (what controller input maps to chan 6?) - set mode (manual / control / autonomous / shutdown) */
/* TODO Better identify drag coefficient (for yaw control allocation) and also thrust coefficient - experiment setup needed */
/* TODO If STM32F3Discovery not placed in middle of quadcopter, translate sensor rotations? - wait until FCB has been mounted, then measure distances */
/* TODO Control integration anti-windup */
/* TODO Control bumpless transfer between modes */
/* TODO Flight modes and control performance settings (slow, normal, aggressive) */
/* TODO Trajectory (from x, y, z and heading refs) and hold position at destinations */
/* TODO Calibration reset if not satisfactory */
/* TODO Memory for storing settings and logging data (Use flash memory / SD card) */
/* TODO Interface with PC for setup (USB connection): Virtual COM port CDC communication established */
/* TODO Detect initial/take-off attitude (use gravity direction) */
/* TODO GetBodyAttitude etc. also updates, make separate update functions in sensors.c */
/* TODO Arm motors (both sticks bottom left (within 95% of min values) */
/* TODO Proximity sensors ADC */
/* TODO Glue pistol on stripboard bottom connections */

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
	/* Init on-board LEDs */
	InitLEDs();

	/* Setup sensors */
	GyroConfig();
	CompassConfig();

	/* Init USB com */
	//initUSB();

	/* Config priority grouping setting */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

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

	/* Setup Timer 3 (used for program periodic execution) */
	TIM3_Setup();
	/* Setup and start Timer 3 for interrupt generation */
	TIM3_SetupIRQ(); // NEEDS TO BE STARTED AFTER SENSOR CONFIG
}

static void InitLEDs(void)
{
	/* Initialize LEDs and User Button available on STM32F3-Discovery board */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED7);
	STM_EVAL_LEDInit(LED8);
	STM_EVAL_LEDInit(LED9);
	STM_EVAL_LEDInit(LED10);

	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);
	STM_EVAL_LEDOff(LED7);
	STM_EVAL_LEDOff(LED8);
	STM_EVAL_LEDOff(LED9);
	STM_EVAL_LEDOff(LED10);
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
