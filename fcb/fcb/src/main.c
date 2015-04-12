/******************************************************************************
 * @file    fcb/main.c
 * @author  ÅF Dragonfly - Daniel Nilsson and Daniel Stenberg, Embedded Systems
 * @version v. 0.0.2
 * @date    2015-04-12
 * @brief   Flight Control program for the ÅF Dragonfly quadcopter
 *
 * @I/O pin mapping overview:
 * PA.00 USER pushbutton
 * PA.05 SPI1_SCK (L3GD20 Gyroscope)
 * PA.06 SPI1_MISO (L3GD20 Gyroscope)
 * PA.07 SPI1_MOSI (L3GD20 Gyroscope)
 * PA.11 USBDM
 * PA.12 USBDP
 * PA.13 SWDAT
 * PA.14 SWCLK
 *
 * PB.03 TRACESWO
 * PB.04 TIM3_CH1 (Receiver PWM input)
 * PB.05 TIM3_CH1 (Receiver PWM input)
 * PB.06 I2C1_SCL (LSM303 Accelerometer/Magnetometer)
 * PB.07 I2C1_SDA (LSM303 Accelerometer/Magnetometer)
 *
 * PC.14 OSC32_IN
 * PC.15 OSC32_OUT
 *
 * PD.03 TIM2_CH1 (Receiver PWM input)
 * PD.04 TIM2_CH2 (Receiver PWM input)
 * PD.06 TIM2_CH4 (Receiver PWM input)
 * PD.07 TIM2_CH3 (Receiver PWM input)
 * PD.12 TIM4_CH1 (ESC/Motor PWM output)
 * PD.13 TIM4_CH2 (ESC/Motor PWM output)
 * PD.14 TIM4_CH3 (ESC/Motor PWM output)
 * PD.15 TIM4_CH4 (ESC/Motor PWM output)
 *
 * PE.00 L3GD20 INT1 (Interrupt 1)
 * PE.01 L3GD20 DRDY/INT2 (Data ready/interrupt 2)
 * PE.02 LSM303 DRDY (Data ready)
 * PE.03 L3GD20 CS_I2C/SPI (Chip select I2C/SPI)
 * PE.04 LSM303 INT1 (Interrupt 1)
 * PE.05 LSM303 INT2 (Interrupt 2)
 * PE.08 LD4 (BLUE LED)
 * PE.09 LD3 (RED LED)
 * PE.10 LD5 (ORANGE LED)
 * PE.11 LD7 (GREEN LED)
 * PE.12 LD9 (BLUE LED)
 * PE.13 LD10 (RED LED)
 * PE.14 LD8 (ORANGE LED)
 * PE.15 LD6 (GREEN LED)
 ******************************************************************************/

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
static __IO uint32_t MilliDelay;
__IO uint32_t UserButtonPressed = 0;

/* Private functions -----------------------------------------------*/
static void
Init_System(void);
static void
Init_LEDs(void);

/* TODO MOVE ALL THIS TO SOME SORT OF TICKET/ISSUE HANDLING SYSTEM IN ONE/TFS */
/* TODO Move IRQ handlers to stm32f30x_it.c */
/* TODO --> Calculation of velocity, especially vertical. Rotate from roll/pitch/yaw estimates and use better accelerometer calibration and filtering */
/* TODO --> Refine sensor settings and algorithm (extended Kalman? Kalman? Quaternions?) */
/* TODO --> Accelerometer calibration using g and axis rotation, use mean function and scale to g */
/* --> Suggestion: TODO Setup interrupt routine/DMA to collect sensor updates (since I2C is slow and has to wait alot) */
/* Suggestion: TODO dynamic time step h in sensor integration and controller? (measure with GetCounter()) */
/* TODO Calibrate RC input (min, max, midpoint for each stick) and map to according position and angle references (account for interval and midpoint offsets) */
/* Suggestion: TODO Better identify drag coefficient (for yaw control allocation) and also thrust coefficient - experiment setup needed */
/* TODO If STM32F3Discovery not placed in middle of quadcopter, translate sensor rotations? - wait until FCB has been mounted, then measure distances */
/* TODO Control integration anti-windup */
/* TODO Control bumpless transfer between control modes */
/* Suggestion: TODO Flight modes and control performance settings (slow, normal, aggressive) */
/* TODO Trajectory generation (from x, y, z and heading/yaw refs) and hold position at destinations (velocity/positional controller transfer) */
/* TODO Calibration reset if not satisfactory */
/* TODO Memory for storing settings and logging data (Use flash memory (EEPROM emulation) / SD card) */
/* TODO Interface with PC for setup (USB connection): Virtual COM port CDC communication established */
/* Suggestion: TODO Arm motors procedure (both sticks bottom left within 95% of min values for 10 seconds before motors can be used) */
/* Suggestion: TODO Glue pistol on breadboard bottom connections */
/* --> TODO Calibration temporarily set to true with some offset values */
/* Suggestion: TODO Re-check execution time using GPIO set and reset bits */
/* Suggestion: TODO cos and sin of pitch, roll, yaw performed repeatedly - store them once for each iteration. Use lookup-table for sin/cos/tan? */
/* Suggestion: TODO Barometer altimeter, proximity sensors, voltage sensor to monitor battery level or can the ~5V onboard be monitored? ADC? PVD? */
/* TODO Reads acc/magn and gyro sensors through interrupt routines (Data ready/DRDY interrupts), setup EXTI for pins. DMA reading beneficial? */
/* Suggestion: Make use of an RTOS? There is support in CMSIS for threads, semaphores etc */
/* Review header file and defines to prevent recursive inclusion */

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

  Init_System();

  // Infinite loop keeps the program alive.
  while (1)
    ;
}

static void Init_System(void)
{
  /* Init on-board LEDs */
  Init_LEDs();

  /* Setup sensors */
  GyroConfig();
  CompassConfig();
  InitPIDControllers();

  /* Reset UserButton_Pressed variable */
  UserButtonPressed = 0x00;

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

  /* Setup Timers 2 and 3 (used for PWM input) */
  TIM2_Setup();
  TIM3_Setup();
  /* Setup and start PWM input (GPIO, NVIC settings) */
  PWM_In_Setup();

  /* Setup Timer 7 (used for program periodic execution) */
  TIM7_Setup();
  /* Setup and start Timer 7 for interrupt generation */
  TIM7_SetupIRQ(); // NEEDS TO BE STARTED AFTER SENSOR CONFIG
}

static void Init_LEDs(void)
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

__IO uint32_t GetUserButton(void)
{
  return UserButtonPressed;
}

void ResetUserButton(void)
{
  UserButtonPressed = 0x00;
}

/** Delay
 * @brief  Inserts a delay time in milliseconds (0.001 s).
 * @param  mTime: specifies the delay time length, in milliseconds.
 */
void Delay(uint32_t mTime)
{
  MilliDelay = mTime;
  while (MilliDelay != 0)
    ;
}

/** MilliDelay_Decrement
 * @brief  Decrements the millisecond timer variables.
 */
void TimingDelay_Decrement(void)
{
  if (MilliDelay != 0x00)
    MilliDelay--;
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
