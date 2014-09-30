#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "stm32f30x_it.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_misc.h"

uint16_t GetPWM_CCR(float dutycycle);
void TIM3_IRQHandler(void);
void TIM3_Setup(void);
void TIM3_SetupIRQ(void);
