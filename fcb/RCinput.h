typedef struct
{
	float PWM_Time1;
	float PWM_Time2;
	float PWM_Time3;
	float PWM_Time4;
	float PWM_Time5;
	float PWM_Time6;
}PWM_TimeTypeDef;


volatile uint16_t PWM_IN_PeriodTicks[4];
volatile uint16_t PWM_IN_DutyCycleTicks[4];
void TIM2_Setup(void);
void PWM_In_Setup(void);
void TIM2_IRQHandler(void);
void GetPWMInputTimes(PWM_TimeTypeDef *PWM_Time);
