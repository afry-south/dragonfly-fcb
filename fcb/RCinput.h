typedef struct
{
	float Throttle;
	float Aileron;
	float Elevator;
	float Rudder;
	float Gear;
	float Auxiliary;
}PWMRC_TimeTypeDef;

void TIM2_Setup(void);
void TIM3_Setup(void);
void PWM_In_Setup(void);
void TIM2_IRQHandler(void);
void GetPWMInputTimes(PWMRC_TimeTypeDef *PWM_Time);
char CheckRCConnection(void);
float GetRCmin(void);
float GetRCmid(void);
float GetRCmax(void);
