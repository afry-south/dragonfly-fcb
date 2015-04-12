/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_OUTPUT_H
#define __MOTOR_OUTPUT_H

#define MOTOR_OUT_SAMPLECLOCK (int) 24000000
#define TIM4_Period (int) 60000
void TIM4_IOconfig(void);
void TIM4_Setup(void);
void TIM4_SetupOC(void);

#endif /* __MOTOR_OUTPUT_H */
